#include "encoder_node.hpp"
// #include "encoders_pkg/msg/wheel_speeds.hpp"; // Больше не используется
#include "std_msgs/msg/float32_multi_array.hpp" // Используем стандартное сообщение
// #include "std_msgs/msg/header.hpp"; // Не используется напрямую в Float32MultiArray

EncoderNode::EncoderNode() : Node("encoder_node"), chip_(GPIO_CHIP_NAME, gpiod::chip::OPEN_BY_NAME) {
    RCLCPP_INFO(this->get_logger(), "Initializing Encoder Node with std_msgs/Float32MultiArray");

    this->declare_parameter<int>("left_encoder_pin_a", 24);
    this->declare_parameter<int>("left_encoder_pin_b", 23);
    this->declare_parameter<int>("right_encoder_pin_a", 8);
    this->declare_parameter<int>("right_encoder_pin_b", 25);
    this->declare_parameter<int>("ppr", 13); // pulses per revolution (для одного канала энкодера)
    this->declare_parameter<int>("gear_ratio", 30);
    this->declare_parameter<double>("publish_rate_hz", 400.0);
    this->declare_parameter<int>("debounce_delay_us", 300);
    // this->declare_parameter<std::string>("frame_id", "base_link"); // frame_id не используется в Float32MultiArray

    left_encoder_pin_a_ = this->get_parameter("left_encoder_pin_a").as_int();
    left_encoder_pin_b_ = this->get_parameter("left_encoder_pin_b").as_int();
    right_encoder_pin_a_ = this->get_parameter("right_encoder_pin_a").as_int();
    right_encoder_pin_b_ = this->get_parameter("right_encoder_pin_b").as_int();
    ppr_ = this->get_parameter("ppr").as_int();
    gear_ratio_ = this->get_parameter("gear_ratio").as_int();
    // frame_id_ = this->get_parameter("frame_id").as_string();

    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    if (publish_rate_hz <= 0) {
        RCLCPP_WARN(this->get_logger(), "Publish rate must be positive. Defaulting to 400 Hz.");
        publish_rate_hz = 400.0;
    }
    publish_interval_seconds_ = 1.0 / publish_rate_hz;

    int debounce_us = this->get_parameter("debounce_delay_us").as_int();
    debounce_duration_us_ = std::chrono::microseconds(debounce_us);

    // Квадратурный энкодер дает 4 изменения состояния на один "pulse" (PPR)
    // events_per_wheel_revolution_ = static_cast<double>(4 * ppr_ * gear_ratio_);
    // Если ppr_ - это количество "щелей" или "импульсов" на оборот вала мотора,
    // и ты считаешь оба фронта на одном канале (или один фронт на двух каналах для направления),
    // то events_per_wheel_revolution_ может быть 2 * ppr_ * gear_ratio_ или 4 * ppr_ * gear_ratio_
    // В твоем предыдущем коде было 2 * ppr_ * gear_ratio_. Оставим так для консистентности.
    // Уточни, что именно означает твой PPR, чтобы правильно это рассчитать.
    // Если PPR - это количество полных циклов сигнала A (или B) на оборот вала мотора,
    // то для квадратурного декодирования (с учетом обоих каналов A и B и обоих фронтов)
    // это будет 4 * PPR_мотора * gear_ratio_ событий на оборот колеса.
    // Если PPR - это уже количество событий, которые ты ожидаешь считать на валу мотора, то просто * gear_ratio.
    // Для простоты, если твой ppr_ = 13 означает 13 полных циклов на одном канале,
    // то для квадратурного декодирования это будет 4 * 13 = 52 событий на оборот вала мотора.
    // Тогда events_per_wheel_revolution_ = 4.0 * ppr_ * gear_ratio_;
    events_per_wheel_revolution_ = static_cast<double>(2 * ppr_ * gear_ratio_); // Как было в твоем коде

    if (events_per_wheel_revolution_ == 0) {
        RCLCPP_ERROR(this->get_logger(), "Events per wheel revolution is zero. Check PPR and gear_ratio.");
        throw std::runtime_error("Events per wheel revolution is zero.");
    }

    RCLCPP_INFO(this->get_logger(), "Encoder PPR (defined by user): %d, Gear Ratio: %d", ppr_, gear_ratio_);
    RCLCPP_INFO(this->get_logger(), "Calculated events_per_wheel_revolution (using 2xPPR rule): %.2f", events_per_wheel_revolution_);
    RCLCPP_INFO(this->get_logger(), "Publishing combined wheel speeds at %.2f Hz (Interval: %.4f s).",
                publish_rate_hz, publish_interval_seconds_);

    left_encoder_ticks_.store(0);
    right_encoder_ticks_.store(0);
    running_.store(true);

    auto now_steady = std::chrono::steady_clock::now();
    left_last_event_time_ = now_steady - debounce_duration_us_ - std::chrono::microseconds(1); // Гарантируем, что первое событие пройдет debounce
    right_last_event_time_ = now_steady - debounce_duration_us_ - std::chrono::microseconds(1);

    try {
        initialize_gpio();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize GPIO: %s", e.what());
        throw; // Перебрасываем исключение, чтобы main мог его поймать
    }

    wheel_speeds_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("encoders_data/wheel_speeds", 10);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(publish_interval_seconds_),
        std::bind(&EncoderNode::calculate_and_publish_speed, this));

    RCLCPP_INFO(this->get_logger(), "Starting GPIO event threads...");
    left_encoder_thread_ = std::thread(&EncoderNode::gpio_event_loop, this,
                                       std::ref(line_left_a_), std::ref(line_left_b_),
                                       std::ref(left_encoder_ticks_), "Left",
                                       std::ref(left_last_event_time_));
    right_encoder_thread_ = std::thread(&EncoderNode::gpio_event_loop, this,
                                        std::ref(line_right_a_), std::ref(line_right_b_),
                                        std::ref(right_encoder_ticks_), "Right",
                                        std::ref(right_last_event_time_));

    RCLCPP_INFO(this->get_logger(), "Encoder node initialized successfully.");
}

EncoderNode::~EncoderNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Encoder node...");
    running_.store(false);
    if (left_encoder_thread_.joinable()) {
        RCLCPP_INFO(this->get_logger(), "Joining left encoder thread...");
        left_encoder_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Left encoder thread joined.");
    }
    if (right_encoder_thread_.joinable()) {
        RCLCPP_INFO(this->get_logger(), "Joining right encoder thread...");
        right_encoder_thread_.join();
        RCLCPP_INFO(this->get_logger(), "Right encoder thread joined.");
    }
    // Освобождение линий GPIO (gpiod::line деструкторы сделают это автоматически при выходе из области видимости)
    RCLCPP_INFO(this->get_logger(), "Encoder node shut down successfully.");
}

void EncoderNode::initialize_gpio() {
    RCLCPP_INFO(this->get_logger(), "Accessing GPIO chip: '%s' (Name: '%s', Label: '%s').",
                GPIO_CHIP_NAME.c_str(), chip_.name().c_str(), chip_.label().c_str());

    RCLCPP_INFO(this->get_logger(), "Requesting GPIO lines...");
    // Левый энкодер
    line_left_a_ = chip_.get_line(left_encoder_pin_a_);
    line_left_b_ = chip_.get_line(left_encoder_pin_b_);
    line_left_a_.request({"encoder_left_a", gpiod::line_request::EVENT_BOTH_EDGES, 0}); // Канал A для событий
    line_left_b_.request({"encoder_left_b", gpiod::line_request::DIRECTION_INPUT, 0});  // Канал B для определения направления
    RCLCPP_INFO(this->get_logger(), "Left Encoder: PinA=%d (events), PinB=%d (input for direction).",
                left_encoder_pin_a_, left_encoder_pin_b_);

    // Правый энкодер
    line_right_a_ = chip_.get_line(right_encoder_pin_a_);
    line_right_b_ = chip_.get_line(right_encoder_pin_b_);
    line_right_a_.request({"encoder_right_a", gpiod::line_request::EVENT_BOTH_EDGES, 0}); // Канал A для событий
    line_right_b_.request({"encoder_right_b", gpiod::line_request::DIRECTION_INPUT, 0});  // Канал B для определения направления
    RCLCPP_INFO(this->get_logger(), "Right Encoder: PinA=%d (events), PinB=%d (input for direction).",
                right_encoder_pin_a_, right_encoder_pin_b_);
    RCLCPP_INFO(this->get_logger(), "GPIO lines requested successfully.");
}

void EncoderNode::gpio_event_loop(gpiod::line& line_a, gpiod::line& line_b,
                                  std::atomic<long long>& ticks,
                                  const std::string& encoder_name,
                                  std::chrono::steady_clock::time_point& last_event_time) {
    RCLCPP_INFO(this->get_logger(), "Starting GPIO event loop for %s encoder.", encoder_name.c_str());
    try {
        while (running_.load()) {
            // Ожидание события на линии A с таймаутом, чтобы цикл не блокировался навсегда
            if (line_a.event_wait(std::chrono::milliseconds(100))) { // Таймаут 100 мс
                auto current_time = std::chrono::steady_clock::now();

                // Читаем событие, чтобы очистить его из очереди линии, даже если отбрасываем из-за debounce
                gpiod::line_event event = line_a.event_read();

                if ((current_time - last_event_time) < debounce_duration_us_) {
                    // RCLCPP_DEBUG(this->get_logger(), "%s encoder debounce, skipping event.", encoder_name.c_str());
                    continue; // Пропускаем событие из-за дребезга
                }
                last_event_time = current_time; // Обновляем время последнего валидного события

                int val_b = line_b.get_value(); // Читаем состояние линии B для определения направления

                // Логика квадратурного энкодера
                // Это одна из возможных реализаций. Направление зависит от того,
                // какой канал (A или B) опережает другой.
                // Предположим:
                // Вращение вперед: A опережает B. Если A нарастает, B=0. Если A спадает, B=1.
                // Вращение назад: B опережает A. Если A нарастает, B=1. Если A спадает, B=0.

                if (event.event_type == gpiod::line_event::RISING_EDGE) { // Фронт на A
                    if (val_b == 0) { // B низкий
                        ticks++; // Вперед
                    } else { // B высокий
                        ticks--; // Назад
                    }
                } else if (event.event_type == gpiod::line_event::FALLING_EDGE) { // Спад на A
                    if (val_b == 1) { // B высокий
                        ticks++; // Вперед
                    } else { // B низкий
                        ticks--; // Назад
                    }
                }
                // RCLCPP_DEBUG(this->get_logger(), "%s encoder tick. Current ticks: %lld", encoder_name.c_str(), ticks.load());
            }
        }
    } catch (const std::system_error& e) {
        if (running_.load()){ // Логируем ошибку, только если узел еще должен работать
            RCLCPP_ERROR(this->get_logger(), "System error in %s encoder GPIO loop: %s (code: %d)", encoder_name.c_str(), e.what(), e.code().value());
        }
    } catch (const std::exception& e) {
         if (running_.load()){
            RCLCPP_ERROR(this->get_logger(), "Exception in %s encoder GPIO loop: %s", encoder_name.c_str(), e.what());
         }
    }
    RCLCPP_INFO(this->get_logger(), "Exiting GPIO event loop for %s encoder.", encoder_name.c_str());
}

void EncoderNode::calculate_and_publish_speed() {
    if (events_per_wheel_revolution_ == 0 || publish_interval_seconds_ == 0) {
        RCLCPP_WARN_ONCE(this->get_logger(), "events_per_wheel_revolution_ or publish_interval_seconds_ is zero, cannot calculate speed.");
        return;
    }

    // Забираем накопленные тики и сбрасываем счетчики
    long long current_left_ticks = left_encoder_ticks_.exchange(0);
    long long current_right_ticks = right_encoder_ticks_.exchange(0);

    // Расчет скорости в оборотах в секунду (rps)
    double left_wheel_speed_rps = static_cast<double>(current_left_ticks) / events_per_wheel_revolution_ / publish_interval_seconds_;
    double right_wheel_speed_rps = static_cast<double>(current_right_ticks) / events_per_wheel_revolution_ / publish_interval_seconds_;

    // Создаем и заполняем сообщение Float32MultiArray
    auto speeds_msg = std_msgs::msg::Float32MultiArray();

    // Опционально: можно добавить описание структуры данных (layout)
    // speeds_msg.layout.dim.emplace_back();
    // speeds_msg.layout.dim[0].label = "wheel_speeds_rps";
    // speeds_msg.layout.dim[0].size = 2; // Два значения: левое и правое колесо
    // speeds_msg.layout.dim[0].stride = 2;
    // speeds_msg.layout.data_offset = 0;

    speeds_msg.data.clear(); // На всякий случай, если объект используется повторно
    speeds_msg.data.push_back(static_cast<float>(left_wheel_speed_rps));
    speeds_msg.data.push_back(static_cast<float>(right_wheel_speed_rps));

    wheel_speeds_publisher_->publish(speeds_msg);

    // RCLCPP_DEBUG(this->get_logger(), "Published speeds: Left=%.4f rps, Right=%.4f rps",
    //             left_wheel_speed_rps, right_wheel_speed_rps);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<EncoderNode> node = nullptr;
    try {
        node = std::make_shared<EncoderNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        if (node) {
            RCLCPP_FATAL(node->get_logger(), "Unhandled exception during EncoderNode execution: %s", e.what());
        } else {
            // Если узел не создался, логгер узла недоступен
            fprintf(stderr, "FATAL: Unhandled exception during EncoderNode creation: %s\n", e.what());
        }
        rclcpp::shutdown(); // Важно корректно завершить работу ROS
        return 1;
    } catch (...) {
        if (node) {
            RCLCPP_FATAL(node->get_logger(), "Unknown unhandled exception in EncoderNode.");
        } else {
            fprintf(stderr, "FATAL: Unknown unhandled exception during EncoderNode creation.\n");
        }
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}

