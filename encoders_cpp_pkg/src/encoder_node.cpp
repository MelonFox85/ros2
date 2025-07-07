#include "encoder_node.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

EncoderNode::EncoderNode() : Node("encoder_node"), chip_(GPIO_CHIP_NAME, gpiod::chip::OPEN_BY_NAME) {
    RCLCPP_INFO(this->get_logger(), "Initializing Encoder Node with 4x Decoding and Speed Filtering");

    this->declare_parameter<int>("left_encoder_pin_a", 24);
    this->declare_parameter<int>("left_encoder_pin_b", 23);
    this->declare_parameter<int>("right_encoder_pin_a", 8);
    this->declare_parameter<int>("right_encoder_pin_b", 25);
    this->declare_parameter<int>("ppr", 13);
    this->declare_parameter<int>("gear_ratio", 30);
    this->declare_parameter<double>("publish_rate_hz", 400.0);
    this->declare_parameter<int>("filter_window_size", 15);

    left_encoder_pin_a_ = this->get_parameter("left_encoder_pin_a").as_int();
    left_encoder_pin_b_ = this->get_parameter("left_encoder_pin_b").as_int();
    right_encoder_pin_a_ = this->get_parameter("right_encoder_pin_a").as_int();
    right_encoder_pin_b_ = this->get_parameter("right_encoder_pin_b").as_int();
    ppr_ = this->get_parameter("ppr").as_int();
    gear_ratio_ = this->get_parameter("gear_ratio").as_int();
    filter_window_size_ = this->get_parameter("filter_window_size").as_int();

    double publish_rate_hz = this->get_parameter("publish_rate_hz").as_double();
    publish_interval_seconds_ = 1.0 / (publish_rate_hz > 0 ? publish_rate_hz : 400.0);

    events_per_wheel_revolution_ = static_cast<double>(4 * ppr_ * gear_ratio_);

    if (events_per_wheel_revolution_ == 0) {
        RCLCPP_ERROR(this->get_logger(), "Events per wheel revolution is zero. Check PPR and gear_ratio.");
        throw std::runtime_error("Events per wheel revolution is zero.");
    }
    RCLCPP_INFO(this->get_logger(), "Using 4x decoding. Calculated events_per_wheel_revolution: %.2f", events_per_wheel_revolution_);
    RCLCPP_INFO(this->get_logger(), "Using moving average filter with window size: %d", filter_window_size_);

    left_encoder_ticks_.store(0);
    right_encoder_ticks_.store(0);
    running_.store(true);

    try {
        initialize_gpio();
    } catch (const std::exception& e) {
        RCLCPP_FATAL(this->get_logger(), "Failed to initialize GPIO: %s", e.what());
        throw;
    }

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(10);
    wheel_speeds_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("encoders_data/wheel_speeds", sensor_qos);

    timer_ = this->create_wall_timer(
        std::chrono::duration<double>(publish_interval_seconds_),
        std::bind(&EncoderNode::calculate_and_publish_speed, this));

    left_speed_buffer_.assign(filter_window_size_, 0.0);
    right_speed_buffer_.assign(filter_window_size_, 0.0);

    RCLCPP_INFO(this->get_logger(), "Starting GPIO event threads...");
    left_encoder_thread_ = std::thread(&EncoderNode::gpio_event_loop, this,
                                       std::ref(line_left_a_), std::ref(line_left_b_),
                                       std::ref(left_encoder_ticks_), "Left");
    right_encoder_thread_ = std::thread(&EncoderNode::gpio_event_loop, this,
                                        std::ref(line_right_a_), std::ref(line_right_b_),
                                        std::ref(right_encoder_ticks_), "Right");

    RCLCPP_INFO(this->get_logger(), "Encoder node initialized successfully.");
}

EncoderNode::~EncoderNode() {
    RCLCPP_INFO(this->get_logger(), "Shutting down Encoder node...");
    running_.store(false);
    if (left_encoder_thread_.joinable()) { left_encoder_thread_.join(); }
    if (right_encoder_thread_.joinable()) { right_encoder_thread_.join(); }
    RCLCPP_INFO(this->get_logger(), "Encoder node shut down successfully.");
}

void EncoderNode::initialize_gpio() {
    RCLCPP_INFO(this->get_logger(), "Requesting GPIO lines for 4x decoding...");
    line_left_a_ = chip_.get_line(left_encoder_pin_a_);
    line_left_b_ = chip_.get_line(left_encoder_pin_b_);
    line_left_a_.request({"enc_L_A", gpiod::line_request::EVENT_BOTH_EDGES, 0});
    line_left_b_.request({"enc_L_B", gpiod::line_request::EVENT_BOTH_EDGES, 0});

    line_right_a_ = chip_.get_line(right_encoder_pin_a_);
    line_right_b_ = chip_.get_line(right_encoder_pin_b_);
    line_right_a_.request({"enc_R_A", gpiod::line_request::EVENT_BOTH_EDGES, 0});
    line_right_b_.request({"enc_R_B", gpiod::line_request::EVENT_BOTH_EDGES, 0});
    RCLCPP_INFO(this->get_logger(), "GPIO lines requested successfully.");
}

void EncoderNode::gpio_event_loop(gpiod::line& line_a, gpiod::line& line_b,
                                  std::atomic<long long>& ticks,
                                  const std::string& encoder_name) {
    RCLCPP_INFO(this->get_logger(), "Starting 4x GPIO event loop for %s encoder.", encoder_name.c_str());

    const int8_t state_table[16] = {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0
    };

    int prev_a = line_a.get_value();
    int prev_b = line_b.get_value();
    int prev_state = (prev_a << 1) | prev_b;

    // --- ИСПРАВЛЕНИЕ ЗДЕСЬ ---
    // 1. Создаем вектор линий
    std::vector<gpiod::line> lines_vec;
    // 2. Добавляем в него наши линии
    lines_vec.push_back(line_a);
    lines_vec.push_back(line_b);
    // 3. Создаем line_bulk из этого вектора
    gpiod::line_bulk bulk_lines(lines_vec);

    try {
        while (running_.load()) {
            gpiod::line_bulk event_bulk = bulk_lines.event_wait(std::chrono::milliseconds(100));
            if (!event_bulk.empty()) {
                int curr_a = line_a.get_value();
                int curr_b = line_b.get_value();
                int curr_state = (curr_a << 1) | curr_b;

                if (curr_state != prev_state) {
                    int8_t increment = state_table[(prev_state << 2) | curr_state];
                    ticks += increment;
                    prev_state = curr_state;
                }
            }
        }
    } catch (const std::exception& e) {
        if (running_.load()) {
            RCLCPP_ERROR(this->get_logger(), "Exception in %s encoder GPIO loop: %s", encoder_name.c_str(), e.what());
        }
    }
    RCLCPP_INFO(this->get_logger(), "Exiting GPIO event loop for %s encoder.", encoder_name.c_str());
}

void EncoderNode::calculate_and_publish_speed() {
    if (events_per_wheel_revolution_ == 0 || publish_interval_seconds_ == 0) {
        return;
    }

    long long current_left_ticks = left_encoder_ticks_.exchange(0);
    long long current_right_ticks = right_encoder_ticks_.exchange(0);

    double raw_left_speed_rps = static_cast<double>(current_left_ticks) / events_per_wheel_revolution_ / publish_interval_seconds_;
    double raw_right_speed_rps = static_cast<double>(current_right_ticks) / events_per_wheel_revolution_ / publish_interval_seconds_;

    left_speed_buffer_.push_back(raw_left_speed_rps);
    left_speed_buffer_.pop_front();
    right_speed_buffer_.push_back(raw_right_speed_rps);
    right_speed_buffer_.pop_front();

    double filtered_left_speed_rps = std::accumulate(left_speed_buffer_.begin(), left_speed_buffer_.end(), 0.0) / filter_window_size_;
    double filtered_right_speed_rps = std::accumulate(right_speed_buffer_.begin(), right_speed_buffer_.end(), 0.0) / filter_window_size_;

    auto speeds_msg = std_msgs::msg::Float32MultiArray();
    speeds_msg.data.push_back(static_cast<float>(filtered_left_speed_rps));
    speeds_msg.data.push_back(static_cast<float>(filtered_right_speed_rps));
    wheel_speeds_publisher_->publish(speeds_msg);
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<EncoderNode> node = nullptr;
    try {
        node = std::make_shared<EncoderNode>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        if (node) {
            RCLCPP_FATAL(node->get_logger(), "Unhandled exception: %s", e.what());
        } else {
            fprintf(stderr, "FATAL: Unhandled exception during creation: %s\n", e.what());
        }
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
