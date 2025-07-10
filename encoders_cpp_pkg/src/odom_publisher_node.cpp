/**
 * @file odom_publisher_node.cpp
 * @author GitHub Copilot (для Айрата)
 * @date 2025-07-09
 * @version 4.0
 * @brief ROS 2 узел для публикации одометрии с разделенными частотами расчета и публикации.
 *
 * @details
 * Этот узел реализует надежный механизм расчета и публикации одометрии,
 * оптимизированный для реальных робототехнических систем. Ключевая особенность
 * этой версии — разделение логики на два независимых по частоте процесса,
 * что позволяет достичь стабильности вычислений и высокой частоты публикации.
 *
 * Принцип работы:
 * 1.  **Потоки чтения энкодеров:** Два низкоуровневых потока непрерывно и
 *     асинхронно считывают данные с квадратурных энкодеров, используя драйвер
 *     ядра Linux ('gpio-rotary-encoder'). Это гарантирует, что ни один тик
 *     не будет пропущен.
 *
 * 2.  **Таймер вычислений (низкочастотный, например, 50 Гц):**
 *     - Срабатывает с заданной, относительно низкой частотой.
 *     - Считывает накопленные значения тиков.
 *     - Рассчитывает пройденное расстояние и изменение угла за прошедший
 *       (длинный) временной интервал. За счет большого интервала `dt`
 *       устраняется шум при дифференцировании.
 *     - Вычисляет стабильные линейную и угловую скорости.
 *     - Интегрирует позу (x, y, th).
 *     - Сохраняет вычисленные позу и скорости в переменных-членах класса.
 *
 * 3.  **Таймер публикаций (высокочастотный, 400 Гц):**
 *     - Срабатывает очень часто, как того требуют вышестоящие системы (планировщик, SLAM).
 *     - Не производит никаких расчетов.
 *     - Просто берет последние вычисленные (и стабильные) значения позы и скоростей
 *       и упаковывает их в сообщения `nav_msgs/msg/Odometry` и `geometry_msgs/msg/TransformStamped`.
 *     - Публикует данные в топик `/odom` и TF.
 *
 * @version 4.0 ИЗМЕНЕНИЯ:
 *  - Реализована архитектура с двумя таймерами: один для расчетов (низкая частота),
 *    другой для публикаций (высокая частота).
 *  - Удалена логика явной фильтрации, так как стабильность достигается за счет
 *    расчета скоростей на более длинном временном интервале.
 *  - Добавлен ROS-параметр `calculation_rate` для гибкой настройки частоты вычислений.
 *  - Полностью переработана структура кода и обновлены комментарии для ясности.
 */

// --- Стандартные библиотеки C++ ---
#include <string>
#include <vector>
#include <cmath>
#include <thread>
#include <atomic>
#include <chrono>
#include <stdexcept>
#include <memory>

// --- Заголовочные файлы ROS 2 ---
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

// --- Заголовочные файлы для работы с устройствами ввода Linux ---
#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <sys/select.h>

// =================================================================================
// ==                  КОНСТАНТЫ И КОНФИГУРАЦИЯ УЗЛА                             ==
// =================================================================================
namespace RobotConstants {
    const double WHEEL_RADIUS = 0.0325;
    const double WHEEL_BASE = 0.158;
    const double TICKS_PER_REVOLUTION = 1560.0;
    const int ODOM_PUBLISH_RATE_HZ = 400;
}

/**
 * @class OdomPublisherNode
 * @brief Основной класс узла, инкапсулирующий всю логику.
 */
class OdomPublisherNode : public rclcpp::Node {
public:
    OdomPublisherNode() : Node("odom_publisher_node"), x_(0.0), y_(0.0), th_(0.0), vx_(0.0), vz_(0.0), running_(true) {
        RCLCPP_INFO(this->get_logger(), "Инициализация узла одометрии 'OdomPublisherNode' v4.0...");

        // --- 1. Объявление и получение параметров ROS ---
        this->declare_parameter<std::string>("left_encoder_device", "/dev/input/event0");
        this->declare_parameter<std::string>("right_encoder_device", "/dev/input/event1");
        this->declare_parameter<int>("calculation_rate", 50);

        this->get_parameter("left_encoder_device", left_encoder_device_);
        this->get_parameter("right_encoder_device", right_encoder_device_);
        this->get_parameter("calculation_rate", calculation_rate_);

        RCLCPP_INFO(this->get_logger(), "Параметры узла:");
        RCLCPP_INFO(this->get_logger(), "  - Устройство левого энкодера: %s", left_encoder_device_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Устройство правого энкодера: %s", right_encoder_device_.c_str());
        RCLCPP_INFO(this->get_logger(), "  - Частота расчетов: %d Гц", calculation_rate_);
        RCLCPP_INFO(this->get_logger(), "  - Частота публикации: %d Гц", RobotConstants::ODOM_PUBLISH_RATE_HZ);

        // --- 2. ПРОВЕРКА ДОСТУПНОСТИ УСТРОЙСТВ ---
        if (!checkDevice(left_encoder_device_) || !checkDevice(right_encoder_device_)) {
            throw std::runtime_error("Сбой инициализации: одно или несколько устройств энкодеров недоступны.");
        }

        // --- 3. Инициализация паблишеров и TF-бродкастера ---
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

        // --- 4. Запуск потоков для чтения данных с энкодеров ---
        RCLCPP_INFO(this->get_logger(), "Запуск потоков для чтения энкодеров...");
        left_encoder_thread_ = std::thread(&OdomPublisherNode::encoderReadLoop, this, left_encoder_device_, std::ref(left_ticks_));
        right_encoder_thread_ = std::thread(&OdomPublisherNode::encoderReadLoop, this, right_encoder_device_, std::ref(right_ticks_));

        // --- 5. Создание двух независимых таймеров ---
        RCLCPP_INFO(this->get_logger(), "Создание таймеров для расчета и публикации...");
        calculation_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / calculation_rate_),
            std::bind(&OdomPublisherNode::calculateOdometry, this));
            
        publish_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / RobotConstants::ODOM_PUBLISH_RATE_HZ),
            std::bind(&OdomPublisherNode::publishData, this));
        
        // --- 6. Расчет производных констант ---
        distance_per_tick_ = (2.0 * M_PI * RobotConstants::WHEEL_RADIUS) / RobotConstants::TICKS_PER_REVOLUTION;
        
        // --- 7. Инициализация переменных состояния ---
        last_calculation_time_ = this->now();
        last_left_ticks_ = left_ticks_.load();
        last_right_ticks_ = right_ticks_.load();

        RCLCPP_INFO(this->get_logger(), "Узел одометрии успешно инициализирован и запущен.");
    }

    ~OdomPublisherNode() {
        RCLCPP_INFO(this->get_logger(), "Остановка узла одометрии...");
        running_.store(false);
        
        if (pipe(left_shutdown_pipe_) != -1) { close(left_shutdown_pipe_[1]); }
        if (pipe(right_shutdown_pipe_) != -1) { close(right_shutdown_pipe_[1]); }

        if (left_encoder_thread_.joinable()) {
            left_encoder_thread_.join();
        }
        if (right_encoder_thread_.joinable()) {
            right_encoder_thread_.join();
        }

        RCLCPP_INFO(this->get_logger(), "Узел одометрии полностью остановлен.");
    }

private:
    bool checkDevice(const std::string& device_path) {
        int fd = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
        if (fd == -1) {
            RCLCPP_FATAL(this->get_logger(), "КРИТИЧЕСКАЯ ОШИБКА: Не удалось открыть устройство '%s'. Причина: %s.",
                         device_path.c_str(), strerror(errno));
            if (errno == EACCES) {
                 RCLCPP_FATAL(this->get_logger(), "Ошибка доступа. Убедитесь, что ваш пользователь входит в группу 'input'. Выполните 'sudo usermod -a -G input $USER' и перезагрузитесь.");
            }
            return false;
        }
        close(fd);
        return true;
    }

    void encoderReadLoop(const std::string& device_path, std::atomic<long>& ticks_atomic) {
        int fd = open(device_path.c_str(), O_RDONLY);
        if (fd == -1) return;
        
        int shutdown_pipe[2];
        if (pipe(shutdown_pipe) == -1) { close(fd); return; }
        
        if (device_path == left_encoder_device_) {
            left_shutdown_pipe_[0] = shutdown_pipe[0]; left_shutdown_pipe_[1] = shutdown_pipe[1];
        } else {
            right_shutdown_pipe_[0] = shutdown_pipe[0]; right_shutdown_pipe_[1] = shutdown_pipe[1];
        }

        struct input_event ev;
        while (running_.load() && rclcpp::ok()) {
            fd_set fds;
            FD_ZERO(&fds);
            FD_SET(fd, &fds);
            FD_SET(shutdown_pipe[0], &fds);

            int max_fd = std::max(fd, shutdown_pipe[0]);
            select(max_fd + 1, &fds, NULL, NULL, NULL);

            if (!running_.load()) break;
            if (FD_ISSET(shutdown_pipe[0], &fds)) break; 

            if (FD_ISSET(fd, &fds)) {
                ssize_t bytes = read(fd, &ev, sizeof(struct input_event));
                if (bytes == sizeof(struct input_event) && ev.type == EV_REL) {
                    ticks_atomic += ev.value;
                } else if (bytes <= 0) {
                    break;
                }
            }
        }
        close(fd);
        close(shutdown_pipe[0]);
        close(shutdown_pipe[1]);
    }

    /**
     * @brief Коллбэк низкочастотного таймера. Выполняет все расчеты.
     */
    void calculateOdometry() {
        long current_left_ticks = left_ticks_.load();
        long current_right_ticks = right_ticks_.load();
        rclcpp::Time current_time = this->now();

        double dt = (current_time - last_calculation_time_).seconds();
        if (dt <= 1e-6) {
            return;
        }

        long delta_left = current_left_ticks - last_left_ticks_;
        long delta_right = current_right_ticks - last_right_ticks_;

        delta_right = -delta_right;

        double dist_left = delta_left * distance_per_tick_;
        double dist_right = delta_right * distance_per_tick_;

        double delta_dist = (dist_left + dist_right) / 2.0;
        double delta_th = (dist_right - dist_left) / RobotConstants::WHEEL_BASE;

        // Рассчитываем и сохраняем скорости для публикации
        vx_ = delta_dist / dt;
        vz_ = delta_th / dt;
        
        // Интегрируем позу
        x_ += delta_dist * cos(th_ + delta_th / 2.0);
        y_ += delta_dist * sin(th_ + delta_th / 2.0);
        th_ += delta_th;

        // Сохраняем состояние для следующей итерации расчета
        last_left_ticks_ = current_left_ticks;
        last_right_ticks_ = current_right_ticks;
        last_calculation_time_ = current_time;
    }

    /**
     * @brief Коллбэк высокочастотного таймера. Выполняет только публикацию.
     */
    void publishData() {
        rclcpp::Time current_time = this->now();

        // --- ПУБЛИКАЦИЯ TF ---
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        t.transform.translation.x = x_;
        t.transform.translation.y = y_;
        t.transform.translation.z = 0.0;
        t.transform.rotation.z = sin(th_ / 2.0);
        t.transform.rotation.w = cos(th_ / 2.0);
        tf_broadcaster_->sendTransform(t);

        // --- ПУБЛИКАЦИЯ ODOMETRY ---
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.orientation = t.transform.rotation;
        
        // Публикуем последние рассчитанные стабильные скорости
        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.angular.z = vz_;

        odom_pub_->publish(odom);
    }

    // --- Переменные-члены класса ---
    // Состояние робота (поза и скорости)
    double x_, y_, th_; // Поза
    double vx_, vz_;   // Скорости

    // Переменные для расчета одометрии
    long last_left_ticks_, last_right_ticks_;
    rclcpp::Time last_calculation_time_;
    
    // Атомарные счетчики тиков, обновляемые из потоков
    std::atomic<long> left_ticks_{0};
    std::atomic<long> right_ticks_{0};
    
    // Потоки для чтения энкодеров
    std::thread left_encoder_thread_;
    std::thread right_encoder_thread_;

    // Механизм корректного завершения
    std::atomic<bool> running_;
    int left_shutdown_pipe_[2];
    int right_shutdown_pipe_[2];
    
    // Объекты ROS
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr calculation_timer_;
    rclcpp::TimerBase::SharedPtr publish_timer_;
    
    // Параметры
    std::string left_encoder_device_;
    std::string right_encoder_device_;
    int calculation_rate_;
    
    // Константы
    double distance_per_tick_;
};
 
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<OdomPublisherNode>();
        rclcpp::spin(node);
    } catch (const std::runtime_error & e) {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Критическая ошибка при инициализации узла: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
