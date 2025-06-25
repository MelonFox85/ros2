#ifndef ENCODER_NODE_HPP_
#define ENCODER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32_multi_array.hpp" // Используем Float32MultiArray
#include <gpiod.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector> // Для std::vector в Float32MultiArray

// Имя GPIO чипа для Raspberry Pi (может быть gpiochip0 или gpiochip4)
// Проверить можно командой `gpioinfo`
const std::string GPIO_CHIP_NAME = "gpiochip0";

class EncoderNode : public rclcpp::Node {
public:
    EncoderNode();
    ~EncoderNode();

private:
    void initialize_gpio();
    void gpio_event_loop(gpiod::line& line_a, gpiod::line& line_b,
                         std::atomic<long long>& ticks,
                         const std::string& encoder_name,
                         std::chrono::steady_clock::time_point& last_event_time);
    void calculate_and_publish_speed();

    // GPIO
    gpiod::chip chip_;
    gpiod::line line_left_a_;
    gpiod::line line_left_b_;
    gpiod::line line_right_a_;
    gpiod::line line_right_b_;

    // Параметры ROS
    int left_encoder_pin_a_;
    int left_encoder_pin_b_;
    int right_encoder_pin_a_;
    int right_encoder_pin_b_;
    int ppr_; // pulses per revolution (от одного из каналов энкодера)
    int gear_ratio_;
    double publish_interval_seconds_;
    // frame_id_ больше не используется напрямую в Float32MultiArray,
    // но может быть полезен для других целей или если ты решишь обернуть сообщение.
    // std::string frame_id_; 

    // Переменные для подсчета тиков
    std::atomic<long long> left_encoder_ticks_;
    std::atomic<long long> right_encoder_ticks_;

    // Переменные для debounce
    std::chrono::microseconds debounce_duration_us_;
    std::chrono::steady_clock::time_point left_last_event_time_;
    std::chrono::steady_clock::time_point right_last_event_time_;

    // Константы
    double events_per_wheel_revolution_; // Количество событий (изменений состояний) на полный оборот колеса

    // ROS элементы
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speeds_publisher_;

    // Потоки для обработки GPIO
    std::thread left_encoder_thread_;
    std::thread right_encoder_thread_;
    std::atomic<bool> running_;
};

#endif // ENCODER_NODE_HPP_
