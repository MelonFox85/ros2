#ifndef ENCODER_NODE_HPP_
#define ENCODER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32_multi_array.hpp"
#include <gpiod.hpp>
#include <string>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <deque>
#include <numeric>

const std::string GPIO_CHIP_NAME = "gpiochip0";

/**
 * @class EncoderNode
 * @brief Узел ROS 2 для считывания и фильтрации данных с квадратурных энкодеров.
 *
 * @details
 * Этот узел отвечает за взаимодействие с GPIO-пинами Raspberry Pi, к которым
 * подключены энкодеры двигателей.
 *
 * Основные задачи узла:
 * 1.  **Инициализация GPIO**: Настраивает пины энкодеров на чтение событий.
 * 2.  **4x Квадратурное декодирование**: В отдельных потоках для каждого энкодера
 *     обрабатывает события (фронт/спад) на **обоих** каналах (A и B). Это позволяет
 *     получать в 4 раза больше отсчетов на один импульс (PPR), обеспечивая
 *     максимально возможное разрешение.
 * 3.  **Фильтрация шума**: Применяет фильтр скользящего среднего для сглаживания
 *     вычисленной скорости, убирая шум квантования.
 * 4.  **Расчет и публикация скорости**: Периодически отправляет отфильтрованные
 *     скорости колес (в об/с) в топик ROS 2.
 *
 * @param ppr - Pulses Per Revolution. Количество импульсов на один оборот вала *мотора*.
 * @param filter_window_size - Размер окна для фильтра скользящего среднего.
 */
class EncoderNode : public rclcpp::Node {
public:
    EncoderNode();
    ~EncoderNode();

private:
    void initialize_gpio();

    /**
     * @brief Основной цикл обработки событий GPIO для одного энкодера.
     *
     * Эта функция выполняется в отдельном потоке. Она ожидает прерывания
     * и, зная предыдущее и текущее состояние пинов A и B, точно определяет
     * направление вращения и инкрементирует счетчик.
     */
    void gpio_event_loop(gpiod::line& line_a, gpiod::line& line_b,
                         std::atomic<long long>& ticks,
                         const std::string& encoder_name);

    void calculate_and_publish_speed();

    // --- Аппаратные ресурсы ---
    gpiod::chip chip_;
    gpiod::line line_left_a_;
    gpiod::line line_left_b_;
    gpiod::line line_right_a_;
    gpiod::line line_right_b_;

    // --- Параметры из ROS ---
    int left_encoder_pin_a_;
    int left_encoder_pin_b_;
    int right_encoder_pin_a_;
    int right_encoder_pin_b_;
    int ppr_;
    int gear_ratio_;
    double publish_interval_seconds_;

    // --- Переменные для подсчета тиков ---
    std::atomic<long long> left_encoder_ticks_;
    std::atomic<long long> right_encoder_ticks_;

    // --- Переменные для фильтрации ---
    int filter_window_size_;
    std::deque<double> left_speed_buffer_;
    std::deque<double> right_speed_buffer_;

    // --- Константа для расчетов ---
    double events_per_wheel_revolution_;

    // --- Компоненты ROS ---
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_speeds_publisher_;

    // --- Потоки ---
    std::thread left_encoder_thread_;
    std::thread right_encoder_thread_;
    std::atomic<bool> running_;
};

#endif // ENCODER_NODE_HPP_
