/**
 * @file      pid_controller.cpp
 * @brief     Узел ПИД-регулятора для системы управления двухколесным самобалансирующимся роботом.
 * @author    Айрат (разработано с помощью Copilot)
 * @date      2025-06-30
 *
 * @section description Описание
 *
 * Данный узел реализует алгоритм ПИД-регулирования для стабилизации и управления движением
 * робота. Он состоит из трех независимых контуров регулирования:
 * 1.  **Контур балансировки**: Отвечает за поддержание вертикального положения платформы робота.
 * 2.  **Контур управления положением**: Отвечает за продольное перемещение робота (вперед/назад).
 * 3.  **Контур управления курсом (рысканием)**: Отвечает за повороты робота.
 *
 * Узел позволяет производить динамическую настройку всех коэффициентов регуляторов
 * через стандартные механизмы ROS 2, включая графические утилиты, такие как `rqt_reconfigure`.
 *
 * @section subscriptions Подписки
 *
 * - **Топик**: `/feedback`
 * - **Тип сообщения**: `std_msgs::msg::Float32MultiArray`
 * - **Описание**: Принимает вектор ошибки состояния `δ = cmd - state`.
 *
 * @section publications Публикации
 *
 * - **Топики**: `/motor_cmd/left`, `/motor_cmd/right`
 * - **Тип сообщения**: `std_msgs::msg::Float32`
 * - **Описание**: Публикует вычисленное управляющее воздействие в виде напряжения [В].
 *
 * - **Топик**: `/pid_debug`
 * - **Тип сообщения**: `std_msgs::msg::Float32MultiArray`
 * - **Описание**: Публикует отладочную информацию о вкладе компонентов ошибки.
 *
 * @section parameters Параметры
 *
 * Все параметры являются динамически реконфигурируемыми.
 * - `balance.kp`, `balance.ki`, `balance.kd` (double): Коэффициенты для контура балансировки.
 * - `position.kp`, `position.ki`, `position.kd` (double): Коэффициенты для контура положения.
 * - `yaw.kp`, `yaw.ki`, `yaw.kd` (double): Коэффициенты для контура рыскания.
 * - `max_voltage` (double): Максимальное (по модулю) напряжение, подаваемое на двигатели [В].
 * - `invert_left`, `invert_right` (bool): Флаги для инвертирования направления вращения.
 */
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

// Структура для хранения коэффициентов одного ПИД-регулятора
struct PIDGains {
    double kp = 0.0, ki = 0.0, kd = 0.0;
};

class PIDControllerNode : public rclcpp::Node
{
public:
    PIDControllerNode() : Node("pid_controller")
    {
        // --- Объявление параметров (разбито для поддержки rqt_reconfigure) ---
        auto desc_ro = rcl_interfaces::msg::ParameterDescriptor{};
        desc_ro.read_only = false; // Явно указываем, что параметры можно менять

        declare_parameter("balance.kp", 10.0, desc_ro);
        declare_parameter("balance.ki", 0.5, desc_ro);
        declare_parameter("balance.kd", 0.0, desc_ro); // Начальное значение для теста

        declare_parameter("position.kp", 0.0, desc_ro);
        declare_parameter("position.ki", 0.0, desc_ro);
        declare_parameter("position.kd", 0.0, desc_ro);

        declare_parameter("yaw.kp", 0.0, desc_ro);
        declare_parameter("yaw.ki", 0.0, desc_ro);
        declare_parameter("yaw.kd", 0.0, desc_ro);
        
        declare_parameter("max_voltage", 12.0, desc_ro);
        declare_parameter("invert_left", false, desc_ro);
        declare_parameter("invert_right", false, desc_ro);

        // Загрузка начальных значений в наши структуры
        load_all_parameters();

        // --- QoS, Подписчики и Издатели ---
        // --- ИЗМЕНЕНИЕ ЗДЕСЬ ---
        // Создаем QoS профиль, который сохраняет последнее сообщение для новых подписчиков.
        // Это аналог "latching" в ROS 1.
        auto motor_cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        motor_cmd_qos.transient_local(); // Устанавливаем Durability в TRANSIENT_LOCAL

        // QoS для подписки на feedback (можно оставить по умолчанию)
        rclcpp::QoS feedback_qos(rclcpp::KeepLast(1));

        feedback_sub_ = create_subscription<Float32MultiArray>("feedback", feedback_qos, std::bind(&PIDControllerNode::feedback_callback, this, std::placeholders::_1));
        
        // Применяем новый QoS к издателям команд моторов
        left_pub_ = create_publisher<Float32>("motor_cmd/left", motor_cmd_qos);
        right_pub_ = create_publisher<Float32>("motor_cmd/right", motor_cmd_qos);
        
        // Для отладки можно оставить обычный QoS
        debug_pub_ = create_publisher<Float32MultiArray>("pid_debug", feedback_qos);
        
        // --- Инициализация переменных ---
        integral_position_ = 0.0;
        integral_yaw_ = 0.0;
        debug_msg_.data.resize(6, 0.0f);

        // --- Регистрация обработчика изменения параметров ---
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PIDControllerNode::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Узел PID-регулятора запущен с QoS=TRANSIENT_LOCAL для команд моторов.");
    }

private:
    // Функция для первоначальной загрузки всех параметров
    void load_all_parameters() {
        this->get_parameter("balance.kp", balance_gains_.kp);
        this->get_parameter("balance.ki", balance_gains_.ki);
        this->get_parameter("balance.kd", balance_gains_.kd);
        this->get_parameter("position.kp", position_gains_.kp);
        this->get_parameter("position.ki", position_gains_.ki);
        this->get_parameter("position.kd", position_gains_.kd);
        this->get_parameter("yaw.kp", yaw_gains_.kp);
        this->get_parameter("yaw.ki", yaw_gains_.ki);
        this->get_parameter("yaw.kd", yaw_gains_.kd);
        this->get_parameter("max_voltage", max_voltage_);
        this->get_parameter("invert_left", invert_left_);
        this->get_parameter("invert_right", invert_right_);
    }

    // Обработчик для динамического изменения параметров
    rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto &param : parameters)
        {
            const auto &name = param.get_name();
            if (name == "balance.kp") balance_gains_.kp = param.as_double();
            else if (name == "balance.ki") balance_gains_.ki = param.as_double();
            else if (name == "balance.kd") balance_gains_.kd = param.as_double();
            else if (name == "position.kp") position_gains_.kp = param.as_double();
            else if (name == "position.ki") position_gains_.ki = param.as_double();
            else if (name == "position.kd") position_gains_.kd = param.as_double();
            else if (name == "yaw.kp") yaw_gains_.kp = param.as_double();
            else if (name == "yaw.ki") yaw_gains_.ki = param.as_double();
            else if (name == "yaw.kd") yaw_gains_.kd = param.as_double();
            else if (name == "max_voltage") max_voltage_ = param.as_double();
            else if (name == "invert_left") invert_left_ = param.as_bool();
            else if (name == "invert_right") invert_right_ = param.as_bool();
        }
        return result;
    }

    void feedback_callback(const Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.size() < 6) return;

        const double error_s = msg->data[0];
        const double error_v = msg->data[1];
        const double error_theta = msg->data[2];
        const double error_theta_dot = msg->data[3];
        const double error_psi = msg->data[4];
        const double error_psi_dot = msg->data[5];

        // --- ЗАЩИТА ОТ ПАДЕНИЯ ---
        // Если робот упал (угол наклона близок к 90 градусам), отключаем моторы.
        // 1.5 радиана ≈ 86 градусов.
        if (std::abs(error_theta) > 1.5) {
            left_pub_->publish(Float32().set__data(0.0f));
            right_pub_->publish(Float32().set__data(0.0f));
            // Сбрасываем интегральные суммы, чтобы не было "рывка" при подъеме
            integral_position_ = 0.0;
            integral_yaw_ = 0.0;
            return; // Прерываем дальнейшие вычисления
        }

        // --- 1. Регулятор баланса ---
        const double u_balance_p = balance_gains_.kp * error_theta;
        const double u_balance_d = balance_gains_.kd * error_theta_dot;
        const double u_balance = u_balance_p + u_balance_d;

        // --- 2. Регулятор положения ---
        integral_position_ += position_gains_.ki * error_s;
        const double u_position = position_gains_.kp * error_s + integral_position_ + position_gains_.kd * error_v;

        // --- 3. Регулятор поворота (рыскания) ---
        integral_yaw_ += yaw_gains_.ki * error_psi;
        const double u_yaw = yaw_gains_.kp * error_psi + integral_yaw_ + yaw_gains_.kd * error_psi_dot;

        // --- Суммирование и ограничение ---
        double v_right = u_balance + u_position + u_yaw;
        double v_left = u_balance + u_position - u_yaw;

        if (invert_left_) v_left = -v_left;
        if (invert_right_) v_right = -v_right;

        v_left = std::clamp(v_left, -max_voltage_, max_voltage_);
        v_right = std::clamp(v_right, -max_voltage_, max_voltage_);

        // --- Публикация команд и отладки ---
        left_pub_->publish(Float32().set__data(static_cast<float>(v_left)));
        right_pub_->publish(Float32().set__data(static_cast<float>(v_right)));
        
        debug_msg_.data[0] = static_cast<float>(position_gains_.kp * error_s + integral_position_);
        debug_msg_.data[1] = static_cast<float>(position_gains_.kd * error_v);
        debug_msg_.data[2] = static_cast<float>(u_balance_p);
        debug_msg_.data[3] = static_cast<float>(u_balance_d);
        debug_msg_.data[4] = static_cast<float>(yaw_gains_.kp * error_psi + integral_yaw_);
        debug_msg_.data[5] = static_cast<float>(yaw_gains_.kd * error_psi_dot);
        debug_pub_->publish(debug_msg_);
    }

    // Параметры
    PIDGains balance_gains_, position_gains_, yaw_gains_;
    double max_voltage_;
    bool invert_left_, invert_right_;

    // Состояния регуляторов
    double integral_position_;
    double integral_yaw_;

    // ROS компоненты
    rclcpp::Subscription<Float32MultiArray>::SharedPtr feedback_sub_;
    rclcpp::Publisher<Float32>::SharedPtr left_pub_;
    rclcpp::Publisher<Float32>::SharedPtr right_pub_;
    rclcpp::Publisher<Float32MultiArray>::SharedPtr debug_pub_;
    Float32MultiArray debug_msg_;
    OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PIDControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
