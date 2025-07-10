/**
 * @file      pid_controller.cpp
 * @brief     Узел ПИД-регулятора для системы управления двухколесным самобалансирующимся роботом.
 * @author    Айрат (разработано с помощью Copilot, доработано и исправлено)
 * @date      2025-07-08
 *
 * @section description Описание
 *
 * Данный узел реализует математически корректный дискретный ПИД-регулятор с защитой
 * от интегрального насыщения (Anti-Windup).
 * Он состоит из трех независимых контуров регулирования:
 * 1.  **Контур балансировки**: Отвечает за поддержание вертикального положения платформы робота.
 * 2.  **Контур управления положением**: Отвечает за продольное перемещение робота (вперед/назад).
 * 3.  **Контур управления курсом (рысканием)**: Отвечает за повороты робота.
 *
 * Ключевой особенностью этой версии является:
 * - Математически корректная реализация интегральной составляющей, которая учитывает
 *   реальное прошедшее время (`dt`) между итерациями цикла управления.
 * - Реализация механизма Anti-Windup, который предотвращает неконтролируемый рост
 *   интегральной суммы, когда выходной сигнал (напряжение) уже ограничен максимумом.
 *   Это критически важно для стабильности робота при больших возмущениях.
 *
 * @section subscriptions Подписки
 *
 * - **Топик**: `/feedback`
 * - **Тип сообщения**: `std_msgs::msg::Float32MultiArray`
 * - **Описание**: Принимает вектор ошибки состояния `δ = cmd - state`.
 *   [δ_s, δ_v, δ_θ, δ_θ̇, δ_ψ, δ_ψ̇]
 *
 * @section publications Публикации
 *
 * - **Топики**: `/motor_cmd/left`, `/motor_cmd/right`
 * - **Тип сообщения**: `std_msgs::msg::Float32`
 * - **Описание**: Публикует вычисленное управляющее воздействие в виде напряжения [В].
 *
 * - **Топик**: `/pid_debug`
 * - **Тип сообщения**: `std_msgs::msg::Float32MultiArray`
 * - **Описание**: Публикует отладочную информацию о вкладе каждого компонента (P, I, D)
 *   для каждого из трех регуляторов. Массив содержит 9 элементов в следующем порядке:
 *   - `[0]`: Balance P (Пропорциональный вклад контура баланса)
 *   - `[1]`: Balance I (Интегральный вклад контура баланса)
 *   - `[2]`: Balance D (Дифференциальный вклад контура баланса)
 *   - `[3]`: Position P (Пропорциональный вклад контура положения)
 *   - `[4]`: Position I (Интегральный вклад контура положения)
 *   - `[5]`: Position D (Дифференциальный вклад контура положения)
 *   - `[6]`: Yaw P (Пропорциональный вклад контура рыскания)
 *   - `[7]`: Yaw I (Интегральный вклад контура рыскания)
 *   - `[8]`: Yaw D (Дифференциальный вклад контура рыскания)
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
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.read_only = false; // Явно указываем, что параметры можно менять

        declare_parameter("balance.kp", 15.0, desc);
        declare_parameter("balance.ki", 200.0, desc);
        declare_parameter("balance.kd", 0.5, desc);

        declare_parameter("position.kp", 0.0, desc);
        declare_parameter("position.ki", 0.0, desc);
        declare_parameter("position.kd", 0.0, desc);

        declare_parameter("yaw.kp", 0.0, desc);
        declare_parameter("yaw.ki", 0.0, desc);
        declare_parameter("yaw.kd", 0.0, desc);

        declare_parameter("max_voltage", 12.0, desc);
        declare_parameter("invert_left", false, desc);
        declare_parameter("invert_right", false, desc);

        // Загрузка начальных значений в наши структуры
        load_all_parameters();

        // --- QoS, Подписчики и Издатели ---
        auto motor_cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        auto feedback_qos = rclcpp::QoS(rclcpp::KeepLast(1));

        feedback_sub_ = create_subscription<Float32MultiArray>("feedback", feedback_qos, std::bind(&PIDControllerNode::feedback_callback, this, std::placeholders::_1));

        left_pub_ = create_publisher<Float32>("motor_cmd/left", motor_cmd_qos);
        right_pub_ = create_publisher<Float32>("motor_cmd/right", motor_cmd_qos);
        debug_pub_ = create_publisher<Float32MultiArray>("pid_debug", feedback_qos);

        // --- Инициализация переменных ---
        integral_balance_ = 0.0;
        integral_position_ = 0.0;
        integral_yaw_ = 0.0;
        // <<< ИЗМЕНЕНО: Резервируем 9 элементов для детальной отладки >>>
        debug_msg_.data.resize(9, 0.0f);

        // Инициализация времени для корректного расчета dt
        prev_time_ = this->get_clock()->now();

        // --- Регистрация обработчика изменения параметров ---
        param_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&PIDControllerNode::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "Узел PID-регулятора запущен. Отладка расширена до 9 компонентов.");
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

        // Вычисление шага по времени dt
        const auto current_time = this->get_clock()->now();
        const double dt = (current_time - prev_time_).seconds();
        prev_time_ = current_time;

        // Защита от некорректного dt при первом запуске или после пауз
        if (dt <= 0.0 || dt > 0.5) {
            return;
        }

        // --- Извлечение ошибок из сообщения ---
        const double error_s = msg->data[0];       // Ошибка по положению (s)
        const double error_v = msg->data[1];       // Ошибка по скорости (s_dot)
        const double error_theta = msg->data[2];   // Ошибка по углу наклона (θ)
        const double error_theta_dot = msg->data[3];// Ошибка по угловой скорости наклона (θ_dot)
        const double error_psi = msg->data[4];     // Ошибка по углу рыскания (ψ)
        const double error_psi_dot = msg->data[5]; // Ошибка по скорости рыскания (ψ_dot)

        // --- ЗАЩИТА ОТ ПАДЕНИЯ ---
        if (std::abs(error_theta) > 1.5) { // 1.5 радиана ~ 86 градусов
            left_pub_->publish(Float32().set__data(0.0f));
            right_pub_->publish(Float32().set__data(0.0f));
            integral_balance_ = 0.0;
            integral_position_ = 0.0;
            integral_yaw_ = 0.0;
            return;
        }

        // --- 1. Регулятор баланса (ПИД) ---
        const double u_balance_p = balance_gains_.kp * error_theta;
        const double u_balance_i = balance_gains_.ki * integral_balance_;
        const double u_balance_d = balance_gains_.kd * error_theta_dot;
        const double u_balance = u_balance_p + u_balance_i + u_balance_d;

        // --- 2. Регулятор положения (ПИД) ---
        const double u_position_p = position_gains_.kp * error_s;
        const double u_position_i = position_gains_.ki * integral_position_;
        const double u_position_d = position_gains_.kd * error_v;
        const double u_position = u_position_p + u_position_i + u_position_d;

        // --- 3. Регулятор поворота (рыскания) (ПИД) ---
        const double u_yaw_p = yaw_gains_.kp * error_psi;
        const double u_yaw_i = yaw_gains_.ki * integral_yaw_;
        const double u_yaw_d = yaw_gains_.kd * error_psi_dot;
        const double u_yaw = u_yaw_p + u_yaw_i + u_yaw_d;

        // --- Суммирование и расчет итогового напряжения (до ограничения) ---
        double v_right_unclamped = u_balance + u_position + u_yaw;
        double v_left_unclamped = u_balance + u_position - u_yaw;

        if (invert_left_) v_left_unclamped = -v_left_unclamped;
        if (invert_right_) v_right_unclamped = -v_right_unclamped;

        // --- Ограничение напряжения (clamping) ---
        double v_left = std::clamp(v_left_unclamped, -max_voltage_, max_voltage_);
        double v_right = std::clamp(v_right_unclamped, -max_voltage_, max_voltage_);

        // --- ANTI-WINDUP ЛОГИКА ---
        // Накапливаем интегральную ошибку только тогда, когда выходной сигнал НЕ НАСЫЩЕН.
        if (std::abs(v_left_unclamped) < max_voltage_ && std::abs(v_right_unclamped) < max_voltage_)
        {
            integral_balance_ += error_theta * dt;
            integral_position_ += error_s * dt;
            integral_yaw_ += error_psi * dt;
        }

        // --- Публикация команд на моторы ---
        left_pub_->publish(Float32().set__data(static_cast<float>(v_left)));
        right_pub_->publish(Float32().set__data(static_cast<float>(v_right)));

        // --- <<< ИЗМЕНЕНО: Заполнение отладочного сообщения 9 компонентами >>> ---
        // Контур балансировки
        debug_msg_.data[0] = static_cast<float>(u_balance_p);
        debug_msg_.data[1] = static_cast<float>(u_balance_i);
        debug_msg_.data[2] = static_cast<float>(u_balance_d);
        // Контур положения
        debug_msg_.data[3] = static_cast<float>(u_position_p);
        debug_msg_.data[4] = static_cast<float>(u_position_i);
        debug_msg_.data[5] = static_cast<float>(u_position_d);
        // Контур рыскания
        debug_msg_.data[6] = static_cast<float>(u_yaw_p);
        debug_msg_.data[7] = static_cast<float>(u_yaw_i);
        debug_msg_.data[8] = static_cast<float>(u_yaw_d);
        
        debug_pub_->publish(debug_msg_);
    }

    // Параметры
    PIDGains balance_gains_, position_gains_, yaw_gains_;
    double max_voltage_;
    bool invert_left_, invert_right_;

    // Состояния регуляторов (интегральные суммы)
    double integral_balance_;
    double integral_position_;
    double integral_yaw_;

    // Переменная для хранения времени предыдущего шага
    rclcpp::Time prev_time_;

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
