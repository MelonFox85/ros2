/**
 * @file      lqr_controller.cpp
 * @brief     Узел LQR-регулятора для двухколёсного балансирующего робота.
 * @author    Айрат (обновлено и исправлено с помощью Copilot)
 * @date      2025-07-09
 *
 * @section description Описание
 *
 * Этот узел реализует линейно-квадратичный регулятор (LQR), являющийся
 * сердцем системы управления высокого уровня. Его задача — преобразовать
 * вектор ошибки состояния в команды напряжения для двигателей, чтобы
 * стабилизировать робота и выполнить заданную команду движения.
 *
 * @section changes Ключевые изменения в этой версии
 *
 * - **Индивидуальные параметры усиления**: Вместо единого вектора `k_gains`,
 *   который некорректно отображался в `rqt_reconfigure`, теперь каждый
 *   коэффициент усиления объявлен как отдельный параметр (`k.s`, `k.v` и т.д.).
 *   Это решает проблему с отображением и позволяет удобно настраивать каждый
 *   коэффициент на лету.
 * - **Оптимизированный колбэк параметров**: Обработчик динамических параметров
 *   теперь обновляет только те значения, которые были изменены, что более
 *   эффективно.
 * - **Защита от насыщения (Clamping)**: Выходное напряжение на моторы
 *   ограничивается параметром `max_voltage` для защиты оборудования.
 *
 * @section subscriptions Подписки
 *
 * - **Топик**: `/feedback` (std_msgs::msg::Float32MultiArray)
 *   - Принимает вектор ошибки состояния `δ = cmd - state`.
 *   - Вектор `δ`: `[δ_s, δ_v, δ_θ, δ_θ̇, δ_ψ, δ_ψ̇]`
 *
 * @section publications Публикации
 *
 * - **Топик**: `/motor_cmd/left`, `/motor_cmd/right` (std_msgs::msg::Float32)
 *   - Публикуют вычисленное напряжение для двигателей [В].
 * - **Топик**: `/lqr_debug` (std_msgs::msg::Float32MultiArray)
 *   - Публикует вклад каждой переменной состояния в управляющий сигнал.
 *   - Вектор: `[u_s, u_v, u_theta, u_theta_dot, u_psi, u_psi_dot]`
 *
 * @section lqr_logic Логика LQR
 *
 * Управление `u = -Kδ` разделено на две части:
 * 1.  **Управление Балансом (`u_balance`)**: `-(k_s*δ_s + k_v*δ_v + k_theta*δ_θ + k_theta_dot*δ_θ̇)`
 * 2.  **Управление Поворотом (`u_yaw`)**: `-(k_psi*δ_ψ + k_psi_dot*δ_ψ̇)`
 *
 * Итоговые команды: `V_left = u_balance - u_yaw`, `V_right = u_balance + u_yaw`.
 */
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <array>
#include <functional>
#include <string>
#include <algorithm> // Для std::clamp

// Псевдонимы для удобства и повышения читаемости
using Float32Msg = std_msgs::msg::Float32;
using Float32ArrayMsg = std_msgs::msg::Float32MultiArray;

class LqrController : public rclcpp::Node
{
public:
    LqrController() : Node("lqr_controller")
    {
        RCLCPP_INFO(get_logger(), "Инициализация узла LQR-регулятора...");

        // --- 1. Объявление всех параметров ---
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.read_only = false; // Явно указываем, что параметры можно менять

        // <<< ИЗМЕНЕНО: Объявляем каждый коэффициент как отдельный параметр >>>
        // Это необходимо для корректной работы с rqt_reconfigure.
        // Имена с точкой (k.s) будут сгруппированы в интерфейсе.
        declare_parameter("k.s", 0.0, desc);
        declare_parameter("k.v", 0.0, desc);
        declare_parameter("k.theta", 0.0, desc);
        declare_parameter("k.theta_dot", 0.0, desc);
        declare_parameter("k.psi", 0.0, desc);
        declare_parameter("k.psi_dot", 0.0, desc);

        declare_parameter("invert_left", false, desc);
        declare_parameter("invert_right", false, desc);
        declare_parameter("max_voltage", 12.0, desc);

        // --- 2. Инициализация издателей и подписчика ---
        auto motor_cmd_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local();
        auto default_qos = rclcpp::QoS(rclcpp::KeepLast(1));

        sub_feedback_ = create_subscription<Float32ArrayMsg>("feedback", default_qos,
            std::bind(&LqrController::feedback_cb, this, std::placeholders::_1));

        pub_left_ = create_publisher<Float32Msg>("motor_cmd/left", motor_cmd_qos);
        pub_right_ = create_publisher<Float32Msg>("motor_cmd/right", motor_cmd_qos);
        pub_debug_ = create_publisher<Float32ArrayMsg>("lqr_debug", default_qos);
        
        debug_msg_.data.resize(6, 0.0f);

        // --- 3. Первоначальная загрузка всех параметров при старте ---
        RCLCPP_INFO(get_logger(), "Первоначальная загрузка параметров...");
        load_parameters();

        // --- 4. Настройка колбэка для динамической реконфигурации ---
        parameters_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&LqrController::parameters_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(), "LQR Controller запущен и готов к работе.");
    }

private:
    /**
     * @brief Основной колбэк, вызываемый при получении вектора ошибки.
     */
    void feedback_cb(const Float32ArrayMsg::SharedPtr msg)
    {
        if (msg->data.size() < 6) {
            RCLCPP_WARN_ONCE(get_logger(), "Получен вектор обратной связи некорректного размера. Ожидается 6, получено %zu.", msg->data.size());
            return;
        }

        const auto& delta = msg->data;

        // Вычисляем вклад каждой компоненты: u_i = k_i * delta_i
        const double u_s         = K_gains_[0] * delta[0]; // K.s
        const double u_v         = K_gains_[1] * delta[1]; // K.v
        const double u_theta     = K_gains_[2] * delta[2]; // K.theta
        const double u_theta_dot = K_gains_[3] * delta[3]; // K.theta_dot
        const double u_psi       = K_gains_[4] * delta[4]; // K.psi
        const double u_psi_dot   = K_gains_[5] * delta[5]; // K.psi_dot

        // Вычисляем управляющие сигналы по закону u = -Kδ
        const double u_balance = -(u_s + u_v + u_theta + u_theta_dot);
        const double u_yaw     = -(u_psi + u_psi_dot);

        // Формирование команд для моторов
        double v_left_unclamped = u_balance - u_yaw;
        double v_right_unclamped = u_balance + u_yaw;

        if (invert_left_)  v_left_unclamped = -v_left_unclamped;
        if (invert_right_) v_right_unclamped = -v_right_unclamped;

        // Ограничение (Clamping) напряжения для защиты моторов
        const float v_left  = std::clamp(static_cast<float>(v_left_unclamped), -max_voltage_f_, max_voltage_f_);
        const float v_right = std::clamp(static_cast<float>(v_right_unclamped), -max_voltage_f_, max_voltage_f_);

        pub_left_->publish(Float32Msg().set__data(v_left));
        pub_right_->publish(Float32Msg().set__data(v_right));

        // Публикация отладочной информации
        debug_msg_.data = {
            static_cast<float>(u_s), static_cast<float>(u_v), static_cast<float>(u_theta),
            static_cast<float>(u_theta_dot), static_cast<float>(u_psi), static_cast<float>(u_psi_dot)
        };
        pub_debug_->publish(debug_msg_);
    }

    /**
     * @brief Загружает все необходимые параметры из Parameter Server в переменные класса.
     */
    void load_parameters()
    {
        // <<< ИЗМЕНЕНО: Загружаем каждый коэффициент отдельно >>>
        K_gains_[0] = this->get_parameter("k.s").as_double();
        K_gains_[1] = this->get_parameter("k.v").as_double();
        K_gains_[2] = this->get_parameter("k.theta").as_double();
        K_gains_[3] = this->get_parameter("k.theta_dot").as_double();
        K_gains_[4] = this->get_parameter("k.psi").as_double();
        K_gains_[5] = this->get_parameter("k.psi_dot").as_double();

        invert_left_  = this->get_parameter("invert_left").as_bool();
        invert_right_ = this->get_parameter("invert_right").as_bool();
        max_voltage_f_ = static_cast<float>(this->get_parameter("max_voltage").as_double());

        RCLCPP_INFO(get_logger(), "Параметры успешно загружены.");
        RCLCPP_INFO(get_logger(), " K = [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
                    K_gains_[0], K_gains_[1], K_gains_[2], K_gains_[3], K_gains_[4], K_gains_[5]);
        RCLCPP_INFO(get_logger(), " Инверсия: Left=%s, Right=%s. Макс. напряжение: %.2f В",
                    invert_left_ ? "true" : "false", invert_right_ ? "true" : "false", max_voltage_f_);
    }

    /**
     * @brief Колбэк для динамического изменения параметров.
     */
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        // Перебираем только те параметры, которые были изменены
        for (const auto &param : parameters) {
            const std::string& name = param.get_name();
            
            // <<< ИЗМЕНЕНО: Обновляем каждый коэффициент отдельно >>>
            if (name == "k.s") K_gains_[0] = param.as_double();
            else if (name == "k.v") K_gains_[1] = param.as_double();
            else if (name == "k.theta") K_gains_[2] = param.as_double();
            else if (name == "k.theta_dot") K_gains_[3] = param.as_double();
            else if (name == "k.psi") K_gains_[4] = param.as_double();
            else if (name == "k.psi_dot") K_gains_[5] = param.as_double();
            else if (name == "invert_left") invert_left_ = param.as_bool();
            else if (name == "invert_right") invert_right_ = param.as_bool();
            else if (name == "max_voltage") max_voltage_f_ = static_cast<float>(param.as_double());
        }
        
        RCLCPP_INFO(get_logger(), "Параметры LQR обновлены через колбэк.");
        return result;
    }

    // --- Переменные-члены класса ---
    rclcpp::Subscription<Float32ArrayMsg>::SharedPtr sub_feedback_;
    rclcpp::Publisher<Float32Msg>::SharedPtr pub_left_, pub_right_;
    rclcpp::Publisher<Float32ArrayMsg>::SharedPtr pub_debug_;
    Float32ArrayMsg debug_msg_;

    // Внутреннее хранилище для коэффициентов и параметров
    std::array<double, 6> K_gains_{};
    bool invert_left_{false};
    bool invert_right_{false};
    float max_voltage_f_{12.0f};

    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameters_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LqrController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
