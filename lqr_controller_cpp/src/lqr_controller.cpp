/**
 * @file      lqr_controller.cpp
 * @brief     Узел LQR-регулятора для двухколёсного балансирующего робота.
 * @author    Айрат (обновлено и исправлено с помощью Copilot)
 * @date      2025-07-08
 *
 * @section description Описание
 *
 * Этот узел реализует линейно-квадратичный регулятор (LQR). Он является
 * сердцем системы управления высокого уровня. Его задача — преобразовать
 * вектор ошибки состояния в команды напряжения для двигателей, чтобы
 * стабилизировать робота и выполнить заданную команду движения.
 *
 * Главной особенностью этой версии является **динамическая реконфигурация**.
 * Вы можете на лету изменять все коэффициенты усиления (например, через
 * rqt_reconfigure), и узел немедленно применит их без перезапуска.
 *
 * @section subscriptions Подписки
 *
 * - **Топик**: `/feedback` (std_msgs/msg/Float32MultiArray)
 *   - Принимает вектор ошибки состояния `δ = cmd - state` от узла `neg_feedback`.
 *   - Вектор `δ` состоит из 6 элементов:
 *     [δ_s, δ_v, δ_θ, δ_θ̇, δ_ψ, δ_ψ̇]
 *
 * @section publications Публикации
 *
 * - **Топик**: `/motor_cmd/left` (std_msgs/msg/Float32)
 *   - Публикует вычисленное напряжение для левого двигателя (В).
 * - **Топик**: `/motor_cmd/right` (std_msgs/msg/Float32)
 *   - Публикует вычисленное напряжение для правого двигателя (В).
 *
 * @section lqr_logic Логика LQR
 *
 * Управление роботом разделено на две независимые задачи:
 *
 * 1.  **Управление Балансом (Balance Control)**:
 *     - Отвечает за движение вперед-назад и удержание равновесия.
 *     - Использует ошибки по позиции (s), скорости (v), углу наклона (θ) и скорости наклона (θ̇).
 *     - Вычисляется общее для обоих колес управляющее воздействие `u_balance`.
 *     - `u_balance = - (k_s*δ_s + k_v*δ_v + k_theta*δ_θ + k_theta_dot*δ_θ̇)`
 *
 * 2.  **Управление Поворотом (Yaw Control)**:
 *     - Отвечает за повороты робота на месте или в движении.
 *     - Использует ошибки по углу рыскания (ψ) и скорости рыскания (ψ̇).
 *     - Вычисляется управляющее воздействие `u_yaw`, которое будет приложено к колесам
 *       с разными знаками для создания крутящего момента.
 *     - `u_yaw = - (k_psi*δ_ψ + k_psi_dot*δ_ψ̇)`
 *
 * 3.  **Итоговые команды для двигателей**:
 *     - Напряжение для моторов формируется путем комбинирования этих двух управляющих сигналов.
 *       Инверсия знака для `u_yaw` как раз и создает поворот.
 *     - V_left  = u_balance - u_yaw
 *     - V_right = u_balance + u_yaw
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <array>
#include <functional>

// Псевдонимы для удобства
using Float32Msg = std_msgs::msg::Float32;
using Float32ArrayMsg = std_msgs::msg::Float32MultiArray;

class LqrController : public rclcpp::Node
{
public:
    LqrController() : Node("lqr_controller")
    {
        // --- 1. Объявление параметров с осмысленными значениями по умолчанию ---
        // Эти значения будут использоваться, если они не переопределены в launch-файле.
        // Они подобраны как типичные начальные значения для балансирующего робота.
        RCLCPP_INFO(get_logger(), "Объявление параметров LQR-регулятора...");
        
        // Коэффициенты для управления БАЛАНСОМ
        declare_parameter("k_s", 0.0);           // Усиление по ошибке позиции (s). Часто начинают с 0.
        declare_parameter("k_v", -0.45);         // Усиление по ошибке скорости (v).
        declare_parameter("k_theta", -11.0);     // Усиление по ошибке угла наклона (θ). Ключевой параметр.
        declare_parameter("k_theta_dot", -2.0);  // Усиление по ошибке скорости наклона (θ̇).
        
        // Коэффициенты для управления ПОВОРОТОМ
        declare_parameter("k_psi", 0.0);         // Усиление по ошибке угла поворота (ψ). Часто начинают с 0.
        declare_parameter("k_psi_dot", -0.5);    // Усиление по ошибке скорости поворота (ψ̇).

        // --- 2. Инициализация издателей и подписчика ---
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        sub_feedback_ = create_subscription<Float32ArrayMsg>(
            "feedback", qos,
            std::bind(&LqrController::feedback_cb, this, std::placeholders::_1));

        pub_left_ = create_publisher<Float32Msg>("motor_cmd/left", qos);
        pub_right_ = create_publisher<Float32Msg>("motor_cmd/right", qos);

        // --- 3. Настройка колбэка для динамической реконфигурации ---
        // Регистрируем функцию, которая будет автоматически вызываться ROS 2
        // каждый раз, когда один или несколько параметров этого узла изменяются.
        parameters_callback_handle_ = add_on_set_parameters_callback(
            std::bind(&LqrController::parameters_callback, this, std::placeholders::_1));

        // --- 4. Первоначальная загрузка параметров и формирование векторов усиления ---
        RCLCPP_INFO(get_logger(), "Первоначальная загрузка параметров...");
        load_and_update_gains();
        
        RCLCPP_INFO(get_logger(), "LQR Controller запущен и готов к работе.");
    }

private:
    /**
     * @brief Колбэк, вызываемый при получении вектора ошибки с топика /feedback.
     */
    void feedback_cb(const Float32ArrayMsg::SharedPtr msg)
    {
        if (msg->data.size() < 6) {
            RCLCPP_WARN_ONCE(get_logger(), "Получен вектор обратной связи некорректного размера. Ожидается 6, получено %zu.", msg->data.size());
            return;
        }

        const auto& delta = msg->data; // Псевдоним для краткости

        // --- Вычисление управляющего воздействия для БАЛАНСА ---
        // u_balance = - (k_balance ⋅ [δ_s, δ_v, δ_θ, δ_θ̇]ᵀ)
        const double u_balance = -(K_balance_[0] * delta[0] +   // s
                                   K_balance_[1] * delta[1] +   // v
                                   K_balance_[2] * delta[2] +   // theta
                                   K_balance_[3] * delta[3]);   // theta_dot

        // --- Вычисление управляющего воздействия для ПОВОРОТА ---
        // u_yaw = - (k_yaw ⋅ [δ_ψ, δ_ψ̇]ᵀ)
        const double u_yaw = -(K_yaw_[0] * delta[4] +           // psi
                               K_yaw_[1] * delta[5]);          // psi_dot

        // --- Формирование итоговых команд для моторов ---
        // Здесь и происходит та самая "инверсия знаков" для создания поворота.
        Float32Msg left_cmd, right_cmd;
        left_cmd.data = static_cast<float>(u_balance - u_yaw);
        right_cmd.data = static_cast<float>(u_balance + u_yaw);

        pub_left_->publish(left_cmd);
        pub_right_->publish(right_cmd);
    }

    /**
     * @brief Загружает все параметры усиления и обновляет внутренние векторы K.
     */
    void load_and_update_gains()
    {
        // Читаем текущие значения всех параметров из ROS Parameter Server
        K_balance_[0] = get_parameter("k_s").as_double();
        K_balance_[1] = get_parameter("k_v").as_double();
        K_balance_[2] = get_parameter("k_theta").as_double();
        K_balance_[3] = get_parameter("k_theta_dot").as_double();
        
        K_yaw_[0] = get_parameter("k_psi").as_double();
        K_yaw_[1] = get_parameter("k_psi_dot").as_double();

        RCLCPP_INFO(get_logger(), "Векторы усиления K обновлены:");
        RCLCPP_INFO(get_logger(), "  K_balance [s, v, th, th_d]: [%.2f, %.2f, %.2f, %.2f]",
                    K_balance_[0], K_balance_[1], K_balance_[2], K_balance_[3]);
        RCLCPP_INFO(get_logger(), "  K_yaw [psi, psi_d]: [%.2f, %.2f]",
                    K_yaw_[0], K_yaw_[1]);
    }

    /**
     * @brief Колбэк, автоматически вызываемый ROS 2 при изменении параметров.
     */
    rcl_interfaces::msg::SetParametersResult parameters_callback(
        const std::vector<rclcpp::Parameter> &parameters)
    {
        (void)parameters; // Подавляем предупреждение о неиспользуемой переменной
        
        // Просто вызываем нашу основную функцию обновления.
        load_and_update_gains();

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "Parameters updated successfully";
        return result;
    }

    // --- Члены класса ---
    rclcpp::Subscription<Float32ArrayMsg>::SharedPtr sub_feedback_;
    rclcpp::Publisher<Float32Msg>::SharedPtr pub_left_;
    rclcpp::Publisher<Float32Msg>::SharedPtr pub_right_;

    // ИСПРАВЛЕНО: Теперь у нас два отдельных вектора усиления,
    // что точно соответствует раздельной логике управления.
    std::array<double, 4> K_balance_{}; // Вектор K для управления балансом
    std::array<double, 2> K_yaw_{};     // Вектор K для управления поворотом

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
