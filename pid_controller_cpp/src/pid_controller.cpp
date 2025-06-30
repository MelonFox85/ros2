/**
 * pid_controller.cpp
 * ------------------------------------------------------------
 * Узел ПИД-регулятора для двухколесного самобалансирующегося робота.
 * Подписывается на топик /feedback (ошибка состояния) и публикует
 * команды напряжения на моторы в топики /motor_cmd/left и /motor_cmd/right.
 *
 * Включает защиту от падения и защиту от "накрутки" интеграла.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

class PIDVoltageController : public rclcpp::Node
{
public:
  PIDVoltageController() : Node("pid_controller_cpp")
  {
    /* -------- Параметры ПИД-коэффициентов ------------------- */
    // Эти значения - просто пример. Их нужно будет тщательно настраивать.
    Kp_theta_ = declare_parameter<double>("gains.kp_theta", 20.0);      // Угол наклона
    Kd_theta_dot_ = declare_parameter<double>("gains.kd_theta_dot", 0.8); // Скорость наклона

    Kp_s_ = declare_parameter<double>("gains.kp_s", 0.1);          // Положение/путь
    Ki_s_ = declare_parameter<double>("gains.ki_s", 0.02);         // Интеграл по положению

    Kp_psi_dot_ = declare_parameter<double>("gains.kp_psi_dot", 0.5); // Скорость рыскания (для поворотов)

    /* -------- Параметры безопасности и ограничений ---------- */
    max_voltage_ = declare_parameter<double>("max_voltage", 12.0);
    max_integral_ = declare_parameter<double>("max_integral", 100.0); // Ограничение интегральной суммы
    fall_angle_rad_ = declare_parameter<double>("fall_angle_deg", 35.0) * (M_PI / 180.0);

    /* -------- Подписчики и Публикаторы ---------------------- */
    rclcpp::QoS qos(rclcpp::KeepLast(1));
    feedback_sub_ = create_subscription<Float32MultiArray>(
      "feedback", qos,
      std::bind(&PIDVoltageController::feedbackCallback, this, std::placeholders::_1));
      
    left_pub_ = create_publisher<Float32>("motor_cmd/left", qos);
    right_pub_ = create_publisher<Float32>("motor_cmd/right", qos);

    RCLCPP_INFO(get_logger(), "ПИД-регулятор напряжения запущен.");
    RCLCPP_INFO(get_logger(), "Критический угол падения: %.1f градусов", fall_angle_rad_ * (180.0/M_PI));
  }

private:
  void feedbackCallback(const Float32MultiArray::SharedPtr msg)
  {
    // Вектор ошибки из /feedback: [s, v, theta, theta_dot, psi, psi_dot]
    if (msg->data.size() < 6) {
      RCLCPP_WARN_ONCE(get_logger(), "Получено сообщение в /feedback с неверным размером.");
      return;
    }

    const float s_err = msg->data[0];
    // const float v_err = msg->data[1]; // Не используется в этой реализации
    const float theta_err = msg->data[2];
    const float theta_dot_err = msg->data[3];
    // const float psi_err = msg->data[4]; // Не используется в этой реализации
    const float psi_dot_err = msg->data[5];

    /* -------- ЗАЩИТА ОТ ПАДЕНИЯ ----------------------------- */
    // Если угол наклона слишком большой, считаем, что робот упал.
    // Публикуем нули, чтобы остановить моторы и предотвратить "судороги".
    if (std::abs(theta_err) > fall_angle_rad_) {
      if (!is_fallen_) {
        RCLCPP_WARN(get_logger(), "Критический угол наклона (%.1f deg)! Остановка моторов.", theta_err * (180.0/M_PI));
        is_fallen_ = true;
        integral_s_ = 0.0; // Сбрасываем интеграл при падении
      }
      left_pub_->publish(Float32().set__data(0.0f));
      right_pub_->publish(Float32().set__data(0.0f));
      return;
    }
    // Если робот вернулся в рабочую зону углов
    if (is_fallen_) {
        RCLCPP_INFO(get_logger(), "Робот вернулся в рабочее положение. Возобновление управления.");
        is_fallen_ = false;
    }

    /* -------- Расчет ПИД ------------------------------------ */
    // Интегральная часть (только для положения s)
    integral_s_ += s_err;
    // Защита от накрутки интеграла (anti-windup)
    integral_s_ = std::clamp(integral_s_, -max_integral_, max_integral_);

    // Базовое напряжение для балансировки и движения
    double base_voltage = Kp_theta_ * theta_err +
                          Kd_theta_dot_ * theta_dot_err +
                          Kp_s_ * s_err +
                          Ki_s_ * integral_s_;

    // Коррекция для поворота
    double turn_voltage = Kp_psi_dot_ * psi_dot_err;

    double v_left = base_voltage + turn_voltage;
    double v_right = base_voltage - turn_voltage; // Знак '-' для поворота в нужную сторону

    // Ограничение максимального напряжения
    v_left = std::clamp(v_left, -max_voltage_, max_voltage_);
    v_right = std::clamp(v_right, -max_voltage_, max_voltage_);

    /* -------- Публикация команд ----------------------------- */
    left_pub_->publish(Float32().set__data(static_cast<float>(v_left)));
    right_pub_->publish(Float32().set__data(static_cast<float>(v_right)));
  }

  // Параметры
  double Kp_theta_, Kd_theta_dot_;
  double Kp_s_, Ki_s_;
  double Kp_psi_dot_;
  double max_voltage_, max_integral_, fall_angle_rad_;

  // Состояние
  double integral_s_{0.0};
  bool is_fallen_{false};

  // ROS
  rclcpp::Subscription<Float32MultiArray>::SharedPtr feedback_sub_;
  rclcpp::Publisher<Float32>::SharedPtr left_pub_;
  rclcpp::Publisher<Float32>::SharedPtr right_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDVoltageController>());
  rclcpp::shutdown();
  return 0;
}
