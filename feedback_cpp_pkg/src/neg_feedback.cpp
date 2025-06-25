/**
 *  neg_feedback.cpp
 *  ─────────────────
 *  Узел ROS 2 Humble для отрицательной обратной связи LQR-балансировщика
 *  двухколёсного робота.
 *
 *  ───────────────────────────────────────────────────────────────────
 *  ВХОДНЫЕ ТОПИКИ
 *  • /cmd                       std_msgs/msg/Float32MultiArray
 *      └─ Командный вектор-эталон (6 элементов). После прихода первого
 *         сообщения узел начинает публиковать /feedback.
 *  • /mpu6050/filtered_data     std_msgs/msg/Float32MultiArray
 *      └─ Отфильтрованные данные ИМУ. Первые три числа:
 *           0: θ  (наклон платформы)         [рад]
 *           1: θ̇ (угловая скорость наклона)  [рад/с]
 *           2: ψ̇ (угловая скорость поворота) [рад/с]
 *  • /encoders_data/wheel_speeds std_msgs/msg/Float32MultiArray
 *      └─ Скорости колес [левое_колесо_об/с, правое_колесо_об/с]
 *         Левое: (-) против часовой (вперед), (+) по часовой (назад)
 *         Правое: (+) по часовой (вперед), (-) против часовой (назад)
 *
 *  ВЫХОДНОЙ ТОПИК
 *  • /feedback                  std_msgs/msg/Float32MultiArray
 *      └─ δ = cmd − state  (6 элементов).
 *
 *  МАТМОДЕЛЬ СИСТЕМЫ
 *  Состояние          x = [s, v, θ, θ̇, ψ, ψ̇]ᵀ
 *  Кинематика (w в об/с, v_fwd - скорость колеса, способствующая движению робота вперед):
 *      v_l_fwd = -w_l · 2πR  (т.к. отрицательный w_l дает движение вперед)
 *      v_r_fwd =  w_r · 2πR  (т.к. положительный w_r дает движение вперед)
 *      v       = ½ (v_l_fwd + v_r_fwd)
 *      ψ̇_enc   = (v_r_fwd − v_l_fwd)/L (оценка ψ̇ по энкодерам)
 *
 *  Здесь вычисляется минимальная оценка (s, v, θ, θ̇, ψ, ψ̇)
 *  для формирования вектора ошибки δ. Полноценный наблюдатель Kalman
 *  не реализован — только интегрирование поступательного движения и
 *  поворота шасси. ψ̇ для вектора состояния берется из ИМУ.
 *
 */

#include <array>
#include <cmath>
#include <cstring> // Для std::memcpy и std::fill (хотя std::fill в <algorithm>)
#include <algorithm> // Для std::min и std::fill
#include <memory>
#include <vector> // Для std::vector в сообщении

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using Float32Array = std_msgs::msg::Float32MultiArray;
namespace qos = rclcpp;

class NegFeedback : public rclcpp::Node
{
public:
  NegFeedback() : Node("neg_feedback")
  {
    wheel_r_    = declare_parameter("wheel_radius",  0.0325);   // м
    wheel_base_ = declare_parameter("wheel_base",   0.1587);    // м

    auto latest_qos = qos::QoS(qos::KeepLast(1));
    auto sensor_qos = qos::SensorDataQoS().keep_last(10);

    sub_cmd_ = create_subscription<Float32Array>(
      "cmd", latest_qos,
      [this](Float32Array::ConstSharedPtr msg)
      {
        copy_array_to(cmd_, *msg);
        have_cmd_ = true;
      });

    sub_encoders_ = create_subscription<Float32Array>(
      "encoders_data/wheel_speeds", sensor_qos,
      std::bind(&NegFeedback::encoders_cb, this, std::placeholders::_1));

    sub_imu_ = create_subscription<Float32Array>(
      "mpu6050/filtered_data", sensor_qos,
      std::bind(&NegFeedback::imu_cb, this, std::placeholders::_1));

    pub_ = create_publisher<Float32Array>("feedback", latest_qos);

    msg_out_.data.resize(6, 0.0f);
    prev_time_ = this->now();

    RCLCPP_INFO(get_logger(),
                "neg_feedback (subscribing to /encoders_data/wheel_speeds) started.");
  }

private:
  static void copy_array_to(std::array<float,6>& dst,
                            const Float32Array&  src)
  {
    std::fill(dst.begin(), dst.end(), 0.0f);
    const size_t n = std::min<size_t>(6, src.data.size());
    if (n > 0) {
        std::memcpy(dst.data(), src.data.data(), n * sizeof(float));
    }
  }

  void encoders_cb(const Float32Array::ConstSharedPtr msg)
  {
    if (msg->data.size() >= 2) {
      w_l_ = msg->data[0]; // Скорость левого колеса (об/с)
      w_r_ = msg->data[1]; // Скорость правого колеса (об/с)
    } else {
      RCLCPP_WARN_ONCE(get_logger(),
                       "Received Float32MultiArray on '%s' with incorrect size. Expected >= 2, got %zu.",
                       sub_encoders_->get_topic_name(), msg->data.size());
    }
  }

  void propagate(double dt)
  {
    // Рассчитываем линейные скорости колес, приведенные к направлению "вперед" для робота (м/с)
    // Левое колесо: отрицательный w_l_ (против часовой) -> движение вперед
    const double v_l_fwd = -w_l_ * 2.0 * M_PI * wheel_r_;
    // Правое колесо: положительный w_r_ (по часовой) -> движение вперед
    const double v_r_fwd =  w_r_ * 2.0 * M_PI * wheel_r_;

    // Линейная скорость центра робота (м/с)
    // Положительная v означает движение робота вперед
    const double v = 0.5 * (v_l_fwd + v_r_fwd);

    // Угловая скорость поворота шасси (рыскание) по энкодерам (рад/с)
    // Положительная psi_d_enc означает поворот робота против часовой стрелки (налево)
    double psi_d_enc = 0.0;
    if (std::abs(wheel_base_) > 1e-6) { // Защита от деления на ноль
      psi_d_enc = (v_r_fwd - v_l_fwd) / wheel_base_;
    }

    // Интегрирование состояния
    // state_[0] - пройденный путь s (м)
    state_[0] += static_cast<float>(v * dt);
    // state_[1] - текущая линейная скорость v (м/с)
    state_[1]  = static_cast<float>(v);
    // state_[4] - угол поворота шасси ψ (рад)
    state_[4] += static_cast<float>(psi_d_enc * dt);
    
    // state_[2] (θ), state_[3] (θ̇) и state_[5] (ψ̇) обновляются из данных ИМУ в imu_cb()
  }

  void imu_cb(const Float32Array::ConstSharedPtr msg)
  {
    if (!have_cmd_ || msg->data.size() < 3) {
        if (!have_cmd_) {
            RCLCPP_INFO_ONCE(get_logger(), "Waiting for initial command on /cmd topic...");
        }
        if (msg->data.size() < 3 && have_cmd_) {
             RCLCPP_WARN_ONCE(get_logger(),
                       "Received Float32MultiArray on '%s' with incorrect size. Expected >= 3, got %zu.",
                       sub_imu_->get_topic_name(), msg->data.size());
        }
        return;
    }

    const auto  current_time = this->now();
    const double dt   = (current_time - prev_time_).seconds();
    prev_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
        RCLCPP_WARN(get_logger(), "Invalid or large dt detected: %.4f seconds. Skipping propagation step.", dt);
    } else {
        propagate(dt);  // обновляем s, v, ψ по данным энкодеров
    }

    state_[2] = msg->data[0]; // θ - наклон платформы
    state_[3] = msg->data[1]; // θ̇ - угловая скорость наклона
    state_[5] = msg->data[2]; // ψ̇ - угловая скорость поворота (рыскание) из ИМУ

    for (size_t i = 0; i < 6; ++i) {
      msg_out_.data[i] = cmd_[i] - state_[i];
    }

    pub_->publish(msg_out_);
  }

  double wheel_r_;
  double wheel_base_;
  std::array<float,6> cmd_{};
  std::array<float,6> state_{}; // [s, v, θ, θ̇, ψ, ψ̇_imu]ᵀ
  bool   have_cmd_{false};
  float w_l_{0.0f}, w_r_{0.0f};
  rclcpp::Time prev_time_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_cmd_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_imu_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_encoders_;
  rclcpp::Publisher<Float32Array>::SharedPtr    pub_;
  Float32Array msg_out_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NegFeedback>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
