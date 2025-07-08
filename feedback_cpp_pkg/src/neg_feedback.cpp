/**
 *  neg_feedback.cpp
 *  ─────────────────
 *  Узел ROS 2 Humble для отрицательной обратной связи LQR-балансировщика
 *  двухколёсного робота.
 *
 *  <<< ИЗМЕНЕНО: Адаптировано для работы со стандартным sensor_msgs/msg/Imu >>>
 *
 *  ───────────────────────────────────────────────────────────────────
 *  ВХОДНЫЕ ТОПИКИ
 *  • /cmd                       std_msgs/msg/Float32MultiArray
 *      └─ Командный вектор-эталон (6 элементов).
 *  • /imu/data                  sensor_msgs/msg/Imu
 *      └─ Отфильтрованные данные ИМУ от imu_complementary_filter.
 *         Содержит ориентацию (кватернион) и угловые скорости.
 *  • /encoders_data/wheel_speeds std_msgs/msg/Float32MultiArray
 *      └─ Скорости колес [левое_колесо_об/с, правое_колесо_об/с]
 *
 *  ВЫХОДНОЙ ТОПИК
 *  • /feedback                  std_msgs/msg/Float32MultiArray
 *      └─ δ = cmd − state  (6 элементов).
 */

#include <array>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp> // <<< ИЗМЕНЕНО: Подключаем заголовок для Imu

// <<< ИЗМЕНЕНО: Добавляем псевдонимы для типов сообщений для краткости
using Float32Array = std_msgs::msg::Float32MultiArray;
using ImuMsg = sensor_msgs::msg::Imu;

namespace qos = rclcpp;

class NegFeedback : public rclcpp::Node
{
public:
  NegFeedback() : Node("neg_feedback")
  {
    wheel_r_    = declare_parameter("wheel_radius",  0.0325);
    wheel_base_ = declare_parameter("wheel_base",   0.1587);

    auto latest_qos = qos::QoS(qos::KeepLast(1));
    // <<< ИЗМЕНЕНО: Используем стандартный SensorDataQoS, так как фильтр публикует с ним
    auto sensor_qos = qos::SensorDataQoS();

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

    // <<< ИЗМЕНЕНО: Подписка на новый топик и новый тип сообщения
    sub_imu_ = create_subscription<ImuMsg>(
      "imu/data", sensor_qos,
      std::bind(&NegFeedback::imu_cb, this, std::placeholders::_1));

    pub_ = create_publisher<Float32Array>("feedback", latest_qos);

    msg_out_.data.resize(6, 0.0f);
    prev_time_ = this->now();

    RCLCPP_INFO(get_logger(),
                "neg_feedback (subscribing to /imu/data and /encoders_data/wheel_speeds) started.");
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
      w_l_ = msg->data[0];
      w_r_ = msg->data[1];
    }
  }

  void propagate(double dt)
  {
    const double v_l_fwd = -w_l_ * 2.0 * M_PI * wheel_r_;
    const double v_r_fwd =  w_r_ * 2.0 * M_PI * wheel_r_;
    const double v = 0.5 * (v_l_fwd + v_r_fwd);
    double psi_d_enc = 0.0;
    if (std::abs(wheel_base_) > 1e-6) {
      psi_d_enc = (v_r_fwd - v_l_fwd) / wheel_base_;
    }
    state_[0] += static_cast<float>(v * dt);
    state_[1]  = static_cast<float>(v);
    state_[4] += static_cast<float>(psi_d_enc * dt);
  }

  // <<< ИЗМЕНЕНО: Функция для извлечения угла наклона (pitch) из кватерниона
  static double get_pitch_from_quaternion(const ImuMsg::_orientation_type& q)
  {
      // Формула для pitch (поворот вокруг оси Y)
      // sin(pitch) = 2 * (w*y - z*x)
      double sinp = 2.0 * (q.w * q.y - q.z * q.x);
      // Ограничиваем значение, чтобы избежать ошибок в asinf из-за неточностей вычислений
      if (std::abs(sinp) >= 1)
          return std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
      else
          return std::asin(sinp);
  }

  // <<< ИЗМЕНЕНО: Сигнатура колбэка и вся его логика
  void imu_cb(const ImuMsg::ConstSharedPtr msg)
  {
    if (!have_cmd_) {
        RCLCPP_INFO_ONCE(get_logger(), "Waiting for initial command on /cmd topic...");
        return;
    }

    const auto current_time = this->now();
    const double dt = (current_time - prev_time_).seconds();
    prev_time_ = current_time;

    if (dt <= 0.0 || dt > 1.0) {
        RCLCPP_WARN(get_logger(), "Invalid or large dt detected: %.4f seconds. Skipping propagation step.", dt);
    } else {
        propagate(dt);  // обновляем s, v, ψ по данным энкодеров
    }

    // --- Извлекаем данные из сообщения Imu ---
    // state_[2] (θ - наклон) вычисляется из кватерниона
    state_[2] = static_cast<float>(get_pitch_from_quaternion(msg->orientation));
    
    // state_[3] (θ̇ - скорость наклона) - это угловая скорость вокруг оси Y
    state_[3] = static_cast<float>(msg->angular_velocity.y);

    // state_[5] (ψ̇ - скорость рыскания) - это угловая скорость вокруг оси Z
    state_[5] = static_cast<float>(msg->angular_velocity.z);

    // --- Формируем и публикуем вектор ошибки ---
    for (size_t i = 0; i < 6; ++i) {
      msg_out_.data[i] = cmd_[i] - state_[i];
    }
    pub_->publish(msg_out_);
  }

  double wheel_r_;
  double wheel_base_;
  std::array<float,6> cmd_{};
  std::array<float,6> state_{};
  bool   have_cmd_{false};
  float w_l_{0.0f}, w_r_{0.0f};
  rclcpp::Time prev_time_;
  rclcpp::Subscription<Float32Array>::SharedPtr sub_cmd_;
  rclcpp::Subscription<ImuMsg>::SharedPtr       sub_imu_; // <<< ИЗМЕНЕНО: Тип указателя
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
