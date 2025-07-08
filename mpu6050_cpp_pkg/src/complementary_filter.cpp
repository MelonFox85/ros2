/**
 * complementary_filter.cpp
 * ------------------------------------------------------------
 * Subscribes  : /mpu6050/raw   (Float32MultiArray, ax ay az gx gy gz)
 * Publishes   : /mpu6050/filtered_data (Float32MultiArray,
 *               [tilt_rad, gyro_y_rad_s, gyro_z_rad_s])
 *
 * This is a C++ rewrite of the previous Python node. The algorithm
 * is identical but avoids the Python→C DDS overhead and GIL stalls.
 *
 * The filter's time constant `tau` is a dynamically configurable
 * ROS2 parameter.
 */

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>

using std::placeholders::_1;
using Float32Array = std_msgs::msg::Float32MultiArray;

class ComplementaryFilter : public rclcpp::Node
{
public:
  ComplementaryFilter() : Node("mpu6050_filter_cpp"), first_run_(true), tilt_rad_(0.0)
  {
    this->declare_parameter<double>("tau", 0.5,
      rcl_interfaces::msg::ParameterDescriptor()
        .set__description("Time constant (tau) for the complementary filter in seconds."));

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(100);

    sub_ = create_subscription<Float32Array>(
        "mpu6050/raw", sensor_qos,
        std::bind(&ComplementaryFilter::raw_cb, this, _1));

    pub_ = create_publisher<Float32Array>(
        "mpu6050/filtered_data", sensor_qos);

    last_time_ = this->get_clock()->now();
    RCLCPP_INFO(get_logger(), "Complementary filter (C++) started");
  }

private:
  void raw_cb(const Float32Array::SharedPtr msg)
  {
    if (msg->data.size() < 6)
    {
      RCLCPP_WARN_ONCE(get_logger(), "Received message with insufficient data size. Skipping.");
      return;
    }

    rclcpp::Time current_time = this->get_clock()->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    if (dt <= 0 || dt > 1.0) {
        return;
    }

    // Порядок данных от издателя: [ax, ay, az, gx, gy, gz]
    // Индексы:                     0,  1,  2,  3,  4,  5
    float ay = msg->data[1];
    float az = msg->data[2];
    float gx = msg->data[3];
    // gyro_y не используется в текущей конфигурации, но оставим для ясности
    // float gyro_y = msg->data[4]; 
    float gyro_z = msg->data[5];


    // Расчет угла наклона (крен/roll) по осям Y и Z
    float accel_tilt_rad = std::atan2(ay, az);

    if (first_run_)
    {
      tilt_rad_ = accel_tilt_rad;
      first_run_ = false;
    }
    else
    {
      double tau = this->get_parameter("tau").as_double();
      double alpha = tau / (tau + dt);
      
      // Интегрируем угловую скорость по оси X (gx) для крена
      tilt_rad_ = alpha * (tilt_rad_ + gx * dt) + (1.0 - alpha) * accel_tilt_rad;
    }

    // Публикуем: [угол наклона (крен), угл. скорость наклона, угл. скорость поворота]
    Float32Array out_msg;
    // --- ИСПРАВЛЕНИЕ: Используем объявленную переменную 'gyro_z' вместо 'gz' ---
    out_msg.data = {tilt_rad_, gx, gyro_z};
    pub_->publish(out_msg);
  }

  rclcpp::Subscription<Float32Array>::SharedPtr sub_;
  rclcpp::Publisher<Float32Array>::SharedPtr pub_;
  rclcpp::Time last_time_;
  bool first_run_;
  float tilt_rad_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComplementaryFilter>());
  rclcpp::shutdown();
  return 0;
}
