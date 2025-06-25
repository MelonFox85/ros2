#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include <array>
#include <numeric>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

class LQRVoltageController : public rclcpp::Node
{
public:
  LQRVoltageController() : Node("lqr_controller_cpp")
  {
    // Обновленные коэффициенты LQR
    const std::vector<double> default_k = {
      -0.60421,   -5.04,  0.70005,   -2.3034,   -0.77094,   0.025169,
      -0.60421,   -5.04,  -0.70005,   -2.3034,   -0.77094,  -0.025169
    };
    K_            = declare_parameter<std::vector<double>>("K", default_k);
    max_voltage_  = declare_parameter<double>("max_voltage", 12.0);
    invert_left_  = declare_parameter<bool>("invert_left",  false);
    invert_right_ = declare_parameter<bool>("invert_right", false);

    rclcpp::QoS qos(rclcpp::KeepLast(1));
    feedback_sub_ = create_subscription<Float32MultiArray>(
      "feedback", qos,
      std::bind(&LQRVoltageController::feedbackCallback, this, std::placeholders::_1));
    left_pub_  = create_publisher<Float32>("motor_cmd/left",  qos);
    right_pub_ = create_publisher<Float32>("motor_cmd/right", qos);

    RCLCPP_INFO(get_logger(), "LQR voltage controller started.");
  }

private:
  void feedbackCallback(const Float32MultiArray::SharedPtr msg)
  {
    const auto & d = msg->data;
    if (d.size() != 6) return;
    // /feedback: [s, v, theta, theta_dot, psi, psi_dot]
    // LQR: [s, theta, psi, v, theta_dot, psi_dot]
    std::array<double,6> x_lqr = {
      d[0], d[2], d[4], d[1], d[3], d[5]
    };

    // Явное применение u = -Kx
    double v_right = 0.0, v_left = 0.0;
    for (size_t j = 0; j < 6; ++j)
      v_right -= K_[j] * x_lqr[j];
    for (size_t j = 0; j < 6; ++j)
      v_left  -= K_[6+j] * x_lqr[j];

    if (invert_left_)  v_left  = -v_left;
    if (invert_right_) v_right = -v_right;

    v_left  = std::clamp(v_left,  -max_voltage_, max_voltage_);
    v_right = std::clamp(v_right, -max_voltage_, max_voltage_);

    RCLCPP_INFO(this->get_logger(),
      "x_lqr = [%.2f %.2f %.2f %.2f %.2f %.2f], v_left = %.2f, v_right = %.2f",
      x_lqr[0], x_lqr[1], x_lqr[2], x_lqr[3], x_lqr[4], x_lqr[5], v_left, v_right);

    left_pub_->publish(Float32().set__data(static_cast<float>(v_left)));
    right_pub_->publish(Float32().set__data(static_cast<float>(v_right)));
  }

  std::vector<double> K_;
  double max_voltage_;
  bool invert_left_;
  bool invert_right_;
  rclcpp::Subscription<Float32MultiArray>::SharedPtr feedback_sub_;
  rclcpp::Publisher<Float32>::SharedPtr left_pub_;
  rclcpp::Publisher<Float32>::SharedPtr right_pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LQRVoltageController>());
  rclcpp::shutdown();
  return 0;
}
