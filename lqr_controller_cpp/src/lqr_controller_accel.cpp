/**
 * LQR-controller → desired wheel ACCELERATION (α*, rad/s²).
 *
 * Input:
 *     /feedback                – std_msgs::msg::Float32MultiArray, 6-element δ-vector
 *
 * Output:
 *     /motor_cmd/left          – std_msgs::msg::Float32,  α_L*  [rad/s²]
 *     /motor_cmd/right         – std_msgs::msg::Float32,  α_R*  [rad/s²]
 *
 * Parameters:
 *     K            double[6]   – LQR gain vector (default from original Python node)
 *     max_alpha    double      – |α|max, saturation limit                  (default: 500.0)
 *     invert_left  bool        – negate sign for the left motor command    (default: false)
 *     invert_right bool        – negate sign for the right motor command   (default: false)
 *
 * The node prints only two informational messages: one at start-up and one at shutdown.
 * Runtime errors (e.g. wrong feedback size) are still logged to ease debugging.
 */

#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

class LQRTorqueController : public rclcpp::Node
{
public:
  LQRTorqueController() : Node("lqr_controller")
  {
    /* ── Declare & read parameters ─────────────────────────────────────────── */
    const std::vector<double> default_k =
      {2.85775, 12.7757, 318.69096, 31.01042, 4.04145, 2.14415};

    K_            = declare_parameter<std::vector<double>>("K", default_k);
    max_alpha_    = declare_parameter<double>("max_alpha",    500.0);
    invert_left_  = declare_parameter<bool>  ("invert_left",  false);
    invert_right_ = declare_parameter<bool>  ("invert_right", false);

    if (K_.size() != 6) {
      RCLCPP_FATAL(get_logger(), "Parameter K must contain exactly 6 elements.");
      throw std::runtime_error("Invalid parameter K size");
    }

    /* ── QoS & comms ──────────────────────────────────────────────────────── */
    rclcpp::QoS qos(rclcpp::KeepLast(1));

    feedback_sub_ = create_subscription<Float32MultiArray>(
      "feedback", qos,   // ← topic name updated
      std::bind(&LQRTorqueController::feedbackCallback, this, std::placeholders::_1));

    left_pub_  = create_publisher<Float32>("motor_cmd/left",  qos);
    right_pub_ = create_publisher<Float32>("motor_cmd/right", qos);

    RCLCPP_INFO(get_logger(), "LQR controller ready.");
  }

  ~LQRTorqueController() override
  {
    RCLCPP_INFO(get_logger(), "LQR controller shutting down.");
  }

private:
  /* ── Callback ───────────────────────────────────────────────────────────── */
  void feedbackCallback(const Float32MultiArray::SharedPtr msg)
  {
    const auto & data = msg->data;

    if (data.size() != 6) {
      RCLCPP_ERROR(get_logger(), "/feedback must be a 6-element vector.");
      return;
    }
    if (std::any_of(data.begin(), data.end(),
                    [](float v){ return std::isnan(v) || std::isinf(v); })) {
      RCLCPP_ERROR(get_logger(), "/feedback contains NaN/Inf.");
      return;
    }

    /* u = −K·δ (signs preserved from original implementation) */
    const double K1 = K_[0], K2 = K_[1], K3 = K_[2];
    const double K4 = K_[3], K5 = K_[4], K6 = K_[5];

    const double d0 = data[0], d1 = data[1], d2 = data[2];
    const double d3 = data[3], d4 = data[4], d5 = data[5];

    double alpha_L =  (K1*d0 + K2*d1 + K3*d2 + K4*d3 + K5*d4 + K6*d5);
    double alpha_R =  (K1*d0 + K2*d1 + K3*d2 + K4*d3 - K5*d4 - K6*d5);

    if (invert_left_)  alpha_L = -alpha_L;
    if (invert_right_) alpha_R = -alpha_R;

    /* Saturate */
    alpha_L = std::clamp(alpha_L, -max_alpha_, max_alpha_);
    alpha_R = std::clamp(alpha_R, -max_alpha_, max_alpha_);

    /* Publish */
    left_pub_->publish(Float32().set__data(static_cast<float>(alpha_L)));
    right_pub_->publish(Float32().set__data(static_cast<float>(alpha_R)));
  }

  /* ── Data ──────────────────────────────────────────────────────────────── */
  std::vector<double> K_;
  double max_alpha_;
  bool invert_left_;
  bool invert_right_;

  rclcpp::Subscription<Float32MultiArray>::SharedPtr feedback_sub_;
  rclcpp::Publisher<Float32>::SharedPtr left_pub_;
  rclcpp::Publisher<Float32>::SharedPtr right_pub_;
};

/* ── main ─────────────────────────────────────────────────────────────────── */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<LQRTorqueController>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("lqr_controller"),
                 "Unhandled exception: %s", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
