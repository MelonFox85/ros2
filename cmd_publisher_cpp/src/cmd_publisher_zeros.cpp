/**
 * cmd_publisher.cpp
 * ------------------------------------------------------------
 * Publishes Float32MultiArray "cmd" with six zeros
 * at a configurable control-rate (Hz).  The message layout
 * is the same as in the original Python node, so the rest of
 * the system (LQR, etc.) does not have to change.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using Float32Array = std_msgs::msg::Float32MultiArray;
using namespace std::chrono_literals;

class CmdPublisher : public rclcpp::Node
{
public:
  CmdPublisher() : Node("cmd_publisher_cpp")
  {
    /* -------- parameters ------------------------------------ */
    // Global control frequency [Hz].  400 Hz is a good default for
    // a self-balancing robot.
    ctrl_rate_ = declare_parameter<double>("control_rate", 400.0);
    if (ctrl_rate_ <= 0.0) {
      RCLCPP_WARN(get_logger(),
                  "control_rate <= 0, forcing 400 Hz");
      ctrl_rate_ = 400.0;
    }
    period_ = std::chrono::duration<double>(1.0 / ctrl_rate_);

    /* -------- publisher ------------------------------------- */
    // Keep only the newest command (depth = 1).
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    pub_ = create_publisher<Float32Array>("cmd", qos);

    /* -------- timer ----------------------------------------- */
    timer_ = create_wall_timer(period_,
                               std::bind(&CmdPublisher::tick, this));

    /* -------- pre-allocate output message ------------------- */
    msg_.data.assign(6, 0.0f);  // six zeros
  }

private:
  /* ---------------------------------------------------------- */
  void tick()
  {
    // No heavy computation: just re-publish the same buffer.
    pub_->publish(msg_);
  }

  /* ---------------- members --------------------------------- */
  double ctrl_rate_;
  std::chrono::duration<double> period_;

  Float32Array msg_;                                   // reused buffer
  rclcpp::Publisher<Float32Array>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr          timer_;
};

/* ======================== main ============================== */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdPublisher>());
  rclcpp::shutdown();
  return 0;
}
