/**
 * complementary_filter.cpp
 * ------------------------------------------------------------
 * Subscribes  : /mpu6050/raw   (Float32MultiArray, ax ay az gx gy gz)
 * Publishes   : /mpu6050/filtered_data (Float32MultiArray,
 *               [tilt_rad, gyro_y_rad_s, gyro_z_rad_s])
 *
 * This is a  C++  rewrite of the previous Python node.  The algorithm
 * is identical but avoids the Python→C DDS overhead and GIL stalls.
 */

#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

using std::placeholders::_1;
using Float32Array = std_msgs::msg::Float32MultiArray;

class ComplementaryFilter : public rclcpp::Node
{
public:
  ComplementaryFilter() : Node("mpu6050_filter_cpp")
  {
    /*
     * -------- parameters --------
     *  tau        – time constant of the complementary filter [s]
     *  gyro_clip  – maximum allowed absolute gyro value    [rad/s]
     *  invert_*   – flips sensor axes to match the robot frame
     */
    tau_       = declare_parameter<double>("tau", 0.02);
    gyro_clip_ = declare_parameter<double>("gyro_clip", 30.0);

    inv_ax_ = declare_parameter<bool>("invert_acc_x",   false);
    inv_ay_ = declare_parameter<bool>("invert_acc_y",   false);
    inv_az_ = declare_parameter<bool>("invert_acc_z",   false);
    inv_gy_ = declare_parameter<bool>("invert_gyro_y",  false);
    inv_gz_ = declare_parameter<bool>("invert_gyro_z",  false);

    // QoS for high-rate sensor streams – BEST_EFFORT, small history
    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(100);

    sub_ = create_subscription<Float32Array>(
        "mpu6050/raw", sensor_qos,
        std::bind(&ComplementaryFilter::raw_cb, this, _1));

    pub_ = create_publisher<Float32Array>(
        "mpu6050/filtered_data", sensor_qos);

    // Pre-allocate output message (no heap alloc at 800 Hz)
    msg_out_.data.resize(3);

    RCLCPP_INFO(get_logger(),
                "Complementary filter started (tau = %.0f ms)",
                tau_ * 1e3);
  }

private:
  /* ------------------ callback ------------------ */
  void raw_cb(const Float32Array::SharedPtr msg)
  {
    // we expect at least 6 floats: ax ay az gx gy gz
    if (msg->data.size() < 6) return;

    float ax = msg->data[0];
    float ay = msg->data[1];
    float az = msg->data[2];
    float gx = msg->data[3];
    float gy = msg->data[4];
    float gz = msg->data[5];

    // optional axis inversion to adapt to board orientation
    if (inv_ax_) ax = -ax;
    if (inv_ay_) ay = -ay;
    if (inv_az_) az = -az;
    if (inv_gy_) gy = -gy;
    if (inv_gz_) gz = -gz;

    // time delta calculation (nanoseconds → seconds)
    int64_t now_ns = this->get_clock()->now().nanoseconds();
    if (!prev_ns_) {
      prev_ns_ = now_ns;
      return;                       // skip first frame (no dt yet)
    }
    double dt = (now_ns - *prev_ns_) * 1e-9;
    prev_ns_  = now_ns;
    if (dt <= 0.0) return;

    // --- complementary filter -------------------------------------
    double alpha      = tau_ / (tau_ + dt);
    double accel_ang  = std::atan2(ax, std::hypot(ay, az));
    double gyro_ang   = angle_ + gy * dt;
    angle_            = alpha * gyro_ang + (1.0 - alpha) * accel_ang;

    // gyro clipping (saturate to prevent controller wind-up)
    if (gy >  gyro_clip_) gy =  gyro_clip_;
    if (gy < -gyro_clip_) gy = -gyro_clip_;
    if (gz >  gyro_clip_) gz =  gyro_clip_;
    if (gz < -gyro_clip_) gz = -gyro_clip_;

    // fill and publish message (pre-allocated)
    msg_out_.data[0] = static_cast<float>(angle_);
    msg_out_.data[1] = gy;
    msg_out_.data[2] = gz;
    pub_->publish(msg_out_);
  }

  /* -------------- members -------------- */
  double tau_;
  double gyro_clip_;
  bool inv_ax_, inv_ay_, inv_az_, inv_gy_, inv_gz_;

  double                 angle_{0.0};        // current tilt [rad]
  std::optional<int64_t> prev_ns_;           // previous time stamp

  Float32Array msg_out_;                     // reused output object

  rclcpp::Subscription<Float32Array>::SharedPtr sub_;
  rclcpp::Publisher   <Float32Array>::SharedPtr pub_;
};

/* ----------------------- main ----------------------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComplementaryFilter>());
  rclcpp::shutdown();
  return 0;
}
