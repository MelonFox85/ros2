/**
 * \file      motor_controller.cpp
 * \author    Airat <your-mail@example.com>
 * \brief     Low-level PWM driver for H-bridge motor driver in a self-balancing robot.
 *
 * Subscriptions:
 *   • /mpu6050/filtered_data   (std_msgs/Float32MultiArray, BE)
 *       data[0] – tilt angle (rad). Used for E-STOP.
 *   • /motor_cmd/left          (std_msgs/Float32) ‹V_left›
 *   • /motor_cmd/right         (std_msgs/Float32) ‹V_right›
 *       desired motor voltage (V), published by the LQR controller.
 *  
 * Publications:
 *   • motor_feedback/left      (std_msgs/Float32MultiArray)
 *   • motor_feedback/right     (std_msgs/Float32MultiArray)
 *       [V_cmd, duty %, V_clamped] for monitoring/debug.
 */

#include <cmath>
#include <functional>
#include <memory>
#include <stdexcept>
#include <vector>

#include <pigpio.h>          // local-mode API
#include <pigpiod_if2.h>     // remote-mode (daemon) API

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

class MotorController : public rclcpp::Node
{
  /* ─── GPIO (BCM) ───────────────────────────────────────────── */
  static constexpr unsigned PWMA = 13, AIN1 = 27, AIN2 = 17;   // Right
  static constexpr unsigned PWMB = 12, BIN1 = 14, BIN2 = 4;    // Left
  static constexpr unsigned STBY = 15;
  static constexpr unsigned PWM_FREQ   = 10'000;               // 10 kHz
  static constexpr unsigned PWM_FULL   = 1'000'000;            // 100 % for hardware_PWM

  static constexpr double DEG2RAD = M_PI / 180.0;

public:
  MotorController() : Node("motor_controller") { init(); }
  ~MotorController() override { shutdown(); }

private:
  void init()
  {
    declare_parameter<double>("critical_angle_deg", 40.0);
    declare_parameter<double>("V_clip",               12.0); // voltage saturation [V]

    declare_parameter<std::string>("imu_topic",        "mpu6050/filtered_data");
    declare_parameter<std::string>("cmd_left_topic",   "motor_cmd/left");
    declare_parameter<std::string>("cmd_right_topic",  "motor_cmd/right");

    crit_angle_rad_ = get_parameter("critical_angle_deg").as_double() * DEG2RAD;
    V_clip_         = get_parameter("V_clip").as_double();

    pi_ = pigpio_start(nullptr,nullptr);           // remote-mode
    if (pi_ >= 0)  remote_ = true;
    else {
      if (gpioInitialise() < 0)
        throw std::runtime_error("pigpio initialisation failed");
      remote_ = false;
    }

    auto setMode = [this](unsigned g,unsigned m){
      remote_ ? set_mode (pi_,g,m) : gpioSetMode(g,m); };
    auto write   = [this](unsigned g,unsigned l){
      remote_ ? gpio_write(pi_,g,l) : gpioWrite  (g,l); };
    hwPWM_ = [this](unsigned g,unsigned freq,unsigned duty){
      return remote_
             ? hardware_PWM(pi_,g,freq,duty)
             : gpioHardwarePWM(g,freq,duty);
    };

    for (unsigned p:{AIN1,AIN2,BIN1,BIN2,STBY})
      { setMode(p,PI_OUTPUT); write(p,0); }
    for (unsigned p:{PWMA,PWMB}) setMode(p,PI_OUTPUT);
    write(STBY,1);

    write_ = write;

    rclcpp::QoS qos_rel(rclcpp::KeepLast(1));
    rclcpp::QoS qos_be (rclcpp::KeepLast(10));
    qos_be.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_be.durability (rclcpp::DurabilityPolicy::Volatile);

    auto imu_t   = get_parameter("imu_topic").as_string();
    auto cmd_l_t = get_parameter("cmd_left_topic").as_string();
    auto cmd_r_t = get_parameter("cmd_right_topic").as_string();

    imu_sub_  = create_subscription<Float32MultiArray>(
                  imu_t, qos_be,
                  std::bind(&MotorController::imuCb,this,std::placeholders::_1));
    cmd_l_sub_= create_subscription<Float32>(
                  cmd_l_t,qos_rel,
                  std::bind(&MotorController::cmdLCb,this,std::placeholders::_1));
    cmd_r_sub_= create_subscription<Float32>(
                  cmd_r_t,qos_rel,
                  std::bind(&MotorController::cmdRCb,this,std::placeholders::_1));

    left_fb_pub_  = create_publisher<Float32MultiArray>("motor_feedback/left", qos_rel);
    right_fb_pub_ = create_publisher<Float32MultiArray>("motor_feedback/right",qos_rel);

    RCLCPP_INFO(get_logger(),
                "MotorController ready (%s-mode, E-STOP %.1f°)",
                remote_?"remote":"local",
                crit_angle_rad_/DEG2RAD);
  }

  void shutdown()
  {
    RCLCPP_INFO(get_logger(),"MotorController shutting down.");
    hardStop();
    if (remote_) pigpio_stop(pi_); else { write_(STBY,0); gpioTerminate(); }
  }

  // IMU callback: checks tilt angle for safety
  void imuCb(const Float32MultiArray::SharedPtr m){
    if (m->data.empty()) return;
    double tilt = m->data[0];
    if (std::abs(tilt) > crit_angle_rad_){
      if (!emergency_stop_)
        RCLCPP_WARN(get_logger(),"E-STOP tilt %.1f°",tilt/DEG2RAD);
      emergency_stop_=true; hardStop(); write_(STBY,0);
    } else if (emergency_stop_){
      RCLCPP_INFO(get_logger(),"Tilt OK, resume.");
      emergency_stop_=false; write_(STBY,1);
    }
  }

  // Command callbacks: receive desired voltage for each motor
  void cmdLCb(const Float32::SharedPtr m){
    drive(clipV(m->data), BIN1,BIN2,PWMB, left_fb_pub_);
  }
  void cmdRCb(const Float32::SharedPtr m){
    drive(clipV(m->data), AIN1,AIN2,PWMA, right_fb_pub_);
  }

  // Saturate voltage to safe range
  double clipV(double v) const {
    return std::clamp(v, -V_clip_, V_clip_);
  }

  /**
   * Motor control logic using desired voltage.
   * Sets direction and PWM duty based on input voltage.
   * - v_cmd: desired voltage (V), after saturation
   */
  void drive(double v_cmd,
             unsigned in1,unsigned in2,unsigned pwm_pin,
             const rclcpp::Publisher<Float32MultiArray>::SharedPtr& pub)
  {
    constexpr double MIN_DUTY = 7.5; // min % to overcome deadzone

    if (emergency_stop_){ brake(in1,in2,pwm_pin); return; }

    double V = std::clamp(v_cmd, -V_clip_, V_clip_);
    double duty_pct = (V / V_clip_) * 100.0;
    duty_pct = std::clamp(duty_pct, -100.0, 100.0);

    unsigned duty_hw;
    if (duty_pct > 0.0) {
        // Применяем минимальную скважность только для малых значений
        if (duty_pct < MIN_DUTY) duty_pct = MIN_DUTY;
        duty_hw = static_cast<unsigned>(std::round(duty_pct / 100.0 * PWM_FULL));
        write_(in1, 1); write_(in2, 0);
    } else if (duty_pct < 0.0) {
        if (duty_pct > -MIN_DUTY) duty_pct = -MIN_DUTY;
        duty_hw = static_cast<unsigned>(std::round(std::abs(duty_pct) / 100.0 * PWM_FULL));
        write_(in1, 0); write_(in2, 1);
    } else {
        write_(in1, 0); write_(in2, 0);
        hwPWM_(pwm_pin, 0, 0);
        return;
    }

    hwPWM_(pwm_pin, PWM_FREQ, duty_hw);

    Float32MultiArray fb;
    fb.data = {static_cast<float>(v_cmd),
               static_cast<float>(duty_pct),
               static_cast<float>(V)};
    pub->publish(fb);
  }

  // Brake the motor (coast)
  void brake(unsigned in1,unsigned in2,unsigned pwm_pin){
    write_(in1,0); write_(in2,0);
    hwPWM_(pwm_pin,0,0);   // coast
  }

  // Stop both motors immediately
  void hardStop(){
    brake(BIN1,BIN2,PWMB); brake(AIN1,AIN2,PWMA);
  }

  /* ---------------- data -------------------------------------- */
  double crit_angle_rad_{}, V_clip_{};
  bool   emergency_stop_{false};

  /* pigpio */
  int  pi_{-1};
  bool remote_{false};

  /* wrappers */
  std::function<void(unsigned,unsigned)>               write_;
  std::function<int(unsigned,unsigned,unsigned)>       hwPWM_;

  /* ROS entities */
  rclcpp::Subscription<Float32MultiArray>::SharedPtr imu_sub_;
  rclcpp::Subscription<Float32>::SharedPtr           cmd_l_sub_, cmd_r_sub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr    left_fb_pub_, right_fb_pub_;
};

int main(int argc,char* argv[])
{
  rclcpp::init(argc,argv);
  try{ rclcpp::spin(std::make_shared<MotorController>()); }
  catch(const std::exception& e){
    RCLCPP_FATAL(rclcpp::get_logger("motor_controller"),"%s",e.what());
  }
  rclcpp::shutdown();
  return 0;
}
