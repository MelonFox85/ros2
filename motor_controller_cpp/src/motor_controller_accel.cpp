/**
 * \file      motor_controller.cpp
 * \author    Airat <your-mail@example.com>
 * \brief     Low-level PWM-драйвер Н-моста для робота-балансира.
 *
 * Подписки (QoS default unless сказано иначе)
 * ────────────────────────────────────────────────────────────
 *   • /mpu6050/filtered_data   (std_msgs/Float32MultiArray,  BE)
 *       data[0] – угол наклона (rad). Служит для E-STOP.
 *   • /encoders_data/left      (std_msgs/Float32)
 *   • /encoders_data/right     (std_msgs/Float32)
 *       данные энкодеров (rev/s) → ω [rad/s].
 *   • /motor_cmd/left          (std_msgs/Float32) ‹α_d›
 *   • /motor_cmd/right         (std_msgs/Float32) ‹α_d›
 *       желаемое угловое ускорение (rad/s²), публикует LQR-контроллер.
 *  
 * Публикации
 * ────────────────────────────────────────────────────────────
 *   • motor_feedback/left      (std_msgs/Float32MultiArray)
 *   • motor_feedback/right     (std_msgs/Float32MultiArray)
 *       [α_d, duty %, V] для мониторинга/отладки.
 *
 * Особенности
 * ────────────────────────────────────────────────────────────
 *   1. Сначала пробуем подключиться к работающему pigpiod
 *      (`pigpio_start` → remote-mode, root не нужен).
 *   2. Если неудачно — fallback к local-mode (`gpioInitialise`)
 *      → нужен sudo или capability CAP_SYS_RAWIO.
 *   3. Защита от опрокидывания: при |tilt| > critical_angle_deg
 *      узел даёт команду тормоз/STBY и ждёт нормализации угла.
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
  static constexpr unsigned PWM_FULL   = 1'000'000;            // 100 % для hardware_PWM

  static constexpr double TWO_PI  = 2.0 * M_PI;
  static constexpr double DEG2RAD = M_PI / 180.0;

public:
  MotorController() : Node("motor_controller") { init(); }
  ~MotorController() override { shutdown(); }

private:
  /* ---------------- initialisation --------------------------- */
  void init()
  {
    /* ----- параметры ----------------------------------------- */
    declare_parameter<double>("critical_angle_deg", 40.0);
    declare_parameter<double>("alpha_clip",          500.0);
    declare_parameter<double>("V_static",              0.0);   // В, компенсация трения

    declare_parameter<double>("gear_ratio",  30.0);
    declare_parameter<double>("R_a",          2.3);
    declare_parameter<double>("K_e_rpm",   1.03e-3);
    declare_parameter<double>("K_t",       9.84e-3);
    declare_parameter<double>("J_rotor",   2.5e-6);
    declare_parameter<double>("J_wheel", 1.7956e-5);
    declare_parameter<double>("V_bus",       12.0);

    declare_parameter<std::string>("imu_topic",        "mpu6050/filtered_data");
    declare_parameter<std::string>("enc_left_topic",   "encoders_data/left");
    declare_parameter<std::string>("enc_right_topic",  "encoders_data/right");
    declare_parameter<std::string>("cmd_left_topic",   "motor_cmd/left");
    declare_parameter<std::string>("cmd_right_topic",  "motor_cmd/right");

    /* ---- чтение и пересчёт ---------------------------------- */
    crit_angle_rad_ = get_parameter("critical_angle_deg").as_double()*DEG2RAD;
    alpha_clip_     = get_parameter("alpha_clip").as_double();
    V_static_       = get_parameter("V_static").as_double();

    const double N        = get_parameter("gear_ratio").as_double();
    R_a_           = get_parameter("R_a").as_double();
    double Ke_spec = get_parameter("K_e_rpm").as_double();   // V / rpm (rotor)
    double Kt_spec = get_parameter("K_t").as_double();       // Nm / A  (rotor)
    double J_rotor = get_parameter("J_rotor").as_double();
    double J_wheel = get_parameter("J_wheel").as_double();
    V_bus_         = get_parameter("V_bus").as_double();

    /* --- константы двигателя на выходном валу --------------- */
    K_e_ = Ke_spec * 60.0 / TWO_PI * N;   // V / (rad/s) на колесе
    K_t_ = Kt_spec * N;                   // Nm/A  на колесе
    J_   = J_wheel + J_rotor * N * N;     // приведённый момент инерции

    /* ----- pigpio ------------------------------------------- */
    pi_ = pigpio_start(nullptr,nullptr);           // remote-mode
    if (pi_ >= 0)  remote_ = true;
    else {
      if (gpioInitialise() < 0)
        throw std::runtime_error("pigpio initialisation failed");
      remote_ = false;
    }

    /* обёртки ------------------------------------------------- */
    auto setMode = [this](unsigned g,unsigned m){
      remote_ ? set_mode (pi_,g,m) : gpioSetMode(g,m); };
    auto write   = [this](unsigned g,unsigned l){
      remote_ ? gpio_write(pi_,g,l) : gpioWrite  (g,l); };
    hwPWM_ = [this](unsigned g,unsigned freq,unsigned duty){
      return remote_
             ? hardware_PWM(pi_,g,freq,duty)
             : gpioHardwarePWM(g,freq,duty);
    };

    /* GPIO init ---------------------------------------------- */
    for (unsigned p:{AIN1,AIN2,BIN1,BIN2,STBY})
      { setMode(p,PI_OUTPUT); write(p,0); }
    for (unsigned p:{PWMA,PWMB}) setMode(p,PI_OUTPUT);
    write(STBY,1);

    write_ = write;

    /* QoS ----------------------------------------------------- */
    rclcpp::QoS qos_rel(rclcpp::KeepLast(1));
    rclcpp::QoS qos_be (rclcpp::KeepLast(10));
    qos_be.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos_be.durability (rclcpp::DurabilityPolicy::Volatile);

    /* ---- подписки/публикации ------------------------------- */
    auto imu_t   = get_parameter("imu_topic").as_string();
    auto enc_l_t = get_parameter("enc_left_topic").as_string();
    auto enc_r_t = get_parameter("enc_right_topic").as_string();
    auto cmd_l_t = get_parameter("cmd_left_topic").as_string();
    auto cmd_r_t = get_parameter("cmd_right_topic").as_string();

    imu_sub_  = create_subscription<Float32MultiArray>(
                  imu_t, qos_be,
                  std::bind(&MotorController::imuCb,this,std::placeholders::_1));
    enc_l_sub_= create_subscription<Float32>(
                  enc_l_t,qos_rel,
                  std::bind(&MotorController::encLCb,this,std::placeholders::_1));
    enc_r_sub_= create_subscription<Float32>(
                  enc_r_t,qos_rel,
                  std::bind(&MotorController::encRCb,this,std::placeholders::_1));
    cmd_l_sub_= create_subscription<Float32>(
                  cmd_l_t,qos_rel,
                  std::bind(&MotorController::cmdLCb,this,std::placeholders::_1));
    cmd_r_sub_= create_subscription<Float32>(
                  cmd_r_t,qos_rel,
                  std::bind(&MotorController::cmdRCb,this,std::placeholders::_1));

    left_fb_pub_  = create_publisher<Float32MultiArray>("motor_feedback/left", qos_rel);
    right_fb_pub_ = create_publisher<Float32MultiArray>("motor_feedback/right",qos_rel);

    RCLCPP_INFO(get_logger(),
                "MotorController ready (%s-mode, E-STOP %.1f°, Ke=%.4f, Kt=%.4f, V_static=%.2f)",
                remote_?"remote":"local",
                crit_angle_rad_/DEG2RAD, K_e_, K_t_, V_static_);
  }

  /* ---------------- shutdown ---------------------------------- */
  void shutdown()
  {
    RCLCPP_INFO(get_logger(),"MotorController shutting down.");
    hardStop();
    if (remote_) pigpio_stop(pi_); else { write_(STBY,0); gpioTerminate(); }
  }

  /* ---------------- callbacks --------------------------------- */
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
  void encLCb(const Float32::SharedPtr m){ omega_left_  = m->data*TWO_PI; }
  void encRCb(const Float32::SharedPtr m){ omega_right_ = m->data*TWO_PI; }
  void cmdLCb(const Float32::SharedPtr m){
    drive(clip(m->data), omega_left_,  BIN1,BIN2,PWMB, left_fb_pub_);
  }
  void cmdRCb(const Float32::SharedPtr m){
    drive(clip(m->data), omega_right_, AIN1,AIN2,PWMA, right_fb_pub_);
  }

  /* ---------------- мотор-логика ------------------------------ */
  double clip(double a) const {
    return std::clamp(a, -alpha_clip_, alpha_clip_);
  }

  /**
   * Управление мотором с учётом реверса и диапазона 10-100%
   */
  void drive(double alpha_d,double omega,
             unsigned in1,unsigned in2,unsigned pwm_pin,
             const rclcpp::Publisher<Float32MultiArray>::SharedPtr& pub)
  {
    if (emergency_stop_){ brake(in1,in2,pwm_pin); return; }

    /* требуемый момент и ток */
    double tau = J_ * alpha_d;
    double I   = tau / K_t_;

    /* требуемое напряжение */
    double V = R_a_ * I + K_e_ * omega;

    V = std::clamp(V, -V_bus_, V_bus_);

    /* Рассчитываем скважность в диапазоне 10-100% */
    double duty_pct = (V / V_bus_) * 100.0;
    duty_pct = std::clamp(duty_pct, -100.0, 100.0);

    unsigned duty_hw;
    if (duty_pct > 0.0) {
        // Движение вперёд
        duty_pct = 10.0 + duty_pct * 0.9;  // Пересчёт 0-100 → 10-100
        duty_hw = static_cast<unsigned>(std::round(duty_pct / 100.0 * PWM_FULL));
        write_(in1, 1); write_(in2, 0);
    } else if (duty_pct < 0.0) {
        // Реверс
        duty_pct = -10.0 + duty_pct * 0.9;  // Пересчёт -100-0 → -100--10
        duty_hw = static_cast<unsigned>(std::round(std::abs(duty_pct) / 100.0 * PWM_FULL));
        write_(in1, 0); write_(in2, 1);
    } else {
        // Полная остановка
        write_(in1, 0); write_(in2, 0);
        hwPWM_(pwm_pin, 0, 0);  // Отключаем PWM
        return;
    }

    /* Установка скважности для hardware_PWM */
    hwPWM_(pwm_pin, PWM_FREQ, duty_hw);

    /* feedback для лога */
    Float32MultiArray fb;
    fb.data = {static_cast<float>(alpha_d),
               static_cast<float>(duty_pct),
               static_cast<float>(V)};
    pub->publish(fb);
  }

  /**
   * Торможение мотора
   */
  void brake(unsigned in1,unsigned in2,unsigned pwm_pin){
    write_(in1,0); write_(in2,0);
    hwPWM_(pwm_pin,0,0);   // coast
  }

  /**
   * Полная остановка моторов
   */
  void hardStop(){
    brake(BIN1,BIN2,PWMB); brake(AIN1,AIN2,PWMA);
  }

  /* ---------------- data -------------------------------------- */
  double crit_angle_rad_{}, alpha_clip_{}, V_static_{};
  double R_a_{}, K_e_{}, K_t_{}, J_{}, V_bus_{};
  double omega_left_{0.0}, omega_right_{0.0};
  bool   emergency_stop_{false};

  /* pigpio */
  int  pi_{-1};
  bool remote_{false};

  /* wrappers */
  std::function<void(unsigned,unsigned)>               write_;
  std::function<int(unsigned,unsigned,unsigned)>       hwPWM_;

  /* ROS entities */
  rclcpp::Subscription<Float32MultiArray>::SharedPtr imu_sub_;
  rclcpp::Subscription<Float32>::SharedPtr           enc_l_sub_, enc_r_sub_;
  rclcpp::Subscription<Float32>::SharedPtr           cmd_l_sub_, cmd_r_sub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr    left_fb_pub_, right_fb_pub_;
};

/* ---------------- main ----------------------------------------- */
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
