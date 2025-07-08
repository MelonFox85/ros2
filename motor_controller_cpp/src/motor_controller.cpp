/**
 * @file      pwm_motor_controller.cpp
 * @brief     Узел управления моторами через PWM с частотой 400 Гц.
 * @author    Айрат (с помощью Copilot), обновлено для ROS 2 стандартов
 * @date      2025-07-07
 *
 * @section description Описание
 *
 * Узел получает команды напряжения для двигателей и преобразует их в ШИМ-сигналы,
 * используя библиотеку pigpiod для аппаратного ШИМ.
 * Реализует защитные механизмы, включая:
 * - Аварийную остановку при превышении критического угла наклона робота.
 * - Ограничение максимального напряжения на двигателях.
 * - Компенсацию "мёртвой зоны" двигателей для более точного управления на малых скоростях.
 *
 * @section subscriptions Подписки
 *
 * - **Топик**: `/imu/data` (sensor_msgs/msg/Imu)
 *   - Получает данные от IMU-фильтра. Угол наклона (pitch) вычисляется из
 *     кватерниона ориентации и используется для аварийной остановки.
 * - **Топик**: `/motor_cmd/left` (std_msgs/msg/Float32)
 *   - Требуемое напряжение для левого двигателя (в вольтах).
 * - **Топик**: `/motor_cmd/right` (std_msgs/msg/Float32)
 *   - Требуемое напряжение для правого двигателя (в вольтах).
 *
 * @section publications Публикации
 *
 * - **Топик**: `/motor_status` (std_msgs/msg/Float32MultiArray)
 *   - Публикует детальный статус работы моторов, включая запрошенное
 *     напряжение, итоговую скважность и флаг аварийной остановки.
 *   - [0] V_left_requested  (В) - запрошенное напряжение левого мотора
 *   - [1] duty_left         (%) - итоговая скважность левого мотора
 *   - [2] V_right_requested (В) - запрошенное напряжение правого мотора
 *   - [3] duty_right        (%) - итоговая скважность правого мотора
 *   - [4] emergency_stop    (0/1) - флаг аварийной остановки (1 - активна)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp> // <<< ИЗМЕНЕНО: Подключаем заголовок для Imu
#include <chrono>
#include <memory>
#include <functional>
#include <cmath>
#include <vector>
#include <pigpiod_if2.h>

using namespace std::chrono_literals;

// <<< ИЗМЕНЕНО: Добавляем псевдонимы для типов сообщений для краткости и ясности
using Float32Msg = std_msgs::msg::Float32;
using Float32ArrayMsg = std_msgs::msg::Float32MultiArray;
using ImuMsg = sensor_msgs::msg::Imu;


class PwmMotorController : public rclcpp::Node
{
public:
    PwmMotorController() : Node("pwm_motor_controller")
    {
        init_parameters();
        init_gpio();
        init_publishers_subscribers();
        
        timer_ = this->create_wall_timer(2500us, std::bind(&PwmMotorController::control_cycle, this));
        
        RCLCPP_INFO(get_logger(), "PWM Motor Controller запущен (частота: 400 Гц, E-STOP: %.1f°)",
                    critical_angle_deg_);
    }
    
    ~PwmMotorController() override
    {
        emergency_stop();
        if (pi_ >= 0) {
            set_gpio(STBY, 0);
            pigpio_stop(pi_);
        }
        RCLCPP_INFO(get_logger(), "PWM Motor Controller остановлен, GPIO освобождены.");
    }

private:
    // --- Конфигурация пинов и ШИМ ---
    static constexpr unsigned PWMA = 13, AIN1 = 27, AIN2 = 17;
    static constexpr unsigned PWMB = 12, BIN1 = 14, BIN2 = 4;
    static constexpr unsigned STBY = 15;
    static constexpr unsigned PWM_FREQ = 10000;
    static constexpr unsigned PWM_RANGE = 1000000;
    static constexpr double DEG2RAD = M_PI / 180.0;
    static constexpr double RAD2DEG = 180.0 / M_PI;
    
    void init_parameters()
    {
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        desc.description = "Критический угол наклона (градусы) для аварийной остановки.";
        critical_angle_deg_ = declare_parameter<double>("critical_angle_deg", 60.0, desc);
        critical_angle_rad_ = critical_angle_deg_ * DEG2RAD;
        
        desc.description = "Максимальное напряжение на двигатели (В).";
        max_voltage_ = declare_parameter<double>("max_voltage", 12.0, desc);
        
        desc.description = "Минимальная скважность (%) для преодоления мёртвой зоны двигателей.";
        min_duty_pct_ = declare_parameter<double>("min_duty_pct", 5.0, desc);
        
        desc.description = "Инвертировать направление левого двигателя.";
        invert_left_ = declare_parameter<bool>("invert_left", false, desc);
        
        desc.description = "Инвертировать направление правого двигателя.";
        invert_right_ = declare_parameter<bool>("invert_right", false, desc);
    }
    
    void init_gpio()
    {
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0) {
            RCLCPP_FATAL(get_logger(), "Не удалось подключиться к pigpiod. Убедитесь, что демон запущен: 'sudo pigpiod'.");
            throw std::runtime_error("Не удалось подключиться к pigpiod");
        }
        
        std::vector<unsigned> output_pins = {AIN1, AIN2, BIN1, BIN2, STBY, PWMA, PWMB};
        for (auto pin : output_pins) {
            set_mode(pi_, pin, PI_OUTPUT);
            gpio_write(pi_, pin, 0);
        }
        gpio_write(pi_, STBY, 1);
    }
    
    void init_publishers_subscribers()
    {
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10));
        // <<< ИЗМЕНЕНО: Используем стандартный профиль QoS для сенсоров
        auto qos_sensor = rclcpp::SensorDataQoS();
        
        // <<< ИЗМЕНЕНО: Подписка на новый топик и новый тип сообщения
        imu_sub_ = create_subscription<ImuMsg>(
            "imu/data", qos_sensor,
            std::bind(&PwmMotorController::imu_callback, this, std::placeholders::_1));
            
        left_cmd_sub_ = create_subscription<Float32Msg>(
            "motor_cmd/left", qos_reliable,
            std::bind(&PwmMotorController::left_cmd_callback, this, std::placeholders::_1));
            
        right_cmd_sub_ = create_subscription<Float32Msg>(
            "motor_cmd/right", qos_reliable,
            std::bind(&PwmMotorController::right_cmd_callback, this, std::placeholders::_1));
        
        motor_status_pub_ = create_publisher<Float32ArrayMsg>("motor_status", qos_reliable);
        motor_status_msg_.data.resize(5, 0.0f);
    }
    
    /**
     * @brief Вспомогательная функция для извлечения угла наклона (pitch) из кватерниона.
     * @param q Кватернион ориентации из сообщения sensor_msgs/msg/Imu.
     * @return Угол наклона в радианах.
     */
    static double get_pitch_from_quaternion(const ImuMsg::_orientation_type& q)
    {
        // Формула для pitch (поворот вокруг оси Y в системе координат ENU)
        // sin(pitch) = 2 * (w*y - z*x)
        double sinp = 2.0 * (q.w * q.y - q.z * q.x);
        // Ограничиваем значение, чтобы избежать ошибок в asinf из-за неточностей вычислений
        if (std::abs(sinp) >= 1)
            return std::copysign(M_PI / 2, sinp); // Возвращаем +/- 90 градусов
        else
            return std::asin(sinp);
    }

    // <<< ИЗМЕНЕНО: Колбэк адаптирован для работы с sensor_msgs/msg/Imu
    void imu_callback(const ImuMsg::SharedPtr msg)
    {
        // Вычисляем угол наклона из кватерниона
        double tilt_angle = get_pitch_from_quaternion(msg->orientation);
        
        if (std::abs(tilt_angle) > critical_angle_rad_) {
            if (!emergency_stop_active_) {
                RCLCPP_WARN(get_logger(), "АВАРИЙНАЯ ОСТАНОВКА: наклон %.1f° превышает критический %.1f°",
                           tilt_angle * RAD2DEG, critical_angle_deg_);
                emergency_stop_active_ = true;
                emergency_stop();
            }
        } else if (emergency_stop_active_) {
            RCLCPP_INFO(get_logger(), "Наклон в норме (%.1f°), возобновление работы",
                       tilt_angle * RAD2DEG);
            emergency_stop_active_ = false;
            set_gpio(STBY, 1);
        }
    }
    
    void left_cmd_callback(const Float32Msg::SharedPtr msg)
    {
        left_voltage_cmd_ = msg->data;
    }
    
    void right_cmd_callback(const Float32Msg::SharedPtr msg)
    {
        right_voltage_cmd_ = msg->data;
    }
    
    void control_cycle()
    {
        if (emergency_stop_active_) {
            publish_status();
            return;
        }
        
        double left_voltage = invert_left_ ? -left_voltage_cmd_ : left_voltage_cmd_;
        double right_voltage = invert_right_ ? -right_voltage_cmd_ : right_voltage_cmd_;
        
        double left_duty = apply_motor_command(left_voltage, BIN1, BIN2, PWMB);
        double right_duty = apply_motor_command(right_voltage, AIN1, AIN2, PWMA);
        
        motor_status_msg_.data[0] = static_cast<float>(left_voltage_cmd_);
        motor_status_msg_.data[1] = static_cast<float>(left_duty);
        motor_status_msg_.data[2] = static_cast<float>(right_voltage_cmd_);
        motor_status_msg_.data[3] = static_cast<float>(right_duty);
        motor_status_msg_.data[4] = emergency_stop_active_ ? 1.0f : 0.0f;
        
        publish_status();
    }
    
    double apply_motor_command(double voltage, unsigned in1, unsigned in2, unsigned pwm_pin)
    {
        voltage = std::clamp(voltage, -max_voltage_, max_voltage_);
        double duty_pct = (voltage / max_voltage_) * 100.0;
        
        if (std::abs(duty_pct) < 1e-3) { // Используем малое число для сравнения с нулем
            duty_pct = 0.0;
        } else if (std::abs(duty_pct) < min_duty_pct_) {
            duty_pct = 0.0;
        } else {
            duty_pct = std::clamp(duty_pct > 0 ? duty_pct + min_duty_pct_ : duty_pct - min_duty_pct_, -100.0, 100.0);
        }
        
        if (duty_pct > 0.0) {
            set_gpio(in1, 1); set_gpio(in2, 0);
            hardware_PWM(pi_, pwm_pin, PWM_FREQ, static_cast<unsigned>((duty_pct / 100.0) * PWM_RANGE));
        } else if (duty_pct < 0.0) {
            set_gpio(in1, 0); set_gpio(in2, 1);
            hardware_PWM(pi_, pwm_pin, PWM_FREQ, static_cast<unsigned>((std::abs(duty_pct) / 100.0) * PWM_RANGE));
        } else {
            set_gpio(in1, 0); set_gpio(in2, 0);
            hardware_PWM(pi_, pwm_pin, 0, 0);
        }
        
        return duty_pct;
    }
    
    void emergency_stop()
    {
        hardware_PWM(pi_, PWMA, 0, 0);
        hardware_PWM(pi_, PWMB, 0, 0);
        set_gpio(AIN1, 0); set_gpio(AIN2, 0);
        set_gpio(BIN1, 0); set_gpio(BIN2, 0);
        set_gpio(STBY, 0);
    }
    
    void set_gpio(unsigned pin, unsigned level)
    {
        gpio_write(pi_, pin, level);
    }
    
    void publish_status()
    {
        motor_status_pub_->publish(motor_status_msg_);
    }
    
    // --- Члены класса ---
    double critical_angle_deg_, critical_angle_rad_, max_voltage_, min_duty_pct_;
    bool invert_left_, invert_right_;
    
    int pi_ = -1;
    bool emergency_stop_active_ = false;
    double left_voltage_cmd_ = 0.0;
    double right_voltage_cmd_ = 0.0;
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_; // <<< ИЗМЕНЕНО: Тип указателя
    rclcpp::Subscription<Float32Msg>::SharedPtr left_cmd_sub_;
    rclcpp::Subscription<Float32Msg>::SharedPtr right_cmd_sub_;
    rclcpp::Publisher<Float32ArrayMsg>::SharedPtr motor_status_pub_;
    Float32ArrayMsg motor_status_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<PwmMotorController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_FATAL(rclcpp::get_logger("pwm_motor_controller"), "Критическая ошибка: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
