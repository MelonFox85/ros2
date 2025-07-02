/**
 * @file      pwm_motor_controller.cpp
 * @brief     Узел управления моторами через PWM с частотой 400 Гц
 * @author    Айрат (с помощью Copilot)
 * @date      2025-06-30
 *
 * @section description Описание
 *
 * Узел получает команды напряжения для двигателей и преобразует их в ШИМ-сигналы.
 * Реализует защитные механизмы, включая:
 * - Аварийную остановку при превышении критического угла наклона
 * - Ограничение напряжения
 * - Компенсацию мёртвой зоны двигателей
 *
 * @section subscriptions Подписки
 *
 * - **Топик**: `/mpu6050/filtered_data` (std_msgs/Float32MultiArray)
 *   - data[0] – угол наклона (рад). Используется для аварийной остановки.
 * - **Топик**: `/motor_cmd/left` (std_msgs/Float32)
 *   - Требуемое напряжение для левого двигателя (В)
 * - **Топик**: `/motor_cmd/right` (std_msgs/Float32)
 *   - Требуемое напряжение для правого двигателя (В)
 *
 * @section publications Публикации
 *
 * - **Топик**: `/motor_status` (std_msgs/Float32MultiArray)
 *   - [0] V_left_requested - запрошенное напряжение левого мотора (В)
 *   - [1] duty_left - итоговая скважность левого мотора (%)
 *   - [2] V_right_requested - запрошенное напряжение правого мотора (В)
 *   - [3] duty_right - итоговая скважность правого мотора (%)
 *   - [4] emergency_stop - флаг аварийной остановки (0/1)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <memory>
#include <functional>
#include <cmath>
#include <pigpiod_if2.h>  // Используем daemon API

using namespace std::chrono_literals;
using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

class PwmMotorController : public rclcpp::Node
{
public:
    PwmMotorController() : Node("pwm_motor_controller")
    {
        init_parameters();
        init_gpio();
        init_publishers_subscribers();
        
        // Таймер для работы с частотой 400 Гц
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
        RCLCPP_INFO(get_logger(), "PWM Motor Controller остановлен");
    }

private:
    // GPIO пины (BCM)
    static constexpr unsigned PWMA = 13, AIN1 = 27, AIN2 = 17;   // Правый мотор
    static constexpr unsigned PWMB = 12, BIN1 = 14, BIN2 = 4;    // Левый мотор
    static constexpr unsigned STBY = 15;                         // Standby
    
    // Настройки ШИМ
    static constexpr unsigned PWM_FREQ = 10000;                  // 10 кГц
    static constexpr unsigned PWM_RANGE = 1000000;               // Диапазон для hardware_PWM
    
    // Константы преобразования
    static constexpr double DEG2RAD = M_PI / 180.0;
    static constexpr double RAD2DEG = 180.0 / M_PI;
    
    // Инициализация параметров
    void init_parameters()
    {
        // Объявление параметров с описаниями
        auto desc = rcl_interfaces::msg::ParameterDescriptor{};
        
        desc.description = "Критический угол наклона (градусы) для аварийной остановки";
        critical_angle_deg_ = declare_parameter<double>("critical_angle_deg", 60.0, desc);
        critical_angle_rad_ = critical_angle_deg_ * DEG2RAD;
        
        desc.description = "Максимальное напряжение на двигатели (В)";
        max_voltage_ = declare_parameter<double>("max_voltage", 12.0, desc);
        
        desc.description = "Минимальная скважность (%) для преодоления мёртвой зоны двигателей";
        min_duty_pct_ = declare_parameter<double>("min_duty_pct", 7.5, desc);
        
        desc.description = "Инвертировать направление левого двигателя";
        invert_left_ = declare_parameter<bool>("invert_left", false, desc);
        
        desc.description = "Инвертировать направление правого двигателя";
        invert_right_ = declare_parameter<bool>("invert_right", false, desc);
    }
    
    // Инициализация GPIO через pigpiod
    void init_gpio()
    {
        // Подключение к демону pigpiod
        pi_ = pigpio_start(nullptr, nullptr);
        if (pi_ < 0) {
            RCLCPP_ERROR(get_logger(), "Не удалось подключиться к pigpiod. Запустите 'sudo pigpiod'.");
            throw std::runtime_error("Не удалось подключиться к pigpiod");
        }
        
        // Настройка пинов
        std::vector<unsigned> output_pins = {AIN1, AIN2, BIN1, BIN2, STBY, PWMA, PWMB};
        for (auto pin : output_pins) {
            set_mode(pi_, pin, PI_OUTPUT);
            gpio_write(pi_, pin, 0);
        }
        
        // Включаем H-мост (выход из режима standby)
        gpio_write(pi_, STBY, 1);
    }
    
    // Инициализация издателей и подписчиков
    void init_publishers_subscribers()
    {
        // QoS профили
        auto qos_reliable = rclcpp::QoS(rclcpp::KeepLast(10));
        auto qos_best_effort = rclcpp::QoS(rclcpp::KeepLast(10));
        qos_best_effort.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        qos_best_effort.durability(rclcpp::DurabilityPolicy::Volatile);
        
        // Подписчики
        imu_sub_ = create_subscription<Float32MultiArray>(
            "mpu6050/filtered_data", qos_best_effort,
            std::bind(&PwmMotorController::imu_callback, this, std::placeholders::_1));
            
        left_cmd_sub_ = create_subscription<Float32>(
            "motor_cmd/left", qos_reliable,
            std::bind(&PwmMotorController::left_cmd_callback, this, std::placeholders::_1));
            
        right_cmd_sub_ = create_subscription<Float32>(
            "motor_cmd/right", qos_reliable,
            std::bind(&PwmMotorController::right_cmd_callback, this, std::placeholders::_1));
        
        // Издатель статуса моторов
        motor_status_pub_ = create_publisher<Float32MultiArray>("motor_status", qos_reliable);
        
        // Инициализация сообщения статуса
        motor_status_msg_.data.resize(5, 0.0f);
    }
    
    // Обработчик данных IMU (для проверки наклона)
    void imu_callback(const Float32MultiArray::SharedPtr msg)
    {
        if (msg->data.empty()) return;
        
        double tilt_angle = msg->data[0];  // В радианах
        
        // Проверка критического угла наклона
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
            set_gpio(STBY, 1);  // Разблокировать H-мост
        }
    }
    
    // Обработчик команды для левого мотора
    void left_cmd_callback(const Float32::SharedPtr msg)
    {
        left_voltage_cmd_ = msg->data;
    }
    
    // Обработчик команды для правого мотора
    void right_cmd_callback(const Float32::SharedPtr msg)
    {
        right_voltage_cmd_ = msg->data;
    }
    
    // Основной цикл управления (400 Гц)
    void control_cycle()
    {
        if (emergency_stop_active_) {
            // В режиме аварийной остановки ничего не делаем
            publish_status();
            return;
        }
        
        // Применяем команды к моторам
        double left_voltage = left_voltage_cmd_;
        if (invert_left_) left_voltage = -left_voltage;
        
        double right_voltage = right_voltage_cmd_;
        if (invert_right_) right_voltage = -right_voltage;
        
        // Применяем команды с учетом ограничений
        double left_duty = apply_motor_command(left_voltage, BIN1, BIN2, PWMB);
        double right_duty = apply_motor_command(right_voltage, AIN1, AIN2, PWMA);
        
        // Публикуем статус
        motor_status_msg_.data[0] = static_cast<float>(left_voltage_cmd_);  // Запрошенное напряжение левого
        motor_status_msg_.data[1] = static_cast<float>(left_duty);          // Скважность левого (%)
        motor_status_msg_.data[2] = static_cast<float>(right_voltage_cmd_); // Запрошенное напряжение правого
        motor_status_msg_.data[3] = static_cast<float>(right_duty);         // Скважность правого (%)
        motor_status_msg_.data[4] = emergency_stop_active_ ? 1.0f : 0.0f;   // Флаг аварийной остановки
        
        publish_status();
    }
    
    // Применение команды к мотору и возврат итоговой скважности (%)
    double apply_motor_command(double voltage, unsigned in1, unsigned in2, unsigned pwm_pin)
    {
        // Ограничение напряжения
        voltage = std::clamp(voltage, -max_voltage_, max_voltage_);
        
        // Преобразование напряжения в скважность (-100% до +100%)
        double duty_pct = (voltage / max_voltage_) * 100.0;
        
        // Учет мёртвой зоны
        if (duty_pct > 0.0 && duty_pct < min_duty_pct_) {
            duty_pct = min_duty_pct_;
        } else if (duty_pct < 0.0 && duty_pct > -min_duty_pct_) {
            duty_pct = -min_duty_pct_;
        }
        
        // Применение скважности к мотору
        if (duty_pct > 0.0) {
            // Вращение вперед
            set_gpio(in1, 1);
            set_gpio(in2, 0);
            unsigned duty_hw = static_cast<unsigned>(std::round((duty_pct / 100.0) * PWM_RANGE));
            hardware_PWM(pi_, pwm_pin, PWM_FREQ, duty_hw);
        } else if (duty_pct < 0.0) {
            // Вращение назад
            set_gpio(in1, 0);
            set_gpio(in2, 1);
            unsigned duty_hw = static_cast<unsigned>(std::round((std::abs(duty_pct) / 100.0) * PWM_RANGE));
            hardware_PWM(pi_, pwm_pin, PWM_FREQ, duty_hw);
        } else {
            // Остановка (coast)
            set_gpio(in1, 0);
            set_gpio(in2, 0);
            hardware_PWM(pi_, pwm_pin, 0, 0);
        }
        
        return duty_pct;
    }
    
    // Аварийная остановка обоих моторов
    void emergency_stop()
    {
        // Отключаем ШИМ
        hardware_PWM(pi_, PWMA, 0, 0);
        hardware_PWM(pi_, PWMB, 0, 0);
        
        // Выключаем управляющие пины
        set_gpio(AIN1, 0);
        set_gpio(AIN2, 0);
        set_gpio(BIN1, 0);
        set_gpio(BIN2, 0);
        
        // Отключаем H-мост
        set_gpio(STBY, 0);
    }
    
    // Вспомогательная функция для установки GPIO
    void set_gpio(unsigned pin, unsigned level)
    {
        gpio_write(pi_, pin, level);
    }
    
    // Публикация статуса моторов
    void publish_status()
    {
        motor_status_pub_->publish(motor_status_msg_);
    }
    
    // Параметры узла
    double critical_angle_deg_;
    double critical_angle_rad_;
    double max_voltage_;
    double min_duty_pct_;
    bool invert_left_;
    bool invert_right_;
    
    // Состояние системы
    int pi_ = -1;  // Дескриптор подключения к pigpiod
    bool emergency_stop_active_ = false;
    double left_voltage_cmd_ = 0.0;
    double right_voltage_cmd_ = 0.0;
    
    // ROS компоненты
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<Float32MultiArray>::SharedPtr imu_sub_;
    rclcpp::Subscription<Float32>::SharedPtr left_cmd_sub_;
    rclcpp::Subscription<Float32>::SharedPtr right_cmd_sub_;
    rclcpp::Publisher<Float32MultiArray>::SharedPtr motor_status_pub_;
    Float32MultiArray motor_status_msg_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    
    try {
        auto node = std::make_shared<PwmMotorController>();
        rclcpp::spin(node);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("pwm_motor_controller"), "Ошибка: %s", e.what());
    }
    
    rclcpp::shutdown();
    return 0;
}
