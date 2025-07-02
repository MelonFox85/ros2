#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include <array>
#include <numeric>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

using std_msgs::msg::Float32;
using std_msgs::msg::Float32MultiArray;

class LQRVoltageController : public rclcpp::Node
{
public:
  LQRVoltageController() : Node("lqr_controller_cpp")
  {
    // Создаем дескрипторы параметров для rqt_reconfigure
    auto double_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    double_param_desc.read_only = false;
    
    auto voltage_range = rcl_interfaces::msg::FloatingPointRange{};
    voltage_range.from_value = 0.0;
    voltage_range.to_value = 24.0;
    voltage_range.step = 0.1;
    double_param_desc.floating_point_range = {voltage_range};
    double_param_desc.description = "Максимальное напряжение (В)";
    
    auto bool_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    bool_param_desc.read_only = false;
    bool_param_desc.description = "Инвертировать направление";
    
    // Объявляем базовые параметры
    max_voltage_ = declare_parameter<double>("max_voltage", 12.0, double_param_desc);
    invert_left_ = declare_parameter<bool>("invert_left", false, bool_param_desc);
    invert_right_ = declare_parameter<bool>("invert_right", false, bool_param_desc);
    
    // Создаем и объявляем коэффициенты LQR как отдельные параметры
    // Коэффициенты для правого мотора
    right_K_ = {
      declare_parameter<double>("right.K_s", -11.352),
      declare_parameter<double>("right.K_theta", -82.573),
      declare_parameter<double>("right.K_psi", 144.35),
      declare_parameter<double>("right.K_v", -29.556),
      declare_parameter<double>("right.K_theta_dot", -4.7688),
      declare_parameter<double>("right.K_psi_dot", 0.89216)
    };
    
    // Коэффициенты для левого мотора
    left_K_ = {
      declare_parameter<double>("left.K_s", -11.352),
      declare_parameter<double>("left.K_theta", -82.573),
      declare_parameter<double>("left.K_psi", -144.35),
      declare_parameter<double>("left.K_v", -29.556),
      declare_parameter<double>("left.K_theta_dot", -4.7688),
      declare_parameter<double>("left.K_psi_dot", -0.89216)
    };
    
    // Настраиваем QoS
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    
    // Подписываемся на обратную связь
    feedback_sub_ = create_subscription<Float32MultiArray>(
      "feedback", qos,
      std::bind(&LQRVoltageController::feedbackCallback, this, std::placeholders::_1));
    
    // Создаем издатели для управляющих сигналов
    left_pub_ = create_publisher<Float32>("motor_cmd/left", qos);
    right_pub_ = create_publisher<Float32>("motor_cmd/right", qos);
    
    // Создаем издатели для отладочной информации о вкладах состояний
    left_debug_pub_ = create_publisher<Float32MultiArray>("lqr_debug/left", qos);
    right_debug_pub_ = create_publisher<Float32MultiArray>("lqr_debug/right", qos);
    
    // Инициализация отладочных сообщений
    left_debug_msg_.data.resize(6, 0.0f);
    right_debug_msg_.data.resize(6, 0.0f);
    
    // Добавляем обработчик изменения параметров
    param_callback_handle_ = this->add_on_set_parameters_callback(
      std::bind(&LQRVoltageController::parametersCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "LQR-регулятор запущен. Используйте rqt_reconfigure для настройки параметров.");
  }

private:
  // Обработчик для динамического изменения параметров
  rcl_interfaces::msg::SetParametersResult parametersCallback(
    const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    
    for (const auto &param : parameters) {
      const auto &name = param.get_name();
      
      // Обновление базовых параметров
      if (name == "max_voltage") max_voltage_ = param.as_double();
      else if (name == "invert_left") invert_left_ = param.as_bool();
      else if (name == "invert_right") invert_right_ = param.as_bool();
      
      // Обновление коэффициентов для правого мотора
      else if (name == "right.K_s") right_K_[0] = param.as_double();
      else if (name == "right.K_theta") right_K_[1] = param.as_double();
      else if (name == "right.K_psi") right_K_[2] = param.as_double();
      else if (name == "right.K_v") right_K_[3] = param.as_double();
      else if (name == "right.K_theta_dot") right_K_[4] = param.as_double();
      else if (name == "right.K_psi_dot") right_K_[5] = param.as_double();
      
      // Обновление коэффициентов для левого мотора
      else if (name == "left.K_s") left_K_[0] = param.as_double();
      else if (name == "left.K_theta") left_K_[1] = param.as_double();
      else if (name == "left.K_psi") left_K_[2] = param.as_double();
      else if (name == "left.K_v") left_K_[3] = param.as_double();
      else if (name == "left.K_theta_dot") left_K_[4] = param.as_double();
      else if (name == "left.K_psi_dot") left_K_[5] = param.as_double();
    }
    
    RCLCPP_DEBUG(get_logger(), "Параметры LQR-регулятора обновлены");
    return result;
  }

  void feedbackCallback(const Float32MultiArray::SharedPtr msg)
  {
    const auto & d = msg->data;
    if (d.size() != 6) {
      RCLCPP_WARN(get_logger(), "Получен вектор состояния неверного размера (%zu вместо 6)", d.size());
      return;
    }
    
    // /feedback: [s, v, theta, theta_dot, psi, psi_dot]
    // LQR: [s, theta, psi, v, theta_dot, psi_dot]
    std::array<double, 6> x_lqr = {
      d[0], d[2], d[4], d[1], d[3], d[5]
    };
    
    // Вычисление вклада каждого состояния для правого мотора
    double v_right = 0.0;
    std::array<double, 6> right_contributions;
    for (size_t j = 0; j < 6; ++j) {
      right_contributions[j] = -right_K_[j] * x_lqr[j];
      v_right += right_contributions[j];
    }
    
    // Вычисление вклада каждого состояния для левого мотора
    double v_left = 0.0;
    std::array<double, 6> left_contributions;
    for (size_t j = 0; j < 6; ++j) {
      left_contributions[j] = -left_K_[j] * x_lqr[j];
      v_left += left_contributions[j];
    }
    
    // Применение инверсии и ограничения
    if (invert_left_)  v_left  = -v_left;
    if (invert_right_) v_right = -v_right;
    
    v_left  = std::clamp(v_left,  -max_voltage_, max_voltage_);
    v_right = std::clamp(v_right, -max_voltage_, max_voltage_);
    
    // Публикация управляющих сигналов
    left_pub_->publish(Float32().set__data(static_cast<float>(v_left)));
    right_pub_->publish(Float32().set__data(static_cast<float>(v_right)));
    
    // Публикация отладочной информации о вкладах
    for (size_t i = 0; i < 6; ++i) {
      left_debug_msg_.data[i] = static_cast<float>(left_contributions[i]);
      right_debug_msg_.data[i] = static_cast<float>(right_contributions[i]);
    }
    
    left_debug_pub_->publish(left_debug_msg_);
    right_debug_pub_->publish(right_debug_msg_);
  }

  // Параметры
  std::vector<double> left_K_;
  std::vector<double> right_K_;
  double max_voltage_;
  bool invert_left_;
  bool invert_right_;
  
  // ROS2 компоненты
  rclcpp::Subscription<Float32MultiArray>::SharedPtr feedback_sub_;
  rclcpp::Publisher<Float32>::SharedPtr left_pub_;
  rclcpp::Publisher<Float32>::SharedPtr right_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr left_debug_pub_;
  rclcpp::Publisher<Float32MultiArray>::SharedPtr right_debug_pub_;
  Float32MultiArray left_debug_msg_;
  Float32MultiArray right_debug_msg_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LQRVoltageController>());
  rclcpp::shutdown();
  return 0;
}
