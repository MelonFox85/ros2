/**
 * cmd_publisher_circle.cpp
 * ------------------------------------------------------------
 * Генерирует и публикует опорный вектор состояния ("cmd")
 * для движения двухколесного робота по круговой траектории.
 *
 * Публикует в топик "cmd" сообщение Float32MultiArray
 * с заданной частотой (control_rate).
 *
 * Параметры:
 *  - control_rate: Частота публикации команд (Гц).
 *  - radius:       Радиус желаемой окружности (м), по умолчанию 0.5.
 *  - w_r:          Желаемая угловая скорость поворота (рад/с), по умолчанию 0.2.
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <chrono>
#include <cmath>

using Float32Array = std_msgs::msg::Float32MultiArray;
using namespace std::chrono_literals;

class CmdPublisherCircle : public rclcpp::Node
{
public:
  CmdPublisherCircle() : Node("cmd_publisher_circle")
  {
    /* -------- Параметры ------------------------------------- */
    // Глобальная частота цикла управления (Гц)
    double ctrl_rate = declare_parameter<double>("control_rate", 400.0);
    if (ctrl_rate <= 0.0) {
      RCLCPP_WARN(get_logger(), "control_rate <= 0, будет использовано значение 400 Гц");
      ctrl_rate = 400.0;
    }
    period_ = std::chrono::duration<double>(1.0 / ctrl_rate);

    // Параметры траектории
    radius_ = declare_parameter<double>("radius", 0.5);
    w_r_ = declare_parameter<double>("w_r", 0.2);

    RCLCPP_INFO(get_logger(), "Генератор команд для круговой траектории запущен.");
    RCLCPP_INFO(get_logger(), "Радиус (radius): %.2f м", radius_);
    RCLCPP_INFO(get_logger(), "Угловая скорость (w_r): %.2f рад/с", w_r_);

    /* -------- Публикатор ------------------------------------ */
    // Храним только последнюю команду (глубина очереди = 1)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    pub_ = create_publisher<Float32Array>("cmd", qos);

    /* -------- Таймер ---------------------------------------- */
    timer_ = create_wall_timer(period_, std::bind(&CmdPublisherCircle::tick, this));

    /* -------- Подготовка сообщения и переменных ------------- */
    msg_.data.resize(6); // Выделяем память для 6 элементов вектора
    start_time_ = this->now(); // Запоминаем время старта для вычисления t
  }

private:
  void tick()
  {
    // 1. Вычисляем текущее время t с момента старта узла
    const double t = (this->now() - start_time_).seconds();

    // 2. Рассчитываем компоненты опорного вектора состояния для движения по кругу
    const float v_ref = static_cast<float>(w_r_ * radius_); // Линейная скорость
    const float psi_ref = static_cast<float>(w_r_ * t);      // Угол рыскания
    const float s_ref = v_ref * static_cast<float>(t);       // Пройденный путь

    // 3. Заполняем вектор состояния `r = [s, v, θ, θ̇, ψ, ψ̇]`
    // r[0]: s_ref - желаемый пройденный путь (м)
    // r[1]: v_ref - желаемая линейная скорость (м/с)
    // r[2]: θ_ref - желаемый угол наклона (рад) - всегда 0 для баланса
    // r[3]: θ_dot_ref - желаемая скорость наклона (рад/с) - всегда 0
    // r[4]: ψ_ref - желаемый угол рыскания (рад)
    // r[5]: ψ_dot_ref - желаемая скорость рыскания (рад/с) - равна w_r
    msg_.data[0] = s_ref;
    msg_.data[1] = v_ref;
    msg_.data[2] = 0.0f;
    msg_.data[3] = 0.0f;
    msg_.data[4] = psi_ref;
    msg_.data[5] = static_cast<float>(w_r_);

    // 4. Публикуем сообщение
    pub_->publish(msg_);
  }

  /* ----------------- Поля класса -------------------------- */
  double radius_; // Радиус окружности (м)
  double w_r_;    // Угловая скорость (рад/с)

  rclcpp::Time start_time_; // Время запуска узла
  std::chrono::duration<double> period_;

  Float32Array msg_; // Буфер для переиспользования
  rclcpp::Publisher<Float32Array>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

/* ======================== main ============================== */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdPublisherCircle>());
  rclcpp::shutdown();
  return 0;
}
