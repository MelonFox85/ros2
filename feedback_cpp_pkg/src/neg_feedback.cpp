/**
 * @file      neg_feedback.cpp
 * @author    GitHub Copilot (для Айрата)
 * @date      2025-07-09
 * @brief     Узел ROS 2 для вычисления вектора ошибки LQR-балансировщика.
 *
 * @details
 * Этот узел реализует контур отрицательной обратной связи, который является
 * ядром системы управления двухколесным роботом. Он подписывается на
 * три ключевых потока данных:
 * 1.  **Командный вектор (`/cmd`):** Целевые значения состояния, к которым
 *     должен стремиться робот (например, целевая скорость, целевой наклон).
 * 2.  **Данные ИМУ (`/imu/data`):** Реальные данные об ориентации робота в
 *     пространстве, в частности, угол наклона (pitch) и его производная.
 * 3.  **Данные одометрии (`/odom`):** Реальные данные о движении робота,
 *     включая его текущее положение, линейную скорость и скорость поворота.
 *
 * Основная задача узла — на основе этих данных сформировать и опубликовать
 * **вектор ошибки (`/feedback`)**, который вычисляется по формуле:
 * `δ = cmd - state`, где `state` — это текущий вектор состояния робота.
 *
 * @section changes ИЗМЕНЕНИЯ
 * - **Адаптация под `/odom`:** Узел переделан для работы с топиком `/odom`
 *   (тип `nav_msgs/msg/Odometry`) вместо кастомного топика скоростей колес.
 *   Это стандартный и предпочтительный подход в ROS 2.
 * - **Интеграция фильтра:** Для борьбы с шумом, присущим данным от энкодеров,
 *   реализован настраиваемый **фильтр скользящего среднего (SMA)**. Он
 *   применяется к данным о линейной и угловой скорости, полученным из
 *   одометрии, обеспечивая более стабильные и гладкие значения для
 *   контура управления.
 * - **Рефакторинг логики:** Внутренняя логика была переработана. Теперь
 *   колбэк от одометрии является основным триггером для всех вычислений,
 *   что обеспечивает синхронизацию и предсказуемость работы.
 *
 * @section topics ВХОДНЫЕ ТОПИКИ
 * - `/cmd` (std_msgs::msg::Float32MultiArray): Командный вектор-эталон.
 * - `/imu/data` (sensor_msgs::msg::Imu): Отфильтрованные данные ИМУ.
 * - `/odom` (nav_msgs::msg::Odometry): Данные от узла одометрии.
 *
 * @section topics ВЫХОДНОЙ ТОПИК
 * - `/feedback` (std_msgs::msg::Float32MultiArray): Вектор ошибки `δ = cmd - state`.
 */

#include <array>
#include <cmath>
#include <cstring>
#include <algorithm>
#include <memory>
#include <vector>
#include <deque>      // Для реализации буфера фильтра
#include <numeric>    // Для std::accumulate

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp> // Подключаем заголовок для Odometry

// Псевдонимы для типов сообщений для повышения читаемости кода
using Float32Array = std_msgs::msg::Float32MultiArray;
using ImuMsg = sensor_msgs::msg::Imu;
using OdomMsg = nav_msgs::msg::Odometry;

namespace qos = rclcpp;

class NegFeedback : public rclcpp::Node
{
public:
  NegFeedback() : Node("neg_feedback")
  {
    // --- Объявление и получение параметров ---
    // Размер окна для фильтра скользящего среднего. Чем больше значение,
    // тем более гладкими будут данные, но и тем больше будет инерционность (задержка).
    filter_window_size_ = declare_parameter("filter_window_size", 100);

    // --- Настройка QoS (Quality of Service) ---
    auto latest_qos = qos::QoS(qos::KeepLast(1));
    auto sensor_qos = qos::SensorDataQoS();

    // --- Создание подписчиков ---
    sub_cmd_ = create_subscription<Float32Array>(
      "cmd", latest_qos,
      [this](Float32Array::ConstSharedPtr msg)
      {
        copy_array_to(cmd_, *msg);
        have_cmd_ = true;
      });

    // Подписка на данные с IMU. Колбэк только сохраняет последние данные.
    sub_imu_ = create_subscription<ImuMsg>(
      "imu/data", sensor_qos,
      std::bind(&NegFeedback::imu_cb, this, std::placeholders::_1));
    
    // Подписка на данные одометрии. Этот колбэк является основным триггером.
    sub_odom_ = create_subscription<OdomMsg>(
      "odom", sensor_qos,
      std::bind(&NegFeedback::odom_cb, this, std::placeholders::_1));

    // --- Создание паблишера ---
    pub_ = create_publisher<Float32Array>("feedback", latest_qos);

    // Инициализация выходного сообщения
    msg_out_.data.resize(6, 0.0f);

    RCLCPP_INFO(get_logger(),
                "Узел neg_feedback запущен. Подписка на /odom и /imu/data. Размер окна фильтра: %d.",
                filter_window_size_);
  }

private:
  /**
   * @brief Безопасно копирует данные из сообщения Float32MultiArray в std::array.
   * @param dst Массив-приемник.
   * @param src Сообщение-источник.
   */
  static void copy_array_to(std::array<float,6>& dst,
                            const Float32Array&  src)
  {
    std::fill(dst.begin(), dst.end(), 0.0f);
    const size_t n = std::min<size_t>(6, src.data.size());
    if (n > 0) {
        std::memcpy(dst.data(), src.data.data(), n * sizeof(float));
    }
  }

  /**
   * @brief Вычисляет угол наклона (pitch) из кватерниона ориентации.
   * @param q Кватернион ориентации из сообщения Imu.
   * @return Угол наклона в радианах.
   */
  static double get_pitch_from_quaternion(const ImuMsg::_orientation_type& q)
  {
      double sinp = 2.0 * (q.w * q.y - q.z * q.x);
      if (std::abs(sinp) >= 1)
          return std::copysign(M_PI / 2, sinp);
      else
          return std::asin(sinp);
  }

  /**
   * @brief Колбэк для сообщений от IMU.
   * @details Эта функция вызывается каждый раз, когда приходит новое сообщение
   * от IMU. Ее единственная задача - извлечь и сохранить самые свежие данные
   * о состоянии робота (наклон и угловые скорости) в переменные-члены класса.
   * @param msg Указатель на полученное сообщение.
   */
  void imu_cb(const ImuMsg::ConstSharedPtr msg)
  {
    latest_pitch_      = static_cast<float>(get_pitch_from_quaternion(msg->orientation));
    latest_pitch_rate_ = static_cast<float>(msg->angular_velocity.y);
    latest_yaw_rate_   = static_cast<float>(msg->angular_velocity.z);
    have_imu_ = true; // Устанавливаем флаг, что у нас есть данные с IMU.
  }

  /**
   * @brief Колбэк для сообщений от одометрии (основной цикл).
   * @details Эта функция является сердцем узла. Она вызывается при получении
   * данных от энкодеров и запускает всю цепочку вычислений:
   * 1. Проверяет наличие всех необходимых данных (команды, IMU).
   * 2. Извлекает "сырые" скорости из сообщения одометрии.
   * 3. Применяет фильтр скользящего среднего для сглаживания скоростей.
   * 4. Собирает полный вектор состояния робота.
   * 5. Вычисляет вектор ошибки и публикует его.
   * @param msg Указатель на полученное сообщение.
   */
  void odom_cb(const OdomMsg::ConstSharedPtr msg)
  {
    // Выходим, если еще не получили начальную команду или данные с IMU.
    if (!have_cmd_ || !have_imu_) {
        RCLCPP_INFO_ONCE(get_logger(), "Ожидание данных от /cmd и /imu/data...");
        return;
    }

    // --- Фильтрация данных о скорости ---
    // Извлекаем "сырые" (зашумленные) скорости из сообщения.
    double raw_vx = msg->twist.twist.linear.x;
    double raw_wz = msg->twist.twist.angular.z;

    // Добавляем новые значения в буферы фильтра.
    vx_buffer_.push_back(raw_vx);
    wz_buffer_.push_back(raw_wz);

    // Поддерживаем постоянный размер буфера, удаляя самые старые значения.
    if (vx_buffer_.size() > static_cast<size_t>(filter_window_size_)) {
        vx_buffer_.pop_front();
        wz_buffer_.pop_front();
    }

    // Вычисляем среднее значение в буфере (результат работы фильтра).
    double filtered_vx = std::accumulate(vx_buffer_.begin(), vx_buffer_.end(), 0.0) / vx_buffer_.size();
    double filtered_wz = std::accumulate(wz_buffer_.begin(), wz_buffer_.end(), 0.0) / wz_buffer_.size();

    // --- Сборка вектора состояния (state) ---
    // [0] s - Положение. Берется напрямую из одометрии.
    state_[0] = static_cast<float>(msg->pose.pose.position.x);
    // [1] v - Линейная скорость. Используем отфильтрованное значение.
    state_[1] = static_cast<float>(filtered_vx);
    // [2] θ - Угол наклона. Используем последнее полученное значение от IMU.
    state_[2] = latest_pitch_;
    // [3] θ̇ - Скорость наклона. Используем последнее полученное значение от IMU.
    state_[3] = latest_pitch_rate_;
    // [4] ψ - Угол рыскания. Берется напрямую из одометрии (вычисляется из кватерниона).
    //     Примечание: для простоты можно использовать msg->pose.pose.orientation,
    //     но для LQR часто важен именно угол, а не кватернион.
    //     Здесь мы оставляем его пустым, так как он не используется в примере.
    //     Если он нужен, его нужно будет вычислить из odom.pose.pose.orientation.
    state_[4] = 0.0f; // TODO: Вычислить yaw из кватерниона, если необходимо.
    // [5] ψ̇ - Скорость рыскания. Используем отфильтрованное значение.
    state_[5] = static_cast<float>(filtered_wz);


    // --- Формирование и публикация вектора ошибки ---
    for (size_t i = 0; i < 6; ++i) {
      msg_out_.data[i] = cmd_[i] - state_[i];
    }
    pub_->publish(msg_out_);
  }

  // --- Переменные-члены класса ---
  std::array<float,6> cmd_{};        // Вектор-эталон (целевые значения)
  std::array<float,6> state_{};      // Текущий вектор состояния робота
  bool have_cmd_{false};             // Флаг: получена ли начальная команда
  bool have_imu_{false};             // Флаг: получены ли данные с IMU

  // Последние полученные данные с IMU
  float latest_pitch_{0.0f};
  float latest_pitch_rate_{0.0f};
  float latest_yaw_rate_{0.0f};

  // Переменные для фильтра
  int filter_window_size_;
  std::deque<double> vx_buffer_;
  std::deque<double> wz_buffer_;

  // Указатели на подписчиков и паблишер
  rclcpp::Subscription<Float32Array>::SharedPtr sub_cmd_;
  rclcpp::Subscription<ImuMsg>::SharedPtr       sub_imu_;
  rclcpp::Subscription<OdomMsg>::SharedPtr      sub_odom_;
  rclcpp::Publisher<Float32Array>::SharedPtr    pub_;
  Float32Array msg_out_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NegFeedback>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
