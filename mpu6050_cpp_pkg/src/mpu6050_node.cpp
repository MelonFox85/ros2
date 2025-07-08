#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp> // <<< ИЗМЕНЕНО: Подключаем нужный тип сообщения
#include <unistd.h>

constexpr int     MPU        = 0x68;
constexpr uint8_t ACC_H      = 0x3B;
constexpr float   ACC_SENS   = 16384.0f;
constexpr float   G_SENS_DPS = 131.0f;
constexpr float   DEG2RAD    = 0.01745329252f;
constexpr float   G_IN_MSS   = 9.80665f; // Ускорение свободного падения

class Mpu6050Node : public rclcpp::Node
{
public:
  Mpu6050Node() : Node("mpu6050_reader_node") // <<< ИЗМЕНЕНО: Более стандартное имя узла
  {
    // <<< ИЗМЕНЕНО: Добавляем параметр для frame_id
    this->declare_parameter<std::string>("frame_id", "imu_link");
    frame_id_ = this->get_parameter("frame_id").as_string();

    fd_ = ::open("/dev/i2c-1", O_RDWR);
    if (fd_ < 0)
    {
      RCLCPP_FATAL(get_logger(), "Не удалось открыть /dev/i2c-1: %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    if (::ioctl(fd_, I2C_SLAVE, MPU) < 0)
    {
      RCLCPP_FATAL(get_logger(), "Не удалось выполнить ioctl(I2C_SLAVE): %s", strerror(errno));
      rclcpp::shutdown();
      return;
    }

    init_chip();

    // <<< ИЗМЕНЕНО: Тип паблишера и название топика
    // Публикуем в топик "imu/data_raw", стандартный для необработанных данных IMU
    pub_ = create_publisher<sensor_msgs::msg::Imu>(
             "imu/data_raw", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(std::chrono::microseconds(2500), // 400 Гц
                               std::bind(&Mpu6050Node::tick, this));

    RCLCPP_INFO(get_logger(), "Узел MPU-6050 запущен. Публикация в '%s' с frame_id '%s'",
      pub_->get_topic_name(), frame_id_.c_str());
  }

  ~Mpu6050Node() override { if (fd_ >= 0) ::close(fd_); }

private:
  void write_reg(uint8_t reg, uint8_t val)
  {
    uint8_t buf[2] = {reg, val};
    i2c_msg msg{.addr = MPU, .flags = 0, .len = 2, .buf = buf};
    i2c_rdwr_ioctl_data xfer{.msgs = &msg, .nmsgs = 1};

    if (::ioctl(fd_, I2C_RDWR, &xfer) < 0)
    {
      RCLCPP_ERROR(get_logger(), "Ошибка записи в регистр I2C: %s", strerror(errno));
    }
  }

  void init_chip()
  {
    write_reg(0x6B, 0x00);
    usleep(50'000);
    write_reg(0x1A, 0x01);
    write_reg(0x19, 0x01);
    write_reg(0x1C, 0x00);
    write_reg(0x1B, 0x00);
  }

  void tick()
  {
    uint8_t reg = ACC_H;
    uint8_t raw[14];

    i2c_msg msgs[2] = {
      { .addr = MPU, .flags = 0,        .len = 1,  .buf = &reg },
      { .addr = MPU, .flags = I2C_M_RD, .len = 14, .buf =  raw }
    };
    i2c_rdwr_ioctl_data xfer{ .msgs = msgs, .nmsgs = 2 };

    if (::ioctl(fd_, I2C_RDWR, &xfer) < 0)
    {
      RCLCPP_WARN(get_logger(), "Ошибка чтения с шины I2C: %s", strerror(errno));
      return;
    }

    auto be16 = [](const uint8_t *p){ return int16_t(p[0] << 8 | p[1]); };
    int16_t ax_raw = be16(raw+0), ay_raw = be16(raw+2), az_raw = be16(raw+4);
    int16_t gx_raw = be16(raw+8), gy_raw = be16(raw+10), gz_raw = be16(raw+12);

    // <<< ИЗМЕНЕНО: Создаем и заполняем сообщение sensor_msgs::msg::Imu
    auto msg = std::make_unique<sensor_msgs::msg::Imu>();

    // Заполняем заголовок
    msg->header.stamp = this->get_clock()->now();
    msg->header.frame_id = frame_id_;

    // Заполняем данные. Акселерометр в м/с^2, гироскоп в рад/с.
    // Ваш код уже правильно конвертировал гироскоп в рад/с.
    // Теперь конвертируем акселерометр из g в м/с^2.
    msg->linear_acceleration.x = (ax_raw / ACC_SENS) * G_IN_MSS;
    msg->linear_acceleration.y = (ay_raw / ACC_SENS) * G_IN_MSS;
    msg->linear_acceleration.z = (az_raw / ACC_SENS) * G_IN_MSS;

    msg->angular_velocity.x = (gx_raw / G_SENS_DPS) * DEG2RAD;
    msg->angular_velocity.y = (gy_raw / G_SENS_DPS) * DEG2RAD;
    msg->angular_velocity.z = (gz_raw / G_SENS_DPS) * DEG2RAD;

    // Поле ориентации (quaternion) оставляем пустым (нули).
    // Фильтр вычислит его за нас.
    msg->orientation.x = 0.0;
    msg->orientation.y = 0.0;
    msg->orientation.z = 0.0;
    msg->orientation.w = 1.0; // Валидный кватернион

    // Заполняем ковариационные матрицы.
    // Если мы не знаем точность, ставим небольшое значение в диагональ
    // и -1 в первый элемент, чтобы показать, что матрица неизвестна.
    // Фильтр обычно использует эти значения.
    msg->orientation_covariance[0] = -1;
    msg->angular_velocity_covariance[0] = 0.02;
    msg->angular_velocity_covariance[4] = 0.02;
    msg->angular_velocity_covariance[8] = 0.02;
    msg->linear_acceleration_covariance[0] = 0.04;
    msg->linear_acceleration_covariance[4] = 0.04;
    msg->linear_acceleration_covariance[8] = 0.04;

    pub_->publish(std::move(msg));
  }

  int fd_;
  std::string frame_id_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mpu6050Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
