#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <unistd.h>

constexpr int     MPU        = 0x68;
constexpr uint8_t ACC_H      = 0x3B;
constexpr float   ACC_SENS   = 16384.0f;
constexpr float   G_SENS_DPS = 131.0f;
constexpr float   DEG2RAD    = 0.01745329252f;

class Mpu6050Node : public rclcpp::Node
{
public:
  Mpu6050Node() : Node("mpu6050_reader_cpp")
  {
    fd_ = ::open("/dev/i2c-1", O_RDWR);
    if (fd_ < 0)
      throw std::runtime_error("open /dev/i2c-1 failed");

    if (::ioctl(fd_, I2C_SLAVE, MPU) < 0)
      throw std::runtime_error("I2C_SLAVE ioctl failed");

    init_chip();

    pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(
             "mpu6050/raw", rclcpp::SensorDataQoS());

    timer_ = create_wall_timer(std::chrono::microseconds(2500),
                               std::bind(&Mpu6050Node::tick, this));

    RCLCPP_INFO(get_logger(), "MPU-6050 reader (C++) started");
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
      perror("I2C_RDWR(write_reg)");
      throw std::runtime_error("i2c write failed");
    }
  }

  void init_chip()
  {
    write_reg(0x6B, 0x00);          // PWR_MGMT_1 ← 0
    usleep(50'000);
    // --- ИСПРАВЛЕНИЕ ЗДЕСЬ: Отключаем DLPF для минимальной задержки ---
    // Устанавливаем значение 0x07, что соответствует "Filter bandwidth 260Hz, delay 0ms" для акселерометра
    // и "Filter bandwidth 256Hz, delay 0.98ms" для гироскопа. Это самый быстрый режим.
    write_reg(0x1A, 0x00);          // CONFIG (DLPF_CFG = 0)
    write_reg(0x19, 0x00);          // SMPLRT_DIV = 0  (1 кГц)
    write_reg(0x1C, 0x00);          // ACC_CFG  ±2 g
    write_reg(0x1B, 0x00);          // GYRO_CFG ±250 dps
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
      perror("I2C_RDWR(read)");
      return;
    }

    auto be16 = [](const uint8_t *p){ return int16_t(p[0] << 8 | p[1]); };
    int16_t ax = be16(raw+0), ay = be16(raw+2), az = be16(raw+4);
    int16_t gx = be16(raw+8), gy = be16(raw+10), gz = be16(raw+12);

    std_msgs::msg::Float32MultiArray msg;
    msg.data = {
      ax / ACC_SENS, ay / ACC_SENS, az / ACC_SENS,
      (gx / G_SENS_DPS) * DEG2RAD,
      (gy / G_SENS_DPS) * DEG2RAD,
      (gz / G_SENS_DPS) * DEG2RAD
    };
    pub_->publish(msg);
  }

  int fd_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Mpu6050Node>());
  rclcpp::shutdown();
  return 0;
}
