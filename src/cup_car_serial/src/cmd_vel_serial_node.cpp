#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <fcntl.h>
#include <glob.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace
{
speed_t baud_to_termios(int baud_rate)
{
  switch (baud_rate) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    default: throw std::invalid_argument("unsupported baud_rate: " + std::to_string(baud_rate));
  }
}

std::vector<std::string> glob_paths(const char * pattern)
{
  glob_t matches{};
  std::vector<std::string> paths;
  if (::glob(pattern, 0, nullptr, &matches) == 0) {
    for (size_t index = 0; index < matches.gl_pathc; ++index) {
      paths.emplace_back(matches.gl_pathv[index]);
    }
  }
  ::globfree(&matches);
  return paths;
}

std::string detect_serial_device()
{
  const char * patterns[] = {
    "/dev/serial/by-id/*",
    "/dev/ttyUSB*",
    "/dev/ttyACM*",
  };

  for (const char * pattern : patterns) {
    const auto paths = glob_paths(pattern);
    if (!paths.empty()) {
      return paths.front();
    }
  }
  return "";
}

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort() {close();}

  void open(const std::string & device, int baud_rate)
  {
    close();
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      throw std::runtime_error("open " + device + ": " + std::strerror(errno));
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
      const std::string error = std::strerror(errno);
      close();
      throw std::runtime_error("tcgetattr: " + error);
    }

    const speed_t speed = baud_to_termios(baud_rate);
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~(IGNBRK | IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
      const std::string error = std::strerror(errno);
      close();
      throw std::runtime_error("tcsetattr: " + error);
    }
    tcflush(fd_, TCIOFLUSH);
  }

  ssize_t read_available(char * buffer, size_t capacity)
  {
    const ssize_t received = ::read(fd_, buffer, capacity);
    if (received >= 0) {
      return received;
    }
    if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
      return 0;
    }
    throw std::runtime_error("serial read: " + std::string(std::strerror(errno)));
  }

  void write_all(const std::string & data)
  {
    const char * buffer = data.data();
    size_t remaining = data.size();
    while (remaining > 0) {
      const ssize_t written = ::write(fd_, buffer, remaining);
      if (written < 0) {
        if (errno == EINTR) {
          continue;
        }
        throw std::runtime_error("serial write: " + std::string(std::strerror(errno)));
      }
      buffer += written;
      remaining -= static_cast<size_t>(written);
    }
  }

  void close()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
  }

private:
  int fd_{-1};
};
}  // namespace

class CmdVelSerialNode : public rclcpp::Node
{
public:
  CmdVelSerialNode()
  : Node("cmd_vel_serial_node")
  {
    configured_device_ = declare_parameter<std::string>("device", "auto");
    baud_rate_ = declare_parameter<int>("baud_rate", 115200);
    topic_ = declare_parameter<std::string>("topic", "/cmd_vel_nav");
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 20.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.4);

    connectedPublisher_ = create_publisher<std_msgs::msg::Bool>(
      "/cup_car_serial/connected", rclcpp::QoS(1).transient_local().reliable());
    receivePublisher_ = create_publisher<std_msgs::msg::String>(
      "/cup_car_serial/rx", 10);

    if (send_rate_hz_ <= 0.0 || command_timeout_s_ <= 0.0) {
      throw std::invalid_argument("send_rate_hz and command_timeout_s must be positive");
    }

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        vx_mps_ = message->linear.x;
        az_radps_ = message->angular.z;
        last_command_time_ = now();
        received_command_ = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / send_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelSerialNode::send_command, this));

    connect();
    RCLCPP_INFO(
      get_logger(), "Forwarding %s to serial device '%s' at %d baud", topic_.c_str(),
      configured_device_.c_str(), baud_rate_);
  }

private:
  void connect()
  {
    last_connect_attempt_ = std::chrono::steady_clock::now();
    const std::string device = configured_device_ == "auto" ?
      detect_serial_device() : configured_device_;
    if (device.empty()) {
      set_connected(false);
      RCLCPP_ERROR(
        get_logger(),
        "No serial device found; searched /dev/serial/by-id, /dev/ttyUSB*, and /dev/ttyACM*");
      return;
    }

    try {
      serial_.open(device, baud_rate_);
      active_device_ = device;
      serial_connected_ = true;
      receive_buffer_.clear();
      set_connected(true);
      RCLCPP_INFO(get_logger(), "Serial port connected: %s", active_device_.c_str());
    } catch (const std::exception & error) {
      serial_connected_ = false;
      active_device_.clear();
      set_connected(false);
      RCLCPP_ERROR(get_logger(), "Cannot open serial port: %s", error.what());
    }
  }

  void set_connected(bool connected)
  {
    if (connection_state_published_ && connected == last_connection_state_) {
      return;
    }
    std_msgs::msg::Bool message;
    message.data = connected;
    connectedPublisher_->publish(message);
    last_connection_state_ = connected;
    connection_state_published_ = true;
  }

  void receive_feedback()
  {
    char buffer[256];
    while (serial_connected_) {
      const ssize_t received = serial_.read_available(buffer, sizeof(buffer));
      if (received == 0) {
        break;
      }
      receive_buffer_.append(buffer, static_cast<size_t>(received));
    }

    size_t newline;
    while ((newline = receive_buffer_.find('\n')) != std::string::npos) {
      std::string line = receive_buffer_.substr(0, newline);
      receive_buffer_.erase(0, newline + 1);
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (!line.empty()) {
        std_msgs::msg::String message;
        message.data = line;
        receivePublisher_->publish(message);
      }
    }

    if (receive_buffer_.size() > 4096) {
      RCLCPP_WARN(get_logger(), "Discarding oversized serial receive buffer");
      receive_buffer_.clear();
    }
  }

  void send_command()
  {
    double vx = 0.0;
    double az = 0.0;
    if (received_command_ && (now() - last_command_time_).seconds() <= command_timeout_s_) {
      vx = vx_mps_;
      az = az_radps_;
    }

    if (!std::isfinite(vx) || !std::isfinite(az)) {
      RCLCPP_WARN(get_logger(), "Ignoring non-finite velocity command");
      vx = 0.0;
      az = 0.0;
    }

    char frame[64];
    const int payload_length = std::snprintf(frame, sizeof(frame), "%.3f,%.3f", vx, az);
    if (payload_length <= 0 || static_cast<size_t>(payload_length) + 2 > sizeof(frame)) {
      RCLCPP_ERROR(get_logger(), "Velocity command cannot be encoded");
      return;
    }
    frame[payload_length] = '\r';
    frame[payload_length + 1] = '\n';
    const size_t frame_length = static_cast<size_t>(payload_length) + 2;

    if (!serial_connected_) {
      const auto elapsed = std::chrono::steady_clock::now() - last_connect_attempt_;
      if (elapsed < std::chrono::seconds(1)) {
        return;
      }
      connect();
      if (!serial_connected_) {
        return;
      }
    }

    try {
      serial_.write_all(std::string(frame, frame_length));
      receive_feedback();
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Serial communication failed: %s", error.what());
      serial_.close();
      serial_connected_ = false;
      active_device_.clear();
      set_connected(false);
    }
  }

  std::string configured_device_;
  std::string active_device_;
  std::string topic_;
  std::string receive_buffer_;
  int baud_rate_{};
  double send_rate_hz_{};
  double command_timeout_s_{};
  SerialPort serial_;
  bool serial_connected_{false};
  bool connection_state_published_{false};
  bool last_connection_state_{false};
  bool received_command_{false};
  double vx_mps_{0.0};
  double az_radps_{0.0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_connect_attempt_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connectedPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr receivePublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSerialNode>());
  rclcpp::shutdown();
  return 0;
}
