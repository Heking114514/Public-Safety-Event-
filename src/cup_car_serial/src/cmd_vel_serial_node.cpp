#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
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
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
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

bool parse_int32(const std::string & text, int32_t * value)
{
  if (text.empty()) {
    return false;
  }

  char * end = nullptr;
  errno = 0;
  const long long parsed = std::strtoll(text.c_str(), &end, 10);
  if (errno != 0 || end == text.c_str() || *end != '\0' ||
    parsed < INT32_MIN || parsed > INT32_MAX)
  {
    return false;
  }
  *value = static_cast<int32_t>(parsed);
  return true;
}

bool parse_encoder_frame(const std::string & line, std::vector<int32_t> * values)
{
  if (line.rfind("ENC,", 0) != 0) {
    return false;
  }

  std::vector<std::string> fields;
  size_t start = 0;
  while (start <= line.size()) {
    const size_t comma = line.find(',', start);
    fields.push_back(line.substr(start, comma - start));
    if (comma == std::string::npos) {
      break;
    }
    start = comma + 1;
  }

  // Current firmware sends ENC,time_ms,sequence,left_total,right_total. Accept
  // the legacy ENC,left_total,right_total form while a controller is being updated.
  if (fields.size() != 5 && fields.size() != 3) {
    return false;
  }

  values->clear();
  values->reserve(4);
  if (fields.size() == 3) {
    int32_t left_total;
    int32_t right_total;
    if (!parse_int32(fields[1], &left_total) || !parse_int32(fields[2], &right_total)) {
      return false;
    }
    values->push_back(0);
    values->push_back(0);
    values->push_back(left_total);
    values->push_back(right_total);
    return true;
  }

  for (size_t index = 1; index < fields.size(); ++index) {
    int32_t value;
    if (!parse_int32(fields[index], &value)) {
      values->clear();
      return false;
    }
    values->push_back(value);
  }
  return true;
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
    rpy_topic_ = declare_parameter<std::string>("rpy_topic", "/imu/rpy");
    send_rate_hz_ = declare_parameter<double>("send_rate_hz", 20.0);
    command_timeout_s_ = declare_parameter<double>("command_timeout_s", 0.4);
    rpy_timeout_s_ = declare_parameter<double>("rpy_timeout_s", 0.4);

    connectedPublisher_ = create_publisher<std_msgs::msg::Bool>(
      "/cup_car_serial/connected", rclcpp::QoS(1).transient_local().reliable());
    receivePublisher_ = create_publisher<std_msgs::msg::String>(
      "/cup_car_serial/rx", 10);
    encoderPublisher_ = create_publisher<std_msgs::msg::Int32MultiArray>(
      "/cup_car_serial/encoder_ticks", 10);

    if (send_rate_hz_ <= 0.0 || command_timeout_s_ <= 0.0 || rpy_timeout_s_ <= 0.0) {
      throw std::invalid_argument(
              "send_rate_hz, command_timeout_s, and rpy_timeout_s must be positive");
    }

    subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      topic_, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        vx_mps_ = message->linear.x;
        az_radps_ = message->angular.z;
        last_command_time_ = now();
        received_command_ = true;
      });

    rpySubscription_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      rpy_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::Vector3Stamped::SharedPtr message) {
        roll_rad_ = message->vector.x;
        pitch_rad_ = message->vector.y;
        yaw_rad_ = message->vector.z;
        last_rpy_time_ = now();
        received_rpy_ = true;
      });

    const auto period = std::chrono::duration<double>(1.0 / send_rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelSerialNode::send_command, this));

    connect();
    RCLCPP_INFO(
      get_logger(), "Forwarding %s and %s to serial device '%s' at %d baud", topic_.c_str(),
      rpy_topic_.c_str(), configured_device_.c_str(), baud_rate_);
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
        publish_encoder_frame(line);
      }
    }

    if (receive_buffer_.size() > 4096) {
      RCLCPP_WARN(get_logger(), "Discarding oversized serial receive buffer");
      receive_buffer_.clear();
    }
  }

  void publish_encoder_frame(const std::string & line)
  {
    std::vector<int32_t> values;
    if (line.rfind("ENC,", 0) != 0) {
      return;
    }
    if (!parse_encoder_frame(line, &values)) {
      RCLCPP_WARN(get_logger(), "Ignoring malformed encoder frame: '%s'", line.c_str());
      return;
    }

    std_msgs::msg::Int32MultiArray message;
    // data: [mcu_time_ms, sequence, left_total_ticks, right_total_ticks]
    message.data = std::move(values);
    encoderPublisher_->publish(message);
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

    char rpy_frame[96];
    size_t rpy_frame_length = 0;
    const bool fresh_rpy = received_rpy_ &&
      (now() - last_rpy_time_).seconds() <= rpy_timeout_s_;
    if (fresh_rpy) {
      if (!std::isfinite(roll_rad_) || !std::isfinite(pitch_rad_) || !std::isfinite(yaw_rad_)) {
        RCLCPP_WARN(get_logger(), "Ignoring non-finite RPY command");
      } else {
        const int rpy_length = std::snprintf(
          rpy_frame, sizeof(rpy_frame), "RPY,%.6f,%.6f,%.6f\r\n", roll_rad_, pitch_rad_, yaw_rad_);
        if (rpy_length > 0 && static_cast<size_t>(rpy_length) < sizeof(rpy_frame)) {
          rpy_frame_length = static_cast<size_t>(rpy_length);
        } else {
          RCLCPP_ERROR(get_logger(), "RPY command cannot be encoded");
        }
      }
    }

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
      if (rpy_frame_length > 0) {
        serial_.write_all(std::string(rpy_frame, rpy_frame_length));
      }
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
  std::string rpy_topic_;
  std::string receive_buffer_;
  int baud_rate_{};
  double send_rate_hz_{};
  double command_timeout_s_{};
  double rpy_timeout_s_{};
  SerialPort serial_;
  bool serial_connected_{false};
  bool connection_state_published_{false};
  bool last_connection_state_{false};
  bool received_command_{false};
  bool received_rpy_{false};
  double vx_mps_{0.0};
  double az_radps_{0.0};
  double roll_rad_{0.0};
  double pitch_rad_{0.0};
  double yaw_rad_{0.0};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_rpy_time_{0, 0, RCL_ROS_TIME};
  std::chrono::steady_clock::time_point last_connect_attempt_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr rpySubscription_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connectedPublisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr receivePublisher_;
  rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr encoderPublisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelSerialNode>());
  rclcpp::shutdown();
  return 0;
}
