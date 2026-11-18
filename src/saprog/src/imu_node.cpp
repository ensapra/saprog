// Linux/WSL serial backend. No GUI dependencies: ROS owns sensor lifetime and state.
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>
#include <fcntl.h>
#include <poll.h>
#include <sys/file.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "saprog/msg/imu_status.hpp"
#include "saprog/srv/connect_imu.hpp"
#include "saprog/imu_filter.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class ImuNode : public rclcpp::Node {
public:
  ImuNode() : Node("saprog_imu") {
    device_ = declare_parameter<std::string>("device", env("SAPROG_IMU_DEVICE"));
    baud_ = declare_parameter<int>("baud_rate", 115200);
    auto_connect_ = declare_parameter<bool>("auto_connect", true);
    frame_ = declare_parameter<std::string>("frame_id", "imu_link");
    helper_ = declare_parameter<std::string>("usb_helper", env("SAPROG_IMU_USB_HELPER"));
    filter_.magnetic_heading = declare_parameter<bool>("use_magnetometer", false);
    // Defaults match SparkFun calculated output; raw counts require explicit scales.
    accel_scale_ = declare_parameter<double>("accel_scale", 1.0);
    gyro_scale_ = declare_parameter<double>("gyro_scale", 1.0);
    mag_scale_ = declare_parameter<double>("mag_scale", 1.0);
    if (!baud_speed(baud_) || !std::isfinite(accel_scale_) || accel_scale_ <= 0 ||
      !std::isfinite(gyro_scale_) || gyro_scale_ <= 0 || !std::isfinite(mag_scale_) || mag_scale_ <= 0) {
      throw std::runtime_error("Invalid baud rate or IMU unit scales");
    }
    raw_pub_ = create_publisher<sensor_msgs::msg::Imu>("/saprog/imu/data_raw", rclcpp::SensorDataQoS());
    data_pub_ = create_publisher<sensor_msgs::msg::Imu>("/saprog/imu/data", rclcpp::SensorDataQoS());
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/saprog/imu/mag", rclcpp::SensorDataQoS());
    relative_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>("/saprog/imu/orientation", rclcpp::SensorDataQoS());
    log_pub_ = create_publisher<std_msgs::msg::String>("/saprog/imu/serial", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<saprog::msg::ImuStatus>("/saprog/imu/status", 10);
    connect_service_ = create_service<saprog::srv::ConnectImu>("/saprog/imu/connect",
      [this](const std::shared_ptr<saprog::srv::ConnectImu::Request> req,
      std::shared_ptr<saprog::srv::ConnectImu::Response> res) {
        if (fd_ >= 0 || helper_pid_ > 0) {
          res->message = "Already connected or attaching USB; disconnect first."; return;
        }
        if (req->baud_rate && !baud_speed(req->baud_rate)) {
          res->message = "Supported baud rates: 9600, 19200, 38400, 57600, 115200, 230400."; return;
        }
        if (req->baud_rate) { baud_ = req->baud_rate; }
        device_ = req->device;
        auto_connect_ = false;
        res->success = connect_device();
        res->message = message_;
      });
    add_trigger("disconnect", [this] {
      auto_connect_ = false; stop_helper(); disconnect(); return true;
    });
    add_trigger("refresh_ports", [this] { discover(); return true; });
    add_trigger("toggle_readings", [this] {
      resume_pending_ = false;
      if (!write_space()) { return false; }
      message_ = "Space toggle sent; waiting for sensor output."; return true;
    });
    add_trigger("zero", [this] {
      if (!fresh() || filter_.calibrating) { message_ = "Zero requires live orientation and no calibration in progress."; return false; }
      filter_.zero(); publish_relative(); message_ = "Current orientation saved as zero."; return true;
    });
    add_trigger("clear_zero", [this] {
      filter_.clear_zero(); if (fresh()) { publish_relative(); }
      message_ = "Zero cleared; displaying the sensor estimate."; return true;
    });
    add_trigger("calibrate", [this] {
      if (!fresh() || filter_.calibrating) { message_ = "Calibration requires live orientation and no calibration in progress."; return false; }
      filter_.start_calibration(); calibration_at_ = Clock::now();
      message_ = "Calibration started; hold flat and still for four seconds."; return true;
    });
    heading_service_ = create_service<std_srvs::srv::SetBool>("/saprog/imu/use_magnetometer",
      [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
      std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
        filter_.set_heading(req->data);
        res->success = true;
        res->message = "Heading mode changed; orientation restarts and zero is cleared.";
        message_ = res->message;
      });
    // Settings are startup parameters. Services own runtime transitions atomically.
    parameter_callback_ = add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = "Use IMU services for runtime changes; parameters configure startup.";
      return result;
    });
    discover();
    read_timer_ = create_wall_timer(5ms, [this] { poll_serial(); });
    status_timer_ = create_wall_timer(200ms, [this] { tick(); });
  }

  ~ImuNode() override { stop_helper(); disconnect(); }

private:
  int fd_ = -1, baud_ = 115200, helper_fd_ = -1;
  pid_t helper_pid_ = -1;
  bool auto_connect_ = false, resume_pending_ = false;
  std::string device_, frame_, helper_, pending_, helper_output_;
  std::string message_ = "Waiting for connection.";
  std::vector<std::string> ports_;
  std::string preferred_;
  saprog::imu::Filter filter_;
  std::uint64_t sample_count_ = 0, invalid_lines_ = 0;
  double accel_scale_=1, gyro_scale_=1, mag_scale_=1;
  Clock::time_point opened_{}, last_sample_{}, calibration_at_{}, helper_at_{};
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr raw_pub_, data_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;
  rclcpp::Publisher<geometry_msgs::msg::QuaternionStamped>::SharedPtr relative_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr log_pub_;
  rclcpp::Publisher<saprog::msg::ImuStatus>::SharedPtr status_pub_;
  rclcpp::Service<saprog::srv::ConnectImu>::SharedPtr connect_service_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr heading_service_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> triggers_;
  rclcpp::TimerBase::SharedPtr read_timer_, status_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_;

  static std::string env(const char * name) { const char * value = std::getenv(name); return value ? value : ""; }
  static speed_t baud_speed(unsigned baud) {
    switch (baud) {
      case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
      case 57600: return B57600; case 115200: return B115200; case 230400: return B230400;
      default: return 0;
    }
  }
  bool fresh() const {
    return fd_ >= 0 && sample_count_ > 0 && filter_.valid && Clock::now()-last_sample_ < 2s;
  }
  static std::string read_word(const std::filesystem::path & path) {
    std::ifstream file(path); std::string word; file >> word; return word;
  }
  void discover() {
    ports_.clear(); preferred_.clear(); std::vector<std::string> matches;
    std::error_code error;
    for (const auto & entry : std::filesystem::directory_iterator("/sys/class/tty", error)) {
      const auto name = entry.path().filename().string();
      if (name.rfind("ttyACM",0) != 0 && name.rfind("ttyUSB",0) != 0) { continue; }
      const std::string device = "/dev/"+name;
      ports_.push_back(device);
      auto parent = std::filesystem::canonical(entry.path()/"device", error);
      if (error) { error.clear(); continue; }
      while (parent != parent.root_path() && !parent.empty()) {
        if (read_word(parent/"idVendor") == "1b4f" && read_word(parent/"idProduct") == "9d0f") {
          matches.push_back(device); break;
        }
        parent = parent.parent_path();
      }
    }
    std::sort(ports_.begin(), ports_.end());
    if (matches.size() == 1) { preferred_ = matches.front(); }
  }
  void add_trigger(const std::string & name, std::function<bool()> fn) {
    triggers_.push_back(create_service<std_srvs::srv::Trigger>("/saprog/imu/"+name,
      [this, fn](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        res->success = fn(); res->message = message_;
      }));
  }
  bool fail(const std::string & text) { message_ = text; return false; }
  bool connect_device(bool allow_helper = true) {
    discover();
    const auto selected = device_.empty() ? preferred_ : device_;
    if (selected.empty()) {
      if (allow_helper && !helper_.empty() && !env("WSL_DISTRO_NAME").empty()) { return start_helper(); }
      return fail("IMU not identified; choose its port and Connect. Servo ports are never selected automatically.");
    }
    const int fd = ::open(selected.c_str(), O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if (fd < 0) { return fail("Could not open "+selected+": "+std::strerror(errno)); }
    if (flock(fd, LOCK_EX|LOCK_NB) < 0) { ::close(fd); return fail("Serial port is already owned by another process."); }
    termios settings{};
    if (tcgetattr(fd, &settings) < 0) { ::close(fd); return fail("Could not read serial settings."); }
    cfmakeraw(&settings);
    settings.c_cflag |= CLOCAL|CREAD;
    settings.c_cflag &= ~CRTSCTS;
    settings.c_cc[VMIN]=0; settings.c_cc[VTIME]=0;
    cfsetispeed(&settings, baud_speed(baud_)); cfsetospeed(&settings, baud_speed(baud_));
    if (tcsetattr(fd, TCSANOW, &settings) < 0) { ::close(fd); return fail("Could not configure serial port."); }
    fd_ = fd; device_ = selected; pending_.clear();
    const bool heading = filter_.magnetic_heading;
    filter_ = saprog::imu::Filter{}; filter_.magnetic_heading = heading;
    sample_count_=0; invalid_lines_=0; resume_pending_=true; opened_=Clock::now();
    message_ = "Connected to "+device_+"; waiting for ten-column calculated samples.";
    return true;
  }
  bool write_space() {
    if (fd_ < 0) { return fail("Not connected."); }
    if (::write(fd_, " ", 1) != 1) { return fail("Could not send Space toggle: "+std::string(std::strerror(errno))); }
    return true;
  }
  void disconnect() {
    if (fd_ >= 0) { ::close(fd_); fd_=-1; }
    pending_.clear(); resume_pending_=false;
    filter_.cancel_calibration("Disconnected; reconnect and calibrate again.");
    filter_.valid=false; filter_.clear_zero();
    message_="Disconnected.";
  }
  // USB attachment is OS setup, not sensor processing. Run the existing helper
  // asynchronously and report its output through ROS without blocking services.
  bool start_helper() {
    int pipes[2];
    if (pipe(pipes) < 0) { return fail("Could not create USB helper pipe."); }
    helper_pid_=fork();
    if (helper_pid_ == 0) {
      setpgid(0,0); ::close(pipes[0]);
      dup2(pipes[1],STDOUT_FILENO); dup2(pipes[1],STDERR_FILENO); ::close(pipes[1]);
      execl("/bin/bash", "bash", helper_.c_str(), static_cast<char *>(nullptr));
      _exit(127);
    }
    ::close(pipes[1]);
    if (helper_pid_ < 0) { ::close(pipes[0]); return fail("Could not start USB helper."); }
    setpgid(helper_pid_,helper_pid_);
    helper_fd_=pipes[0]; fcntl(helper_fd_,F_SETFL,O_NONBLOCK);
    fcntl(helper_fd_,F_SETFD,FD_CLOEXEC);
    helper_at_=Clock::now(); helper_output_.clear(); message_="Attaching IMU USB to WSL…";
    return true;
  }
  void stop_helper() {
    if (helper_pid_ > 0) { kill(-helper_pid_,SIGKILL); waitpid(helper_pid_,nullptr,0); helper_pid_=-1; }
    if (helper_fd_ >= 0) { ::close(helper_fd_); helper_fd_=-1; }
  }
  void poll_helper() {
    if (helper_pid_ <= 0) { return; }
    char buffer[1024]; ssize_t n;
    while ((n=::read(helper_fd_,buffer,sizeof(buffer))) > 0) {
      if (helper_output_.size() < 8192) { helper_output_.append(buffer,n); }
    }
    int status=0;
    if (waitpid(helper_pid_,&status,WNOHANG) == helper_pid_) {
      helper_pid_=-1; ::close(helper_fd_); helper_fd_=-1;
      if (WIFEXITED(status) && WEXITSTATUS(status)==0) {
        const auto first=helper_output_.find_first_not_of(" \r\n\t");
        const auto last=helper_output_.find_last_not_of(" \r\n\t");
        if (first != std::string::npos) { device_=helper_output_.substr(first,last-first+1); }
        connect_device(false);
      } else { message_="USB setup failed: "+helper_output_; }
    } else if (Clock::now()-helper_at_ > 35s) { stop_helper(); message_="USB attachment timed out."; }
  }
  void poll_serial() {
    if (fd_ < 0) { return; }
    pollfd descriptor{fd_,POLLIN,0};
    if (::poll(&descriptor,1,0)>0 && (descriptor.revents & (POLLERR|POLLHUP|POLLNVAL))) {
      disconnect(); message_="IMU disconnected or serial read failed."; return;
    }
    char buffer[4096];
    // Bounded work per callback keeps services responsive at high sensor rates.
    for (int chunk=0; chunk<16; ++chunk) {
      const auto n=::read(fd_,buffer,sizeof(buffer));
      if (n<0 && errno!=EAGAIN && errno!=EWOULDBLOCK && errno!=EINTR) {
        const std::string error=std::strerror(errno); disconnect(); message_="Serial read failed: "+error; return;
      }
      if (n<=0) { break; }
      for (ssize_t i=0; i<n; ++i) {
        if (buffer[i]=='\r' || buffer[i]=='\n') {
          if (!pending_.empty()) { handle_line(pending_); pending_.clear(); }
        } else { pending_+=buffer[i]; }
        if (pending_.size()>16384) { pending_.clear(); ++invalid_lines_; }
      }
    }
  }
  static geometry_msgs::msg::Quaternion quaternion(const saprog::imu::Quat & q) {
    geometry_msgs::msg::Quaternion result;
    result.w=q[0]; result.x=q[1]; result.y=q[2]; result.z=q[3]; return result;
  }
  static void vector(geometry_msgs::msg::Vector3 & dest, const saprog::imu::Vec & source, double scale) {
    dest.x=source[0]*scale; dest.y=source[1]*scale; dest.z=source[2]*scale;
  }
  void publish_relative() {
    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.stamp=now(); msg.header.frame_id=filter_.zeroed ? "imu_zero" : "imu_world";
    msg.quaternion=quaternion(filter_.relative()); relative_pub_->publish(msg);
  }
  void handle_line(const std::string & line) {
    std_msgs::msg::String log; log.data=line; log_pub_->publish(log);
    saprog::imu::Sample sample;
    if (!saprog::imu::parse_sample(line,sample)) { ++invalid_lines_; return; }
    for (int i=0; i<3; ++i) {
      sample.accel[i]*=accel_scale_; sample.gyro[i]*=gyro_scale_; sample.mag[i]*=mag_scale_;
      if (!std::isfinite(sample.accel[i]) || !std::isfinite(sample.gyro[i]) || !std::isfinite(sample.mag[i])) { ++invalid_lines_; return; }
    }
    ++sample_count_; last_sample_=Clock::now(); resume_pending_=false;
    if (sample_count_ == 1) { message_="Receiving IMU samples; C++ ROS filtering active."; }
    filter_.update(sample);
    sensor_msgs::msg::Imu msg;
    msg.header.stamp=now(); msg.header.frame_id=frame_;
    vector(msg.linear_acceleration,sample.accel,9.80665);
    vector(msg.angular_velocity,sample.gyro,saprog::imu::radians);
    msg.orientation_covariance[0]=-1; raw_pub_->publish(msg);
    msg.orientation=quaternion(filter_.attitude);
    msg.orientation_covariance[0]=filter_.valid ? 0 : -1;
    vector(msg.angular_velocity,saprog::imu::subtract(sample.gyro,filter_.bias),saprog::imu::radians);
    data_pub_->publish(msg);
    sensor_msgs::msg::MagneticField mag;
    mag.header=msg.header; vector(mag.magnetic_field,sample.mag,1e-6); mag_pub_->publish(mag);
    if (filter_.valid) { publish_relative(); }
  }
  void tick() {
    if (auto_connect_) { auto_connect_=false; connect_device(); }
    poll_helper();
    if (resume_pending_ && fd_>=0 && Clock::now()-opened_ > 1500ms) {
      resume_pending_=false;
      if (write_space()) { message_="No valid samples yet; sent one resume toggle."; }
    }
    if (filter_.calibrating && Clock::now()-calibration_at_ > 12s) {
      filter_.cancel_calibration("Calibration timed out: insufficient fresh samples.");
    }
    saprog::msg::ImuStatus msg;
    msg.header.stamp=now(); msg.header.frame_id=frame_;
    msg.connected=fd_>=0; msg.streaming=msg.connected && sample_count_>0 && Clock::now()-last_sample_<2s;
    msg.orientation_valid=fresh(); msg.zeroed=filter_.zeroed;
    msg.calibrating=filter_.calibrating; msg.calibrated=filter_.calibrated;
    msg.magnetic_heading=filter_.magnetic_heading; msg.correcting_tilt=filter_.correcting_tilt;
    msg.device=device_; msg.baud_rate=baud_; msg.ports=ports_; msg.message=message_;
    msg.calibration_message=filter_.calibration_status;
    msg.sample_count=sample_count_; msg.invalid_lines=invalid_lines_; msg.time_gaps=filter_.time_gaps;
    msg.sensor_time_ms=filter_.last_time_ms; msg.sample_hz=filter_.sample_hz;
    msg.peak_gyro_dps=filter_.peak_gyro_dps; msg.gyro_bias_dps=filter_.bias;
    status_pub_->publish(msg);
  }
};

int main(int argc, char ** argv) {
  rclcpp::init(argc,argv);
  try { rclcpp::spin(std::make_shared<ImuNode>()); }
  catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("saprog_imu"),"%s",error.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
