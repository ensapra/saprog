// Linux/WSL serial backend. No GUI dependencies: ROS owns sensor lifetime and state.
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "saprog/imu_filter.hpp"
#include "saprog/imu_message_adapter.hpp"
#include "saprog/imu_transport.hpp"
#include "saprog/msg/imu_status.hpp"
#include "saprog/srv/connect_imu.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using Clock = std::chrono::steady_clock;

class ImuNode : public rclcpp::Node
{
public:
  ImuNode() : Node("saprog_imu")
  {
    device_ = declare_parameter<std::string>("device", env("SAPROG_IMU_DEVICE"));
    explicit_device_ = !device_.empty();
    baud_ = declare_parameter<int>("baud_rate", 115200);
    auto_connect_ = declare_parameter<bool>("auto_connect", true);
    frame_ = declare_parameter<std::string>("frame_id", "imu_link");
    helper_ = declare_parameter<std::string>("usb_helper", env("SAPROG_IMU_USB_HELPER"));
    filter_.magnetic_heading = declare_parameter<bool>("use_magnetometer", false);
    // Defaults match SparkFun calculated output; raw counts require explicit scales.
    accel_scale_ = declare_parameter<double>("accel_scale", 1.0);
    gyro_scale_ = declare_parameter<double>("gyro_scale", 1.0);
    mag_scale_ = declare_parameter<double>("mag_scale", 1.0);
    if (baud_ <= 0 || !std::isfinite(accel_scale_) || accel_scale_ <= 0 ||
        !std::isfinite(gyro_scale_) || gyro_scale_ <= 0 || !std::isfinite(mag_scale_) ||
        mag_scale_ <= 0) {
      throw std::runtime_error("Invalid baud rate or IMU unit scales");
    }
    raw_pub_ =
        create_publisher<sensor_msgs::msg::Imu>("/saprog/imu/data_raw", rclcpp::SensorDataQoS());
    data_pub_ =
        create_publisher<sensor_msgs::msg::Imu>("/saprog/imu/data", rclcpp::SensorDataQoS());
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("/saprog/imu/mag",
                                                                 rclcpp::SensorDataQoS());
    relative_pub_ = create_publisher<geometry_msgs::msg::QuaternionStamped>(
        "/saprog/imu/orientation", rclcpp::SensorDataQoS());
    log_pub_ =
        create_publisher<std_msgs::msg::String>("/saprog/imu/serial", rclcpp::SensorDataQoS());
    status_pub_ = create_publisher<saprog::msg::ImuStatus>("/saprog/imu/status", 10);
    connect_service_ = create_service<saprog::srv::ConnectImu>(
        "/saprog/imu/connect", [this](const std::shared_ptr<saprog::srv::ConnectImu::Request> req,
                                      std::shared_ptr<saprog::srv::ConnectImu::Response> res) {
          if (transport_.connected() || helper_process_.running()) {
            res->message = "Already connected or attaching USB; disconnect first.";
            return;
          }
          if (req->baud_rate && req->baud_rate != 9600 && req->baud_rate != 19200 &&
              req->baud_rate != 38400 && req->baud_rate != 57600 && req->baud_rate != 115200 &&
              req->baud_rate != 230400) {
            res->message = "Supported baud rates: 9600, 19200, 38400, 57600, 115200, 230400.";
            return;
          }
          if (req->baud_rate) {
            baud_ = req->baud_rate;
          }
          device_ = req->device;
          explicit_device_ = !device_.empty();
          auto_connect_ = true;
          res->success = connect_device();
          res->message = message_;
        });
    add_trigger("disconnect", [this] {
      auto_connect_ = false;
      stop_helper();
      disconnect();
      return true;
    });
    add_trigger("refresh_ports", [this] {
      discover();
      return true;
    });
    add_trigger("toggle_readings", [this] {
      resume_pending_ = false;
      if (!write_space()) {
        return false;
      }
      message_ = "Space toggle sent; waiting for sensor output.";
      return true;
    });
    add_trigger("zero", [this] {
      if (!fresh() || filter_.calibrating) {
        message_ = "Zero requires live orientation and no calibration in progress.";
        return false;
      }
      filter_.zero();
      publish_relative();
      message_ = "Current orientation saved as zero.";
      return true;
    });
    add_trigger("clear_zero", [this] {
      filter_.clear_zero();
      if (fresh()) {
        publish_relative();
      }
      message_ = "Zero cleared; displaying the sensor estimate.";
      return true;
    });
    add_trigger("calibrate", [this] {
      if (!fresh() || filter_.calibrating) {
        message_ = "Calibration requires live orientation and no calibration in progress.";
        return false;
      }
      filter_.start_calibration();
      calibration_at_ = Clock::now();
      message_ = "Calibration started; hold flat and still for four seconds.";
      return true;
    });
    heading_service_ = create_service<std_srvs::srv::SetBool>(
        "/saprog/imu/use_magnetometer",
        [this](const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
               std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
          filter_.set_heading(req->data);
          res->success = true;
          res->message = "Heading mode changed; orientation restarts and zero is cleared.";
          message_ = res->message;
        });
    // Settings are startup parameters. Services own runtime transitions atomically.
    parameter_callback_ =
        add_on_set_parameters_callback([](const std::vector<rclcpp::Parameter> &) {
          rcl_interfaces::msg::SetParametersResult result;
          result.successful = false;
          result.reason = "Use IMU services for runtime changes; parameters configure startup.";
          return result;
        });
    discover();
    read_timer_ = create_wall_timer(5ms, [this] { poll_serial(); });
    status_timer_ = create_wall_timer(200ms, [this] { tick(); });
  }

  ~ImuNode() override
  {
    stop_helper();
    disconnect();
  }

private:
  int baud_ = 115200;
  bool auto_connect_ = false, resume_pending_ = false;
  bool explicit_device_ = false;
  std::string device_, frame_, helper_;
  std::string message_ = "Waiting for connection.";
  std::vector<std::string> ports_;
  std::string preferred_;
  saprog::imu::SerialTransport transport_;
  saprog::imu::UsbHelperProcess helper_process_;
  saprog::imu::LineFramer framer_;
  saprog::imu::Filter filter_;
  std::uint64_t sample_count_ = 0, invalid_lines_ = 0;
  double accel_scale_ = 1, gyro_scale_ = 1, mag_scale_ = 1;
  Clock::time_point opened_{}, last_sample_{}, calibration_at_{};
  Clock::time_point retry_at_{};
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

  static std::string env(const char *name)
  {
    const char *value = std::getenv(name);
    return value ? value : "";
  }
  bool fresh() const
  {
    return transport_.connected() && sample_count_ > 0 && filter_.valid &&
           Clock::now() - last_sample_ < 2s;
  }
  void discover()
  {
    const auto devices = saprog::imu::SerialTransport::discover();
    ports_ = devices.ports;
    preferred_ = devices.preferred;
  }
  void add_trigger(const std::string &name, std::function<bool()> fn)
  {
    triggers_.push_back(create_service<std_srvs::srv::Trigger>(
        "/saprog/imu/" + name, [this, fn](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                          std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
          res->success = fn();
          res->message = message_;
        }));
  }
  bool fail(const std::string &text)
  {
    message_ = text;
    return false;
  }
  bool connect_device(bool allow_helper = true)
  {
    discover();
    const auto selected = explicit_device_ ? device_ : preferred_;
    if (selected.empty()) {
      if (allow_helper && !helper_.empty() && !env("WSL_DISTRO_NAME").empty()) {
        if (start_helper()) {
          retry_at_ = Clock::now() + 4s;
          return true;
        }
        retry_at_ = Clock::now() + 3s;
        return false;
      }
      retry_at_ = Clock::now() + 3s;
      return fail("IMU not identified; check the USB connection and restart the IMU backend. Servo "
                  "ports are never selected automatically; retrying automatically.");
    }
    std::string error;
    if (!transport_.open(selected, baud_, error)) {
      retry_at_ = Clock::now() + 3s;
      return fail(error);
    }
    device_ = selected;
    framer_ = saprog::imu::LineFramer{};
    const bool heading = filter_.magnetic_heading;
    filter_ = saprog::imu::Filter{};
    filter_.magnetic_heading = heading;
    sample_count_ = 0;
    invalid_lines_ = 0;
    resume_pending_ = true;
    opened_ = Clock::now();
    retry_at_ = opened_ + 3s;
    message_ = "Connected to " + device_ + "; waiting for ten-column calculated samples.";
    return true;
  }
  bool write_space()
  {
    std::string error;
    return transport_.write_space(error) || fail(error);
  }
  void disconnect()
  {
    transport_.close();
    framer_.clear();
    resume_pending_ = false;
    filter_.cancel_calibration("Disconnected; reconnect and calibrate again.");
    filter_.valid = false;
    filter_.clear_zero();
    message_ = "Disconnected.";
  }
  // USB attachment is OS setup, not sensor processing. Run the existing helper
  // asynchronously and report its output through ROS without blocking services.
  bool start_helper()
  {
    std::string error;
    if (!helper_process_.start(helper_, error)) {
      return fail(error);
    }
    message_ = "Attaching IMU USB to WSL…";
    return true;
  }
  void stop_helper()
  {
    helper_process_.stop();
  }
  void poll_helper()
  {
    if (!helper_process_.running()) {
      return;
    }
    const auto result = helper_process_.poll(Clock::now());
    if (result.state == saprog::imu::HelperState::succeeded) {
      std::string error;
      if (transport_.open(result.device, baud_, error)) {
        device_ = result.device;
        framer_ = saprog::imu::LineFramer{};
        const bool heading = filter_.magnetic_heading;
        filter_ = saprog::imu::Filter{};
        filter_.magnetic_heading = heading;
        sample_count_ = 0;
        invalid_lines_ = 0;
        resume_pending_ = true;
        opened_ = Clock::now();
        retry_at_ = opened_ + 3s;
        message_ = "Connected to " + device_ + "; waiting for ten-column calculated samples.";
      } else {
        message_ = error;
        retry_at_ = Clock::now() + 3s;
      }
    } else if (result.state == saprog::imu::HelperState::failed) {
      message_ = "USB setup failed; retrying: " + result.diagnostics;
      retry_at_ = Clock::now() + 30s;
    } else if (result.state == saprog::imu::HelperState::timed_out) {
      message_ = "USB attachment timed out; retrying.";
      retry_at_ = Clock::now() + 30s;
    }
  }
  void poll_serial()
  {
    if (!transport_.connected()) {
      return;
    }
    if (transport_.failed()) {
      disconnect();
      message_ = "IMU disconnected or serial read failed; reconnecting automatically.";
      retry_at_ = Clock::now() + 1s;
      return;
    }
    char buffer[4096];
    // Bounded work per callback keeps services responsive at high sensor rates.
    for (int chunk = 0; chunk < 16; ++chunk) {
      const auto n = transport_.read(buffer, sizeof(buffer));
      if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        const std::string error = std::strerror(errno);
        disconnect();
        message_ = "Serial read failed: " + error + "; reconnecting automatically.";
        retry_at_ = Clock::now() + 1s;
        return;
      }
      if (n <= 0) {
        break;
      }
      const auto errors_before = framer_.rejected_lines();
      for (const auto &line : framer_.append(buffer, static_cast<std::size_t>(n))) {
        handle_line(line);
      }
      invalid_lines_ += framer_.rejected_lines() - errors_before;
    }
  }
  void publish_relative()
  {
    geometry_msgs::msg::QuaternionStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = filter_.zeroed ? "imu_zero" : "imu_world";
    msg.quaternion = saprog::imu::to_ros_quaternion(filter_.relative());
    relative_pub_->publish(msg);
  }
  void handle_line(const std::string &line)
  {
    std_msgs::msg::String log;
    log.data = line;
    log_pub_->publish(log);
    saprog::imu::Sample sample;
    if (!saprog::imu::parse_sample(line, sample)) {
      ++invalid_lines_;
      return;
    }
    for (int i = 0; i < 3; ++i) {
      sample.accel[i] *= accel_scale_;
      sample.gyro[i] *= gyro_scale_;
      sample.mag[i] *= mag_scale_;
      if (!std::isfinite(sample.accel[i]) || !std::isfinite(sample.gyro[i]) ||
          !std::isfinite(sample.mag[i])) {
        ++invalid_lines_;
        return;
      }
    }
    ++sample_count_;
    last_sample_ = Clock::now();
    resume_pending_ = false;
    if (sample_count_ == 1) {
      message_ = "Receiving IMU samples; C++ ROS filtering active.";
    }
    filter_.update(sample);
    sensor_msgs::msg::Imu raw, data;
    sensor_msgs::msg::MagneticField mag;
    saprog::imu::make_ros_messages(sample, filter_, frame_, now(), raw, data, mag);
    raw_pub_->publish(raw);
    data_pub_->publish(data);
    mag_pub_->publish(mag);
    if (filter_.valid) {
      publish_relative();
    }
  }
  void tick()
  {
    if (auto_connect_ && !transport_.connected() && !helper_process_.running() &&
        Clock::now() >= retry_at_) {
      connect_device();
    }
    poll_helper();
    if (transport_.connected() && sample_count_ > 0 && Clock::now() - last_sample_ > 2s) {
      disconnect();
      message_ = "IMU samples stopped; reconnecting automatically.";
      retry_at_ = Clock::now() + 1s;
    } else if (transport_.connected() && sample_count_ == 0 && Clock::now() - opened_ > 8s) {
      disconnect();
      message_ = "No valid IMU samples; reopening the sensor automatically.";
      retry_at_ = Clock::now() + 1s;
    }
    if (resume_pending_ && transport_.connected() && Clock::now() - opened_ > 1500ms) {
      resume_pending_ = false;
      if (write_space()) {
        message_ = "No valid samples yet; sent one resume toggle.";
      }
    }
    if (filter_.calibrating && Clock::now() - calibration_at_ > 12s) {
      filter_.cancel_calibration("Calibration timed out: insufficient fresh samples.");
    }
    saprog::msg::ImuStatus msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_;
    msg.connected = transport_.connected();
    msg.streaming = msg.connected && sample_count_ > 0 && Clock::now() - last_sample_ < 2s;
    msg.orientation_valid = fresh();
    msg.zeroed = filter_.zeroed;
    msg.calibrating = filter_.calibrating;
    msg.calibrated = filter_.calibrated;
    msg.magnetic_heading = filter_.magnetic_heading;
    msg.correcting_tilt = filter_.correcting_tilt;
    msg.device = device_;
    msg.baud_rate = baud_;
    msg.ports = ports_;
    msg.message = message_;
    msg.calibration_message = filter_.calibration_status;
    msg.sample_count = sample_count_;
    msg.invalid_lines = invalid_lines_;
    msg.time_gaps = filter_.time_gaps;
    msg.sensor_time_ms = filter_.last_time_ms;
    msg.sample_hz = filter_.sample_hz;
    msg.peak_gyro_dps = filter_.peak_gyro_dps;
    msg.gyro_bias_dps = filter_.bias;
    status_pub_->publish(msg);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<ImuNode>());
  } catch (const std::exception &error) {
    RCLCPP_FATAL(rclcpp::get_logger("saprog_imu"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
