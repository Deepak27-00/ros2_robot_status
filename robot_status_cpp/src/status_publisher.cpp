// Fake battery/temperature sensor.
//
// Demonstrates:
//   * a publisher on a timer      -> /robot_status   (robot_status_interfaces/msg/RobotStatus)
//   * a service server            -> /reset_status   (robot_status_interfaces/srv/ResetStatus)
//   * parameters, including one that can be changed at runtime (publish_rate_hz)

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "robot_status_interfaces/msg/robot_status.hpp"
#include "robot_status_interfaces/srv/reset_status.hpp"

using RobotStatus = robot_status_interfaces::msg::RobotStatus;
using ResetStatus = robot_status_interfaces::srv::ResetStatus;

class StatusPublisher : public rclcpp::Node
{
public:
  StatusPublisher()
  : Node("status_publisher"), rng_(std::random_device{}())
  {
    // Parameters: declare them with defaults; values can come from YAML, launch or CLI.
    declare_parameter("publish_rate_hz", 1.0);
    declare_parameter("battery_drain_per_sec", 0.5);
    declare_parameter("low_battery_threshold", 20.0);

    last_tick_ = now();
    publisher_ = create_publisher<RobotStatus>("robot_status", 10);
    reset_srv_ = create_service<ResetStatus>(
      "reset_status",
      [this](const std::shared_ptr<ResetStatus::Request> request,
      std::shared_ptr<ResetStatus::Response> response) {handle_reset(request, response);});

    start_timer(get_parameter("publish_rate_hz").as_double());

    // "on_set" validates a change before it is applied; "post_set" reacts once it is applied.
    // The returned handles must be kept alive, otherwise the callbacks are unregistered.
    on_set_handle_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {return validate_parameters(params);});
    post_set_handle_ = add_post_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {apply_parameters(params);});

    RCLCPP_INFO(get_logger(), "Status publisher started");
  }

private:
  void start_timer(double rate_hz)
  {
    auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(1.0 / rate_hz));
    // Assigning a new timer destroys (and stops) the previous one.
    timer_ = create_wall_timer(period, [this]() {publish_status();});
    RCLCPP_INFO(get_logger(), "Publishing at %.2f Hz", rate_hz);
  }

  rcl_interfaces::msg::SetParametersResult validate_parameters(
    const std::vector<rclcpp::Parameter> & params)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    for (const auto & param : params) {
      if (param.get_name() == "publish_rate_hz" && param.as_double() <= 0.0) {
        result.successful = false;
        result.reason = "publish_rate_hz must be > 0";
      } else if (param.get_name() == "battery_drain_per_sec" && param.as_double() < 0.0) {
        result.successful = false;
        result.reason = "battery_drain_per_sec must be >= 0";
      }
    }
    return result;
  }

  void apply_parameters(const std::vector<rclcpp::Parameter> & params)
  {
    for (const auto & param : params) {
      if (param.get_name() == "publish_rate_hz") {
        start_timer(param.as_double());
      }
    }
  }

  void publish_status()
  {
    const rclcpp::Time stamp = now();
    const double dt = (stamp - last_tick_).seconds();
    last_tick_ = stamp;

    // Simulate: battery drains linearly, temperature creeps up as the battery empties, plus noise.
    const double drain = get_parameter("battery_drain_per_sec").as_double();
    battery_ = std::max(0.0, battery_ - drain * dt);
    const double target_temp = 30.0 + 0.2 * (100.0 - battery_);
    temperature_ += 0.1 * (target_temp - temperature_) + noise_(rng_);

    RobotStatus msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = "base_link";
    msg.battery_percentage = static_cast<float>(battery_);
    msg.battery_voltage = static_cast<float>(10.5 + 2.1 * (battery_ / 100.0));
    msg.temperature_celsius = static_cast<float>(temperature_);
    std::tie(msg.level, msg.message) = classify();

    publisher_->publish(msg);

    if (msg.level != RobotStatus::LEVEL_OK) {
      // Throttle so a persistent warning does not flood the console.
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "%s", msg.message.c_str());
    }
  }

  std::pair<uint8_t, std::string> classify() const
  {
    const double low = get_parameter("low_battery_threshold").as_double();
    char buf[64];
    if (battery_ <= 5.0) {
      std::snprintf(buf, sizeof(buf), "Battery critical: %.1f%%", battery_);
      return {RobotStatus::LEVEL_ERROR, buf};
    }
    if (temperature_ > 50.0) {
      std::snprintf(buf, sizeof(buf), "Overheating: %.1f C", temperature_);
      return {RobotStatus::LEVEL_ERROR, buf};
    }
    if (battery_ < low) {
      std::snprintf(buf, sizeof(buf), "Battery low: %.1f%%", battery_);
      return {RobotStatus::LEVEL_WARN, buf};
    }
    return {RobotStatus::LEVEL_OK, "All systems nominal"};
  }

  void handle_reset(
    const std::shared_ptr<ResetStatus::Request> request,
    std::shared_ptr<ResetStatus::Response> response)
  {
    if (!(request->battery_percentage > 0.0f && request->battery_percentage <= 100.0f)) {
      response->success = false;
      response->message = "battery_percentage must be in (0, 100]";
      return;
    }

    battery_ = request->battery_percentage;
    temperature_ = 30.0;
    char buf[64];
    std::snprintf(buf, sizeof(buf), "Battery reset to %.1f%%", battery_);
    response->success = true;
    response->message = buf;
    RCLCPP_INFO(get_logger(), "%s", buf);
  }

  double battery_{100.0};
  double temperature_{30.0};
  rclcpp::Time last_tick_;
  std::mt19937 rng_;
  std::normal_distribution<double> noise_{0.0, 0.2};

  rclcpp::Publisher<RobotStatus>::SharedPtr publisher_;
  rclcpp::Service<ResetStatus>::SharedPtr reset_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_handle_;
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr post_set_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StatusPublisher>());
  rclcpp::shutdown();
  return 0;
}
