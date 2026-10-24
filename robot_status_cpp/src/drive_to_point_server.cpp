// Action server that drives the turtlesim turtle to an (x, y) goal.
//
// Demonstrates:
//   * an action server        -> /drive_to_point  (robot_status_interfaces/action/DriveToPoint)
//   * goal / cancel / accepted callbacks, feedback and results
//   * running the long goal in its own thread so the executor keeps processing /turtle1/pose

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "robot_status_interfaces/action/drive_to_point.hpp"
#include "turtlesim/msg/pose.hpp"

// turtlesim's window spans roughly 0 .. 11.09 on both axes.
constexpr double kWorldMin = 0.0;
constexpr double kWorldMax = 11.0;
constexpr double kControlRateHz = 20.0;

class DriveToPointServer : public rclcpp::Node
{
public:
  using DriveToPoint = robot_status_interfaces::action::DriveToPoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<DriveToPoint>;
  using Pose = turtlesim::msg::Pose;
  using Twist = geometry_msgs::msg::Twist;

  DriveToPointServer()
  : Node("drive_to_point_server")
  {
    declare_parameter("distance_tolerance", 0.1);
    declare_parameter("linear_gain", 1.5);
    declare_parameter("angular_gain", 6.0);
    declare_parameter("max_linear_speed", 2.0);

    cmd_pub_ = create_publisher<Twist>("turtle1/cmd_vel", 10);
    pose_sub_ = create_subscription<Pose>(
      "turtle1/pose", 10,
      [this](const Pose::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        pose_ = *msg;
      });

    action_server_ = rclcpp_action::create_server<DriveToPoint>(
      this, "drive_to_point",
      [this](const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const DriveToPoint::Goal> goal) {
        return on_goal(uuid, goal);
      },
      [this](const std::shared_ptr<GoalHandle> goal_handle) {return on_cancel(goal_handle);},
      [this](const std::shared_ptr<GoalHandle> goal_handle) {on_accepted(goal_handle);});

    RCLCPP_INFO(get_logger(), "Drive-to-point action server ready");
  }

private:
  rclcpp_action::GoalResponse on_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const DriveToPoint::Goal> goal)
  {
    if (goal->x < kWorldMin || goal->x > kWorldMax || goal->y < kWorldMin || goal->y > kWorldMax) {
      RCLCPP_WARN(
        get_logger(), "Rejecting goal (%.2f, %.2f): outside the turtlesim world", goal->x, goal->y);
      return rclcpp_action::GoalResponse::REJECT;
    }
    // exchange() sets the flag and returns its old value in one atomic step.
    if (goal_active_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "Rejecting goal: already driving to another goal");
      return rclcpp_action::GoalResponse::REJECT;
    }
    RCLCPP_INFO(get_logger(), "Accepted goal (%.2f, %.2f)", goal->x, goal->y);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse on_cancel(const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_INFO(get_logger(), "Cancel requested");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void on_accepted(const std::shared_ptr<GoalHandle> goal_handle)
  {
    // Return quickly so the executor is not blocked; do the actual driving in a separate thread.
    std::thread{[this, goal_handle]() {
        execute(goal_handle);
        if (rclcpp::ok()) {
          stop();
        }
        goal_active_ = false;
      }}.detach();
  }

  void execute(const std::shared_ptr<GoalHandle> goal_handle)
  {
    const auto goal = goal_handle->get_goal();
    const double tolerance = get_parameter("distance_tolerance").as_double();
    const double k_lin = get_parameter("linear_gain").as_double();
    const double k_ang = get_parameter("angular_gain").as_double();
    const double max_speed = get_parameter("max_linear_speed").as_double();
    auto feedback = std::make_shared<DriveToPoint::Feedback>();
    rclcpp::Rate rate(kControlRateHz);

    while (rclcpp::ok()) {
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(make_result(false, "Goal canceled"));
        return;
      }

      std::optional<Pose> pose = current_pose();
      if (!pose) {
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Waiting for turtle1/pose...");
        rate.sleep();
        continue;
      }

      const double dx = goal->x - pose->x;
      const double dy = goal->y - pose->y;
      const double distance = std::hypot(dx, dy);

      if (distance < tolerance) {
        char buf[64];
        std::snprintf(buf, sizeof(buf), "Reached goal (error %.3f)", distance);
        goal_handle->succeed(make_result(true, buf));
        return;
      }

      // Simple proportional controller: turn towards the goal, and only drive forward
      // when roughly facing it (cos of the heading error scales the forward speed).
      double heading_error = std::atan2(dy, dx) - pose->theta;
      heading_error = std::atan2(std::sin(heading_error), std::cos(heading_error));

      Twist cmd;
      cmd.linear.x = std::min(k_lin * distance, max_speed) * std::max(0.0, std::cos(heading_error));
      cmd.angular.z = k_ang * heading_error;
      cmd_pub_->publish(cmd);

      feedback->distance_remaining = static_cast<float>(distance);
      goal_handle->publish_feedback(feedback);
      rate.sleep();
    }
    // rclcpp::ok() is false: the node is shutting down, so the goal can no longer be reported.
  }

  std::shared_ptr<DriveToPoint::Result> make_result(bool success, const std::string & message)
  {
    auto result = std::make_shared<DriveToPoint::Result>();
    result->success = success;
    result->message = message;
    if (auto pose = current_pose()) {
      result->final_x = pose->x;
      result->final_y = pose->y;
    }
    RCLCPP_INFO(get_logger(), "%s", message.c_str());
    return result;
  }

  std::optional<Pose> current_pose()
  {
    std::lock_guard<std::mutex> lock(pose_mutex_);
    return pose_;
  }

  void stop()
  {
    cmd_pub_->publish(Twist());
  }

  rclcpp::Publisher<Twist>::SharedPtr cmd_pub_;
  rclcpp::Subscription<Pose>::SharedPtr pose_sub_;
  rclcpp_action::Server<DriveToPoint>::SharedPtr action_server_;

  std::mutex pose_mutex_;
  std::optional<Pose> pose_;
  std::atomic<bool> goal_active_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DriveToPointServer>());
  rclcpp::shutdown();
  return 0;
}
