#include <tf2/LinearMath/Quaternion.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace robot_navigation {

using Waypoints = std::vector<geometry_msgs::msg::PoseStamped>;
using visualization_msgs::msg::MarkerArray;
using namespace std::chrono_literals;

namespace {
geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);  // Roll, Pitch, Yaw (in radians)
  return tf2::toMsg(q);
}

Waypoints load_waypoints_from_yaml(const std::string& file_path) {
  Waypoints waypoints;
  YAML::Node config = YAML::LoadFile(file_path);

  if (config["waypoints"]) {
    for (const auto& waypoint_node : config["waypoints"]) {
      geometry_msgs::msg::PoseStamped waypoint;
      waypoint.header.frame_id = "map";
      waypoint.header.stamp = rclcpp::Clock().now();

      // Load position
      waypoint.pose.position.x = waypoint_node["position"]["x"].as<double>();
      waypoint.pose.position.y = waypoint_node["position"]["y"].as<double>();

      // Load yaw and convert to quaternion
      double yaw = waypoint_node["yaw"].as<double>();
      waypoint.pose.orientation = yaw_to_quaternion(yaw);

      waypoints.push_back(waypoint);
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "No waypoints found in YAML file.");
  }

  return waypoints;
}
}  // namespace

class FollowWaypointsActionClient : public rclcpp::Node {
 public:
  using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
  using GoalHandleFollowWaypoints = rclcpp_action::ClientGoalHandle<FollowWaypoints>;

  explicit FollowWaypointsActionClient(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  void send_goal();

 private:
  rclcpp_action::Client<FollowWaypoints>::SharedPtr client_ptr_;
  rclcpp::Publisher<MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string waypoints_file_path_;

  void publish_waypoint_markers(const Waypoints& waypoints);
  void goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr& goal_handle);

  void feedback_callback(GoalHandleFollowWaypoints::SharedPtr,
                         const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
    // TODO:(optional) Implement feedback callback. Do something with the feedback received from the server.
  }

  void result_callback(const GoalHandleFollowWaypoints::WrappedResult& result);
};

FollowWaypointsActionClient::FollowWaypointsActionClient(const rclcpp::NodeOptions& options)
    : Node("follow_waypoints_action_client", options) {
  this->declare_parameter<std::string>("waypoints_file_path");
  this->get_parameter("waypoints_file_path", this->waypoints_file_path_);

  // TODO: Create action client for /follow_waypoints
  // this->client_ptr_ = ...

  // The goal will be sent 500ms after the node is started. The goal is only sent once because the timer is canceled in
  // the send_goal function. This ensures that node is fully initialized and spinning before sending the goal.
  timer_ = this->create_wall_timer(500ms, std::bind(&FollowWaypointsActionClient::send_goal, this));

  // Initialize marker publisher. This is used to visualize the waypoints in Rviz.
  auto custom_qos = rclcpp::QoS(1).transient_local();
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("waypoint_markers", custom_qos);
}

void FollowWaypointsActionClient::send_goal() {
  // Cancel the timer to send the goal only once
  this->timer_->cancel();

  // Block until the action server is available
  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  // Load waypoints from YAML file
  Waypoints waypoints;
  try {
    waypoints = load_waypoints_from_yaml(waypoints_file_path_);
  } catch (const YAML::BadFile& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from YAML file: %s", e.what());
    rclcpp::shutdown();
  }

  publish_waypoint_markers(waypoints);

  // https://github.com/ros-navigation/navigation2/blob/main/nav2_msgs/action/FollowWaypoints.action
  FollowWaypoints::Goal goal_msg;

  // TODO: Fill the goal message

  auto send_goal_options = rclcpp_action::Client<FollowWaypoints>::SendGoalOptions();

  // TODO: Set up goal response, feedback, and result callbacks

  // TODO: Asynchronously send the goal
  // this->client_ptr_->...
}

void FollowWaypointsActionClient::goal_response_callback(const GoalHandleFollowWaypoints::SharedPtr& goal_handle) {
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server. Shuting down client.");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void FollowWaypointsActionClient::result_callback(const GoalHandleFollowWaypoints::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      // TODO: Handle missed waypoints
      // Print the index of missing waypoints to log
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      break;
  }

  rclcpp::shutdown();
}

void FollowWaypointsActionClient::publish_waypoint_markers(const Waypoints& waypoints) {
  visualization_msgs::msg::MarkerArray marker_array;

  for (size_t i = 0; i < waypoints.size(); ++i) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = this->now();
    marker.ns = "waypoints";
    marker.id = i;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;

    // Set position
    marker.pose = waypoints[i].pose;

    // Set scale
    marker.scale.x = 1.0;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;

    // Set color (blue)
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Set lifetime
    marker.lifetime = rclcpp::Duration::from_seconds(0);  // 0 means persistent

    marker_array.markers.push_back(marker);
  }

  marker_pub_->publish(marker_array);
}

}  // namespace robot_navigation

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<robot_navigation::FollowWaypointsActionClient>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}