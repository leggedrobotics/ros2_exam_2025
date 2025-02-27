#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/pose.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

// TODO: Include necessary headers for used messages

class DummyLocalizationNode : public rclcpp::Node {
 public:
  DummyLocalizationNode() : Node("map_to_base_link_tf_broadcaster") {
    if (this->get_parameter("use_sim_time").as_bool()) {
      RCLCPP_WARN(this->get_logger(), "Using sim time");
    } else {
      RCLCPP_INFO(this->get_logger(), "Using real time");
    }
    RCLCPP_INFO(this->get_logger(), "Starting Dummy Localization Node");
    // TODO: Initialize the TransformBroadcaster
    // tf_broadcaster_ = ...

    // TODO: Create a subscription to the /robot_pose topic
    // pose_subscription_ = ...
  }

 private:
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscription_;

  void pose_callback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    RCLCPP_INFO_ONCE(this->get_logger(), "Received first pose message");
    // TODO: Publish the transform from odom to base_link
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyLocalizationNode>());
  rclcpp::shutdown();
  return 0;
}