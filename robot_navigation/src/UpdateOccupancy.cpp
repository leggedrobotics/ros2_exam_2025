#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "nav2_costmap_2d/costmap_layer.hpp"

namespace robot_navigation {

class SimpleObstacleLayer : public nav2_costmap_2d::CostmapLayer {
 public:
  using LaserSubscriber = message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>;
  using LaserFilter = tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>;
  void onInitialize() override {
    auto node = node_.lock();
    if (!node) {
      throw std::runtime_error("Failed to lock node");
    }

    // Declare parameters
    declareParameter("enabled", rclcpp::ParameterValue(true));

    node->get_parameter(name_ + "." + "enabled", enabled_);

    // Set default values of the costmap
    setDefaultValue(nav2_costmap_2d::FREE_SPACE);
    nav2_costmap_2d::CostmapLayer::matchSize();

    auto sub_opt = rclcpp::SubscriptionOptions();
    sub_opt.callback_group = callback_group_;

    auto custom_qos_profile = rclcpp::SensorDataQoS().keep_last(50);
    laser_sub_ =
        std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan, rclcpp_lifecycle::LifecycleNode>>(
            node, "/scan", custom_qos_profile.get_rmw_qos_profile(), sub_opt);
    laser_sub_->unsubscribe();

    laser_filter_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
        *laser_sub_, *tf_, "map", 50, node->get_node_logging_interface(), node->get_node_clock_interface(),
        tf2::durationFromSec(0.3));
    laser_filter_->setTolerance(rclcpp::Duration::from_seconds(0.05));
    laser_filter_->registerCallback(std::bind(&SimpleObstacleLayer::laserScanCallback, this, std::placeholders::_1));

    current_ = true;

    RCLCPP_INFO(rclcpp::get_logger("SimpleObstacleLayer"), "SimpleObstacleLayer initialized");
  }

  void activate() override {
    if (!enabled_) {
      return;
    }
    RCLCPP_INFO(rclcpp::get_logger("SimpleObstacleLayer"), "SimpleObstacleLayer activated");
    laser_filter_->clear();
    laser_sub_->subscribe();
  }

  void deactivate() override { laser_sub_->unsubscribe(); }

  void updateBounds(double robot_x, double robot_y, double robot_yaw, double *min_x, double *min_y, double *max_x,
                    double *max_y) override {
    if (!enabled_ || latest_cloud_.data.empty()) {
      return;
    }
    (void)robot_x;
    (void)robot_y;
    (void)robot_yaw;

    // Process each point in cloud
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(latest_cloud_, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(latest_cloud_, "y");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y) {
      double px = *iter_x, py = *iter_y;

      unsigned int mx, my;
      if (!worldToMap(px, py, mx, my)) {
        RCLCPP_WARN_THROTTLE(rclcpp::get_logger("SimpleObstacleLayer"), *clock_, 5000,
                             "Point outside of costmap bounds, (%f, %f)", px, py);
        continue;
      }

      unsigned int index = getIndex(mx, my);
      costmap_[index] = nav2_costmap_2d::LETHAL_OBSTACLE;
      touch(px, py, min_x, min_y, max_x, max_y);
    }
  }

  void updateCosts(nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j, int max_i, int max_j) override {
    if (!enabled_) {
      return;
    }

    updateWithMax(master_grid, min_i, min_j, max_i, max_j);
  }

  void reset() override { resetMaps(); }

  bool isClearable() override { return true; }

 private:
  void laserScanCallback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan) {
    // Hint: we uses tf2_ros::MessageFilter for this callback. The message filter will call this callback only if both
    // messages and the transformation from the msg.header.frame_id to the target frame at msg.header.stamp are
    // available. You can assume that the transformation is available in this callback.
    if (!enabled_) {
      return;
    }

    try {
      // TODO: Look up transform from laser frame to map frame
      // You are given a pointer to the tf2_ros::Buffer object `tf_` which can be used to look up transforms
      // Hint: Use `tf_->lookupTransform` to look up transform
      geometry_msgs::msg::TransformStamped transform_stamped;
      // transform_stamped = ...

      // Convert laser scan to point cloud in laser frame
      sensor_msgs::msg::PointCloud2 cloud;
      laser_geometry::LaserProjection projector;
      projector.projectLaser(*scan, cloud);

      // TODO: Transform point cloud to map frame. Write the transformed point cloud to latest_cloud_
      // Hint: Use `tf2::doTransform` to transform the point cloud

      // The data points stored in latest_cloud_ are in the map frame and will be write to the cost map in the
      // `updateBounds` and `updateCosts` functions. We have implemented these functions for you.

    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(rclcpp::get_logger("SimpleObstacleLayer"), "Failed to transform laser scan: %s", ex.what());
      return;
    } catch (const std::exception &e) {
      RCLCPP_WARN(rclcpp::get_logger("SimpleObstacleLayer"), "Error processing laser scan: %s", e.what());
      return;
    }
  }

  sensor_msgs::msg::PointCloud2 latest_cloud_;
  std::shared_ptr<LaserSubscriber> laser_sub_;
  std::shared_ptr<LaserFilter> laser_filter_;
};

}  // namespace robot_navigation

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(robot_navigation::SimpleObstacleLayer, nav2_costmap_2d::Layer)