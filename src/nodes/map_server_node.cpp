/**
 * @file map_server_node.cpp
 * @brief CloudGuessr Map Server - 맵을 PointCloud2로 publish
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>

#include "cloudguessr/backend/io.hpp"
#include "cloudguessr/backend/preprocess.hpp"

class MapServerNode : public rclcpp::Node
{
public:
  MapServerNode()
  : Node("cloudguessr_map_server")
  {
    // Parameters
    this->declare_parameter("map_file", "");
    this->declare_parameter("map_frame", "map");
    this->declare_parameter("voxel_size", 0.0);
    this->declare_parameter("publish_rate", 1.0);

    map_file_ = this->get_parameter("map_file").as_string();
    map_frame_ = this->get_parameter("map_frame").as_string();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    double publish_rate = this->get_parameter("publish_rate").as_double();

    if (map_file_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "map_file parameter is required");
      return;
    }

    // Load map
    RCLCPP_INFO(this->get_logger(), "Loading map: %s", map_file_.c_str());
    map_cloud_ = cloudguessr::io::loadPointCloud(map_file_);

    if (!map_cloud_ || map_cloud_->empty()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to load map");
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Loaded %zu points", map_cloud_->size());

    // Downsample if requested
    if (voxel_size_ > 0.0) {
      RCLCPP_INFO(this->get_logger(), "Downsampling with voxel size: %.2f", voxel_size_);
      map_cloud_ = cloudguessr::preprocess::voxelDownsample(map_cloud_, voxel_size_);
      RCLCPP_INFO(this->get_logger(), "After downsampling: %zu points", map_cloud_->size());
    }

    // Convert to ROS message
    pcl::toROSMsg(*map_cloud_, map_msg_);
    map_msg_.header.frame_id = map_frame_;

    // Publisher (latched via transient local QoS)
    auto qos = rclcpp::QoS(1).transient_local().reliable();
    map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cloudguessr/map", qos);

    // Publish periodically
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MapServerNode::publishMap, this));

    RCLCPP_INFO(this->get_logger(), "Map server ready");
  }

private:
  void publishMap()
  {
    map_msg_.header.stamp = this->now();
    map_pub_->publish(map_msg_);
  }

  std::string map_file_;
  std::string map_frame_;
  double voxel_size_;

  cloudguessr::PointCloudPtr map_cloud_;
  sensor_msgs::msg::PointCloud2 map_msg_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapServerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
