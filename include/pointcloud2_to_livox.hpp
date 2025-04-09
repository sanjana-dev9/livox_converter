#ifndef POINTCLOUD2_TO_LIVOX_HPP
#define POINTCLOUD2_TO_LIVOX_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "livox_interfaces/msg/custom_msg.hpp"
#include "std_msgs/msg/header.hpp"

class PointCloud2ToLivox : public rclcpp::Node
{
public:
    PointCloud2ToLivox();

private:
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<livox_interfaces::msg::CustomMsg>::SharedPtr publisher_;
};

#endif  // POINTCLOUD2_TO_LIVOX_HPP