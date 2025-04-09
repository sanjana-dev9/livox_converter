#include "pointcloud2_to_livox.hpp"

using namespace std::chrono_literals;

PointCloud2ToLivox::PointCloud2ToLivox() : Node("pointcloud2_to_livox")
{
    publisher_ = this->create_publisher<livox_interfaces::msg::CustomMsg>("livox_pointcloud", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "pointcloud2_input", 10, std::bind(&PointCloud2ToLivox::callback, this, std::placeholders::_1));
}

void PointCloud2ToLivox::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    livox_interfaces::msg::CustomMsg output;
    output.header = msg->header;
    output.timebase = output.header.stamp.sec * 1000000000ULL + output.header.stamp.nanosec; // Default timebase from header
    output.point_num = msg->width * msg->height;
    output.lidar_id = 0; // Default lidar_id (you might want to make this configurable)
    
    // Initialize points array
    output.points.resize(output.point_num);
    
    // Find field offsets
    int x_offset = -1;
    int y_offset = -1;
    int z_offset = -1;
    int intensity_offset = -1;
    int tag_offset = -1;
    int line_offset = -1;
    
    for (const auto& field : msg->fields) {
        if (field.name == "x") x_offset = field.offset;
        else if (field.name == "y") y_offset = field.offset;
        else if (field.name == "z") z_offset = field.offset;
        else if (field.name == "intensity") intensity_offset = field.offset;
        else if (field.name == "tag") tag_offset = field.offset;
        else if (field.name == "line") line_offset = field.offset;
    }
    
    // Check if essential fields are present
    if (x_offset == -1 || y_offset == -1 || z_offset == -1) {
        RCLCPP_ERROR(this->get_logger(), "PointCloud2 missing x, y, or z fields!");
        return;
    }
    
    // Extract points
    const uint8_t* data_ptr = msg->data.data();
    for (uint32_t i = 0; i < output.point_num; ++i) {
        // Set offset_time to 0 as we don't have this information in standard PointCloud2
        output.points[i].offset_time = 0;
        
        // Extract coordinates (these are required)
        output.points[i].x = *reinterpret_cast<const float*>(data_ptr + x_offset);
        output.points[i].y = *reinterpret_cast<const float*>(data_ptr + y_offset);
        output.points[i].z = *reinterpret_cast<const float*>(data_ptr + z_offset);
        
        // Extract intensity if available
        if (intensity_offset != -1) {
            output.points[i].reflectivity = static_cast<uint8_t>(*reinterpret_cast<const float*>(data_ptr + intensity_offset));
        } else {
            output.points[i].reflectivity = 0;
        }
        
        // Extract tag if available
        if (tag_offset != -1) {
            output.points[i].tag = *(data_ptr + tag_offset);
        } else {
            output.points[i].tag = 0;
        }
        
        // Extract line if available
        if (line_offset != -1) {
            output.points[i].line = *(data_ptr + line_offset);
        } else {
            output.points[i].line = 0;
        }
        
        // Move to next point
        data_ptr += msg->point_step;
    }
    
    publisher_->publish(output);
}