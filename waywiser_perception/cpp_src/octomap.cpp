#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/point_cloud2.hpp>  
#include <geometry_msgs/msg/transform_stamped.hpp>  
#include <tf2_ros/transform_listener.hpp>  
#include <tf2_ros/buffer.hpp>  
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>  
  
#include <octomap/octomap.h>  
#include <octomap/OcTreeStamped.h>  
#include <octomap_msgs/msg/octomap.hpp>  
#include <octomap_msgs/conversions.h>  
  
class OctomapNode : public rclcpp::Node {  
public:  
    OctomapNode() : Node("octomap_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {  
        // Initialize OcTreeStamped with 0.1m resolution  
        octree_ = std::make_unique<octomap::OcTreeStamped>(0.1);  
          
        // Subscribe to PointCloud2 from OAK-D  
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(  
            "/camera/depth/points", 10,  
            std::bind(&OctomapNode::pointcloudCallback, this, std::placeholders::_1));  
          
        // Publisher for RViz visualization  
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(  
            "/octomap_full", 1);  
          
        RCLCPP_INFO(this->get_logger(), "Octomap node initialized");  
    }  
  
private:  
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {  
        try {  
            // Transform point cloud to base_link frame  
            geometry_msgs::msg::TransformStamped transform_stamped;  
            transform_stamped = tf_buffer_.lookupTransform(  
                "base_link", msg->header.frame_id, msg->header.stamp);  
              
            sensor_msgs::msg::PointCloud2 transformed_cloud;  
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);  
              
            // Convert to octomap point cloud  
            octomap::Pointcloud cloud;  
              
            // Find x, y, z field offsets  
            int x_offset = -1, y_offset = -1, z_offset = -1;  
            for (const auto& field : transformed_cloud.fields) {  
                if (field.name == "x") x_offset = field.offset;  
                else if (field.name == "y") y_offset = field.offset;  
                else if (field.name == "z") z_offset = field.offset;  
            }  
              
            if (x_offset == -1 || y_offset == -1 || z_offset == -1) {  
                RCLCPP_ERROR(this->get_logger(), "Point cloud missing xyz fields");  
                return;  
            }  
              
            // Parse point cloud data  
            const uint8_t* data_ptr = transformed_cloud.data.data();  
            size_t point_count = transformed_cloud.width * transformed_cloud.height;  
              
            for (size_t i = 0; i < point_count; ++i) {  
                const uint8_t* point_ptr = data_ptr + (i * transformed_cloud.point_step);  
                  
                // Extract XYZ as float32  
                float x = *reinterpret_cast<const float*>(point_ptr + x_offset);  
                float y = *reinterpret_cast<const float*>(point_ptr + y_offset);  
                float z = *reinterpret_cast<const float*>(point_ptr + z_offset);  
                  
                // Skip invalid points (NaN or infinite)  
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) {  
                    continue;  
                }  
                  
                cloud.push_back(x, y, z);  
            }  
              
            // Get sensor origin in base_link frame  
            octomap::point3d sensor_origin(  
                transform_stamped.transform.translation.x,  
                transform_stamped.transform.translation.y,  
                transform_stamped.transform.translation.z);  
              
            // Insert point cloud into OcTreeStamped  
            octree_->insertPointCloud(cloud, sensor_origin);  
              
            // Optional: Degrate outdated nodes for dynamic handling  
            octree_->degradeOutdatedNodes(30); // 30 second timeout  
              
            // Publish for RViz  
            publishOctomap();  
              
        } catch (const tf2::TransformException& ex) {  
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());  
        }  
    }  
      
    void publishOctomap() {  
        octomap_msgs::msg::Octomap octomap_msg;  
        octomap_msg.header.frame_id = "base_link";  
        octomap_msg.header.stamp = this->now();  
          
        if (octomap_msgs::fullMapToMsg(*octree_, octomap_msg)) {  
            octomap_pub_->publish(octomap_msg);  
        }  
    }  
      
    std::unique_ptr<octomap::OcTreeStamped> octree_;  
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;  
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;  
    tf2_ros::Buffer tf_buffer_;  
    tf2_ros::TransformListener tf_listener_;  
};  
  
int main(int argc, char** argv) {  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<OctomapNode>());  
    rclcpp::shutdown();  
    return 0;  
}