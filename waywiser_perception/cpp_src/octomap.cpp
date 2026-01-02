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
        // Initialise OcTreeStamped with 0.1m resolution (voxel size, deafault is 0.05m)
        octree_ = std::make_unique<octomap::OcTreeStamped>(0.05);  
          
        // Subscribe to PointCloud2 published by the OAK-D driver
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(  
            "/sensors/camera/color/points", 10,  
            std::bind(&OctomapNode::pointcloudCallback, this, std::placeholders::_1));  
          
        // Publisher for RViz visualization  
        octomap_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>(
            "/octomap_full", 1);    // Queue size 1 since we only wish to keep the latest map
          
        RCLCPP_INFO(this->get_logger(), "Octomap node initialised.");  
    }  
  
private:  
    void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {   // Runs every time a PointCloud2 arrives
        try { 
            // Transform PointCloud2 to target frame
            geometry_msgs::msg::TransformStamped transform_stamped;  
            transform_stamped = tf_buffer_.lookupTransform(  
                "odom", msg->header.frame_id, msg->header.stamp);  
            
            sensor_msgs::msg::PointCloud2 transformed_cloud;  
            tf2::doTransform(*msg, transformed_cloud, transform_stamped);  
            
            // Convert PointCloud2 to OctoMap Pointcloud 
            octomap::Pointcloud cloud;  
              
            // Find the XYZ field offsets inside the PointCloud2 layout  
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

            // Parse raw buffer (point cloud data) to extract XYZ per point
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

            // Compute sensor origin in camera_link frame  
            octomap::point3d sensor_origin(  
                transform_stamped.transform.translation.x,  
                transform_stamped.transform.translation.y,  
                transform_stamped.transform.translation.z);  

            // Insert point cloud into OcTreeStamped  
            octree_->insertPointCloud(cloud, sensor_origin);  
              
            // Degrate outdated nodes for dynamic updates  
            octree_->degradeOutdatedNodes(1); // timeout in seconds  
            
            publishOctomap();  
              
        } catch (const tf2::TransformException& ex) {  
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());  
        }  
    }  
      
    void publishOctomap() {  
        octomap_msgs::msg::Octomap octomap_msg; 
        octomap_msg.header.frame_id = "odom";    // Declare which TF frame the map is expressed in
        octomap_msg.header.stamp = this->now();  
          
        if (octomap_msgs::fullMapToMsg(*octree_, octomap_msg)) {    // Serialise tree into the octomap_msg
            octomap_pub_->publish(octomap_msg);  
        }  
    }  
      
    std::unique_ptr<octomap::OcTreeStamped> octree_;    // OcTree is solely owned by this node via unique_ptr
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;  
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr octomap_pub_;  
    tf2_ros::Buffer tf_buffer_;  
    tf2_ros::TransformListener tf_listener_;  
};  
  
int main(int argc, char * argv[]) {  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<OctomapNode>());  
    rclcpp::shutdown();  
    return 0;  
}
