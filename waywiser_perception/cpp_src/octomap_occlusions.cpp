#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <mutex>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>  

#include "waywiser_twist_safety/msg/emergency_stop_state.hpp"  
#include "waywiser/waywiser_utils.hpp"  

class OctomapOcclusionsNode : public rclcpp::Node {  
public:  
    OctomapOcclusionsNode() : Node("octomap_occlusions_node"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) 
    { 
        // Declare parameters with default values
        this->declare_parameter<std::string>("map_frame", "odom"); 
        this->declare_parameter<double>("tf_timeout_sec", 0.5); 
        this->declare_parameter<double>("volume_threshold", 0.1); 
        this->declare_parameter<std::string>("octomap_topic", "octomap_full"); 
        this->declare_parameter<double>("max_range", 5.0); 

        // Retrieve parameters  
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
        this->get_parameter("volume_threshold", volume_threshold_);  
        this->get_parameter("octomap_topic", octomap_topic_);  
        this->get_parameter("max_range", max_range_);

        // Subscribe to OctoMap
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_topic_,
            rclcpp::QoS(1).transient_local().reliable(),    // Keep only the latest message in the buffer
            std::bind(&OctomapOcclusionsNode::octomapCallback, this, std::placeholders::_1)
        );
          
         // Initialise emergency stop publisher  
        emergency_stop_publisher_ = this->create_publisher<waywiser_twist_safety::msg::EmergencyStopState>( 
            "/emergency_stop/target_state",   
            QOS_PROFILES::RELIABLE_TRANSIENT_LOCAL_QOS
        ); 
      
        // Initialise emergency stop message  
        emergency_stop_msg_.sender_id = "octomap_occlusions";  
        emergency_stop_msg_.state = waywiser_twist_safety::msg::EmergencyStopState::ACTIVE;  

        RCLCPP_INFO(this->get_logger(),   
            "OctomapOcclusionsNode initialised with threshold: %.2f m³",
            volume_threshold_);
    }  
  
private: 

    void octomapCallback(const octomap_msgs::msg::Octomap::SharedPtr msg) {  
        geometry_msgs::msg::TransformStamped transform_stamped;
        double unknown_volume = 0.0;    // Occlusions

        try {
            transform_stamped = tf_buffer_.lookupTransform(
                map_frame_, msg->header.frame_id, msg->header.stamp,
                rclcpp::Duration::from_seconds(tf_timeout_sec_));
            
            {
                std::lock_guard<std::mutex> lock(map_mutex_);  

                auto* abstract_tree = octomap_msgs::msgToMap(*msg); 
                auto* octree_stamped = dynamic_cast<octomap::OcTreeStamped*>(abstract_tree);  
                
                if (!octree_stamped) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to convert OctoMap message (not an OcTreeStamped)");
                    delete abstract_tree;  // Clean up allocated memory
                    return;
                }  
                
                octree_.reset(octree_stamped);

                if (octree_->size() == 0) {
                    RCLCPP_INFO(this->get_logger(), "OcTree is empty"); 
                    return;
                }

                unknown_volume = calculateOccludedVolume(transform_stamped);  
            }   // lock released

            RCLCPP_INFO(this->get_logger(),   
                "Occluded volume: %.4f m³ (threshold: %.2f m³)",   
                unknown_volume, volume_threshold_);  

            if(unknown_volume > volume_threshold_) {
                emergency_stop_msg_.stamp = this->get_clock()->now();  
                emergency_stop_msg_.reason = "Unknown volume " + std::to_string(unknown_volume) +   
                                            " m³ exceeds threshold " + std::to_string(volume_threshold_) + " m³";  
                emergency_stop_publisher_->publish(emergency_stop_msg_);  
                
                RCLCPP_INFO(this->get_logger(),     
                    "Emergency stop triggered - unknown volume: %.4f m³ exceeds threshold: %.2f m³",     
                    unknown_volume, volume_threshold_); 
            }

        } catch (const tf2::TransformException & ex) {
            RCLCPP_WARN(this->get_logger(), "Transform error: %s", ex.what());
        }
    }

    // Calculate the volume of voxels within max range not seen by the sensor 
    double calculateOccludedVolume(const geometry_msgs::msg::TransformStamped transform_stamped) const {
        const double max_range_cubed = max_range_ * max_range_ * max_range_;
        const double sphere_volume = (4.0/3.0) * M_PI * max_range_cubed; 
        double known_volume = 0.0; 

        const octomap::point3d sensor_origin(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z
        );

        // Bounding box iterator to limit search area to only voxels within max range (bounding box is cubic however so will include corners outside the max range sphere)
        octomap::point3d bbx_min = sensor_origin - octomap::point3d(max_range_, max_range_, max_range_);  
        octomap::point3d bbx_max = sensor_origin + octomap::point3d(max_range_, max_range_, max_range_);  

        // Iterate through existing leaf nodes in the octree 
        for(auto it = octree_->begin_leafs_bbx(bbx_min, bbx_max), end = octree_->end_leafs_bbx(); it != end; ++it) {
      
            const octomap::point3d voxel_center = it.getCoordinate();   // Voxel's center coordinate
            const double distance = (voxel_center - sensor_origin).norm();    // Distance from sensor to voxel (Euclidean length)

            if (distance <= max_range_) {   // Skip voxels outside max range
                const double voxel_size = it.getSize(); 
                const double voxel_volume = voxel_size * voxel_size * voxel_size; 
                
                known_volume += voxel_volume; 
            }  
        } 
        return sphere_volume - known_volume;
    }
        
    // Safe initialisers for cached values (before parameters are loaded)
    std::string map_frame_ {"odom"};
    double tf_timeout_sec_ {0.5};
    double volume_threshold_ {0.1};
    std::string octomap_topic_ {"/octomap_full"};
    double max_range_{5.0};

    // TF2 components
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // ROS components  
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

    // Octree storage  
    std::unique_ptr<octomap::OcTreeStamped> octree_;  
    std::mutex map_mutex_;  

    // Emergency stop
    rclcpp::Publisher<waywiser_twist_safety::msg::EmergencyStopState>::SharedPtr emergency_stop_publisher_;  
    waywiser_twist_safety::msg::EmergencyStopState emergency_stop_msg_;
};

int main(int argc, char * argv[]) {  
    rclcpp::init(argc, argv);  
    auto node = std::make_shared<OctomapOcclusionsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();  
    return 0;  
}
