#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>  
#include <tf2/LinearMath/Vector3.h>  
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
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
        this->declare_parameter<std::string>("octomap_topic", "octomap_full");  
        this->declare_parameter<double>("tf_timeout_sec", 0.5); 
        this->declare_parameter<double>("volume_threshold", 0.1); 
        this->declare_parameter<double>("max_range", 5.0); 
        this->declare_parameter<double>("camera_fov", 70.0);
        this->declare_parameter<double>("sensor_height_above_ground", 0.055);

        // Retrieve parameters  
        this->get_parameter("map_frame", map_frame_);
        this->get_parameter("tf_timeout_sec", tf_timeout_sec_);
        this->get_parameter("volume_threshold", volume_threshold_);  
        this->get_parameter("octomap_topic", octomap_topic_);  
        this->get_parameter("max_range", max_range_);
        this->get_parameter("camera_fov", camera_fov_);
        this->get_parameter("sensor_height_above_ground", sensor_height_above_ground_);

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
            "OctomapOcclusionsNode initialised with threshold: %.2f m³, FoV (conical): %.1f°",
            volume_threshold_, camera_fov_);
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
    double calculateOccludedVolume(const geometry_msgs::msg::TransformStamped& transform_stamped) const {
        const double fov_rad = camera_fov_ * M_PI / 180.0;  // convert to radians
        const double half_fov = fov_rad / 2.0; 
        const double cone_base_radius = max_range_ * std::tan(half_fov);
        const double cone_volume = (1.0/3.0) * M_PI * cone_base_radius * cone_base_radius * max_range_; // volume = (1/3) × π x r² × h

        double known_volume = 0.0; 

        const octomap::point3d sensor_origin(
            transform_stamped.transform.translation.x,
            transform_stamped.transform.translation.y,
            transform_stamped.transform.translation.z
        );

        // Calculate dynamic ground height  
        const double ground_height = sensor_origin.z() - sensor_height_above_ground_;

        // Extract quaternion from transform (convert geometry_msgs quaternion to tf2::Quaternion)
        tf2::Quaternion q;  
        tf2::fromMsg(transform_stamped.transform.rotation, q);

        // Create forward vector in camera frame (currently Z-axis)  
        tf2::Vector3 forward_vector(0, 0, 1);  

        // Rotate forward vector by quaternion to get the camera's actual orientation in map frame
        tf2::Vector3 transformed_forward = tf2::quatRotate(q, forward_vector); 

        // Type convert back to octomap::point3d  
        const octomap::point3d forward_direction(  
            transformed_forward.x(),   
            transformed_forward.y(),   
            transformed_forward.z()  
        );  
        
        // Bounding box iterator to limit search area to only voxels within max range (using a larger box to contain the cone)
        octomap::point3d bbx_min = sensor_origin - octomap::point3d(max_range_, max_range_, max_range_);  
        octomap::point3d bbx_max = sensor_origin + octomap::point3d(max_range_, max_range_, max_range_);

        // Iterate through existing leaf nodes in the octree 
        for(auto it = octree_->begin_leafs_bbx(bbx_min, bbx_max), end = octree_->end_leafs_bbx(); it != end; ++it) {
            const octomap::point3d voxel_center = it.getCoordinate();   // Voxel's center coordinate

            // Skip voxels under ground  
            if (voxel_center.z() < ground_height) {  
                continue;  
            }  

            const octomap::point3d voxel_direction = voxel_center - sensor_origin; 
            const double distance = voxel_direction.norm();    // Distance from sensor to voxel

            if (distance <= max_range_ && distance > 0) {   // Skip voxels outside max range
                // Check if voxel is within cone angle  
                const double cos_angle = voxel_direction.dot(forward_direction) / distance; // divide by distance to normalise
                const double angle = std::acos(std::clamp(cos_angle, -1.0, 1.0));   // convert the cosine back to an angle in radians, and clamp between [-1, 1]
                
                if (angle <= half_fov) { 
                    // Voxel is within the cone's FoV, add its volume
                    const double voxel_size = it.getSize();  
                    const double voxel_volume = voxel_size * voxel_size * voxel_size;  
                    known_volume += voxel_volume;  
                } 
            } 
        } 
        return cone_volume - known_volume;
    }
        
    // Safe initialisers for cached values (before parameters are loaded)
    std::string map_frame_ {"odom"};
    std::string octomap_topic_ {"/octomap_full"};
    double tf_timeout_sec_ {0.5};
    double volume_threshold_ {0.1};
    double max_range_{5.0};
    double camera_fov_{70.0};
    double sensor_height_above_ground_{0.055};

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
