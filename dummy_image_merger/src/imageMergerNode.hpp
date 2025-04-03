#pragma once

#include "imageMergerNode.hpp"
#include "imageUtils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "std_msgs/msg/float32_multi_array.hpp"
#include "custom_msgs/msg/fusion_matrix.hpp"

// Nav2 mgs
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

#include <stack>

using namespace rclcpp_lifecycle;

struct rover_pose{
  // Position
  float x_pos;
  float y_pos;

  // IMU orientation
  float x;
  float y;
  float z;
  float w;
  float local_heading;
};

struct rover_gps{
  float lat;
  float lon;
};

class imageMergerNode : public LifecycleNode
{
    public:
        imageMergerNode(const std::string & node_name, bool intra_process_comms = false);
        ~imageMergerNode();

    private:
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const  State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const State & state);
               
        // Callbacks msgs
        void publishMultiArray();
        void subRoverPose(geometry_msgs::msg::PoseWithCovarianceStamped msg);
        void subRoverGPS(sensor_msgs::msg::NavSatFix msg);

        std::shared_ptr<rclcpp::TimerBase> publishMultiArray_timer_;
        std::shared_ptr<LifecyclePublisher<custom_msgs::msg::FusionMatrix>> image_publisher_;
        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::NavSatFix>> gps_sub_;
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>> rover_pose_sub_;

        // Nav2 Stacks
        std::stack<rover_pose> q_rover_pose;
        std::stack<rover_gps> q_rover_gps;
        pthread_mutex_t rover_pose_mutex_ = PTHREAD_MUTEX_INITIALIZER;
        pthread_mutex_t rover_gps_mutex_  = PTHREAD_MUTEX_INITIALIZER;

        int multi_seq_ = 0;
        int output_multi_img_height_   = 720;
        int output_multi_img_width_    = 1280;
        int output_multi_img_channels_ = 5;
        int input_thermal_img_width_   = 640;
        int input_thermal_img_height_  = 480;

        void clearqroverPose(std::stack<rover_pose> &q);
        void clearqroverGPS(std::stack<rover_gps> &q);
};