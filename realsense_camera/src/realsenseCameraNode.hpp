#pragma once

#include "realsenseCameraDevice.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "custom_msgs/msg/realsense_depth_matrix.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

using namespace rclcpp_lifecycle;

class realsenseCameraNode : public LifecycleNode
{
    public:
        realsenseCameraNode(const std::string & node_name, bool intra_process_comms = false);
        ~realsenseCameraNode();

    private:
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const  State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const State & state);
        
        void publishRealsenseImages();

        realsenseCameraDevice* RealsenseCameraDev_ = NULL;

        size_t count_;

        std::shared_ptr<rclcpp::TimerBase> publishRealsenseImages_timer_;
        std::shared_ptr<LifecyclePublisher<sensor_msgs::msg::Image>> pubRGBImage_;
        std::shared_ptr<LifecyclePublisher<custom_msgs::msg::RealsenseDepthMatrix>> pubDepthImage_;
};