#pragma once

#include "thermalCameraDevice.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "custom_msgs/msg/optris_temperature_matrix.hpp"
#include <sensor_msgs/image_encodings.hpp>

using namespace rclcpp_lifecycle;

class thermalCameraNode : public LifecycleNode
{
    public:
        thermalCameraNode(const std::string & node_name, bool intra_process_comms = false);
        ~thermalCameraNode();

    private:
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const  State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const State & state);

        void publishThermalImage();
        void poolerThermalImage();

        thermalCameraDevice* ThermalCameraDev_ = NULL;

        std::shared_ptr<rclcpp::TimerBase> thermalImagepublisher_timer_;
        std::shared_ptr<rclcpp::TimerBase> thermalFeedPooler_timer_;

        std::shared_ptr<LifecyclePublisher<sensor_msgs::msg::Image>> tempRGB_image_publisher_;
        std::shared_ptr<LifecyclePublisher<custom_msgs::msg::OptrisTemperatureMatrix>> tempfloat_image_publisher_;
};

