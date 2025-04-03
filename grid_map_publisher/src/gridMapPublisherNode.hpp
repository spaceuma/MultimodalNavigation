#pragma once

#include "gridMapPublisherNode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "custom_msgs/msg/segment_dem_matrix.hpp"
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/msg/grid_map.hpp>
#include <cmath>
#include <memory>
#include <utility>

#include <stack>

using namespace rclcpp_lifecycle;

class gridMapPublisherNode : public LifecycleNode
{
    public:
        gridMapPublisherNode(const std::string & node_name, bool intra_process_comms = false);
        ~gridMapPublisherNode();

    private:
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const  State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const State & state);
        node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const State & state);

        // Callbacks msgs
        void sub_seg(custom_msgs::msg::SegmentDemMatrix msg);

        // Defining publishers and subs
        std::shared_ptr<rclcpp::TimerBase> publishGridMap_timer_;
        std::shared_ptr<LifecyclePublisher<grid_map_msgs::msg::GridMap>> grid_map_publisher_;
        std::shared_ptr<rclcpp::Subscription<custom_msgs::msg::SegmentDemMatrix>> seg_sub_;
        bool debug_mode_ = false;
};