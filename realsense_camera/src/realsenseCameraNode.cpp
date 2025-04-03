#include "realsenseCameraNode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "yaml-cpp/yaml.h"
#include <cmath>

realsenseCameraNode::realsenseCameraNode(const std::string & node_name, bool intra_process_comms)
  : LifecycleNode(node_name,rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  this->declare_parameter("config_file", "");
}

realsenseCameraNode::~realsenseCameraNode()
{
}

node_interfaces::LifecycleNodeInterface::CallbackReturn realsenseCameraNode::on_configure(const rclcpp_lifecycle::State & state)
{
    
  bool configure_exit_state = false;
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On configure state");
  
  std::string config_file = this->get_parameter("config_file").as_string();
  YAML::Node config       = YAML::LoadFile(config_file);
  if (!config["config_output_imgs_publish_hz_rate"] || !config["config_output_topic_RGB"] || !config["config_output_topic_Depth"]) 
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  
  // Read configuration
  float config_output_imgs_publish_hz_rate = config["config_output_imgs_publish_hz_rate"].as<float>();
  std::string config_output_topic_RGB      = config["config_output_topic_RGB"].as<std::string>();
  std::string config_output_topic_Depth    = config["config_output_topic_Depth"].as<std::string>();
  int publish_rate                         = round((1/config_output_imgs_publish_hz_rate) * 1000);

  RealsenseCameraDev_ = new realsenseCameraDevice();
  RealsenseCameraDev_->initRealsense();

  // Creating publisher and subscriber
  pubRGBImage_    = this->create_publisher<sensor_msgs::msg::Image>(config_output_topic_RGB, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  pubDepthImage_  = this->create_publisher<custom_msgs::msg::RealsenseDepthMatrix>(config_output_topic_Depth, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  publishRealsenseImages_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&realsenseCameraNode::publishRealsenseImages, this));

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void realsenseCameraNode::publishRealsenseImages()
{
  float pihalf     = 1.57;
  auto orientation =  RealsenseCameraDev_->getTheta().z + pihalf;

  dataFrames images_realsense = RealsenseCameraDev_->getImagesData();
  auto color_image_realsense  = images_realsense.color_data;
  auto depth_image_realsense  = images_realsense.depth_data;
  auto depth_scale_realsense  = images_realsense.depth_scale;
  auto timestamp_realsense    = images_realsense.timestamp;

  /* PUBLISH RGB */
  int rgb_image_size          = REALSENSE_XRES * REALSENSE_YRES * 3;
  auto rgb_rs_msg             = std::make_unique<sensor_msgs::msg::Image>();

  rgb_rs_msg->header.frame_id = "Realsense_RGB";
  rgb_rs_msg->width           = REALSENSE_XRES;
  rgb_rs_msg->height          = REALSENSE_YRES;
  rgb_rs_msg->step            = REALSENSE_XRES * 3;
  rgb_rs_msg->encoding        = sensor_msgs::image_encodings::RGB8; 
  rgb_rs_msg->is_bigendian    = 0;
  rgb_rs_msg->header.stamp    = rclcpp::Time(timestamp_realsense);
  rgb_rs_msg->data.resize(rgb_image_size);

  std::memcpy(&rgb_rs_msg->data[0], color_image_realsense, rgb_image_size);
  // RCLCPP_INFO(this->get_logger(), "Publishing RGB Image");
  pubRGBImage_->publish(std::move(rgb_rs_msg));

  /* PUBLISH DEPTH */
  int depth_image_size          = REALSENSE_XRES * REALSENSE_YRES * 2;
  auto depth_rs_msg             = std::make_unique<custom_msgs::msg::RealsenseDepthMatrix>();
  depth_rs_msg->header.frame_id = "Realsense_Depth";
  depth_rs_msg->width           = REALSENSE_XRES;
  depth_rs_msg->height          = REALSENSE_YRES;
  depth_rs_msg->depth_scale     = depth_scale_realsense;
  depth_rs_msg->header.stamp    = rclcpp::Time(timestamp_realsense);
  depth_rs_msg->data.resize(depth_image_size);

  std::memcpy(&depth_rs_msg->data[0], depth_image_realsense, depth_image_size);
  // RCLCPP_INFO(this->get_logger(), "Publishing Depth Image");
  pubDepthImage_->publish(std::move(depth_rs_msg));

}


node_interfaces::LifecycleNodeInterface::CallbackReturn realsenseCameraNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On activate state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn realsenseCameraNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On deactivate state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn realsenseCameraNode::on_cleanup(const rclcpp_lifecycle::State & state)
{

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On cleanup state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn realsenseCameraNode::on_shutdown(const rclcpp_lifecycle::State & state)
{

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On on_shutdown state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn realsenseCameraNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On error state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
}


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<realsenseCameraNode> realsense_camera_node = std::make_shared<realsenseCameraNode>("realsense_camera_node");
  exe.add_node(realsense_camera_node->get_node_base_interface());

  exe.spin();
  rclcpp::shutdown();
}