#include "thermalCameraNode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "yaml-cpp/yaml.h"
#include "Image.h"
#include <cmath>


thermalCameraNode::thermalCameraNode(const std::string & node_name, bool intra_process_comms)
  : LifecycleNode(node_name,rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  this->declare_parameter("config_file", "");
}

thermalCameraNode::~thermalCameraNode()
{
}

node_interfaces::LifecycleNodeInterface::CallbackReturn thermalCameraNode::on_configure(const rclcpp_lifecycle::State & state)
{
    
  bool configure_exit_state = false;
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On configure state");
  
  std::string config_file = this->get_parameter("config_file").as_string();
  YAML::Node config = YAML::LoadFile(config_file);

  if (!config["camera_serial"] || !config["publish_rate"] || !config["pooling_rate"] || !config["config_output_topic_thermal"] || !config["config_output_topic_float"]) return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  // Read configuration
  int config_serial    = config["camera_serial"].as<int>();
  std::string config_output_topic_RGB    = config["config_output_topic_thermal"].as<std::string>();
  std::string config_output_topic_float  = config["config_output_topic_float"].as<std::string>();
  float config_publish = config["publish_rate"].as<float>();
  float config_pooling = config["pooling_rate"].as<float>();
  int publish_rate     = floor((1/config_publish) * 1000);
  int pooling_rate     = floor((1/config_pooling) * 1000);

  // Get config folder path
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("thermal_camera");
  std::string camera_config_xml = package_share_directory + "/config/camera_conf/" + std::to_string(config_serial).c_str() + ".xml";
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Config file located at %s", camera_config_xml.c_str());

  ThermalCameraDev_ = new thermalCameraDevice();
  configure_exit_state = ThermalCameraDev_->configureCamera(camera_config_xml.c_str());

  if(configure_exit_state)
  {
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Thermal Camera configured");
  }
  else
  {
    RCLCPP_ERROR(rclcpp::get_logger(__PRETTY_FUNCTION__), "Thermal Camera not configured properly");
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  tempRGB_image_publisher_   = this->create_publisher<sensor_msgs::msg::Image>(config_output_topic_RGB, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  tempfloat_image_publisher_ = this->create_publisher<custom_msgs::msg::OptrisTemperatureMatrix>(config_output_topic_float, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  
  thermalImagepublisher_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&thermalCameraNode::publishThermalImage, this));
  thermalFeedPooler_timer_ = this->create_wall_timer(std::chrono::milliseconds(pooling_rate), std::bind(&thermalCameraNode::poolerThermalImage, this));

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void thermalCameraNode::poolerThermalImage()
{
  evo::IRDeviceError retval;

  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "poolerThermalImage entered");
  retval = ThermalCameraDev_->processCameraData();
  RCLCPP_DEBUG(rclcpp::get_logger(__PRETTY_FUNCTION__), "poolerThermalImage exited with retval: %d", retval);
}


void thermalCameraNode::publishThermalImage()
{
  long long timestamp = -1;
  bool bad_rgb, bad_float = false;

  if (!tempRGB_image_publisher_->is_activated() || !tempfloat_image_publisher_->is_activated()) 
  {
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Lifecycle publisher is currently inactive. Messages are not published.");
  } 
  else 
  {
    // Creating RGB message
    int rgb_image_size = ThermalCameraDev_->thermal_height_ *ThermalCameraDev_->thermal_width_ * 3;
    unsigned char* rgb_capture_buffer   = new unsigned char[rgb_image_size];
    timestamp = ThermalCameraDev_->obtainRGBThermalImage(rgb_capture_buffer);
    if(timestamp != -1)
    {
      bad_rgb = false;

      auto rgb_thermal_msg = std::make_unique<sensor_msgs::msg::Image>();
      rgb_thermal_msg->header.frame_id = "camera_link_RGBthermal";
      rgb_thermal_msg->width  = ThermalCameraDev_->thermal_width_;
      rgb_thermal_msg->height = ThermalCameraDev_->thermal_height_;
      rgb_thermal_msg->step   = ThermalCameraDev_->thermal_width_ * 3;
      rgb_thermal_msg->encoding = sensor_msgs::image_encodings::RGB8;
      rgb_thermal_msg->is_bigendian = 0;
      rgb_thermal_msg->header.stamp = rclcpp::Time(timestamp);
      rgb_thermal_msg->data.resize(rgb_image_size);
      std::memcpy(&rgb_thermal_msg->data[0], rgb_capture_buffer, rgb_image_size);

      // Sending msg
      tempRGB_image_publisher_->publish(std::move(rgb_thermal_msg));
    }
    else
    {
      bad_rgb = true;
    }

    
    // Creating float message
    int float_thermal_size = ThermalCameraDev_->thermal_height_ * ThermalCameraDev_->thermal_width_ * 4;
    float* float_capture_buffer   = new float[float_thermal_size];
    timestamp = ThermalCameraDev_->obtainFloatThermalImage(float_capture_buffer);
    if(timestamp != -1)
    {
      bad_float = false;

      auto float_thermal_msg = std::make_unique<custom_msgs::msg::OptrisTemperatureMatrix>();
      float_thermal_msg->header.frame_id = "camera_link_FLOATthermal";
      float_thermal_msg->width  = ThermalCameraDev_->thermal_width_;
      float_thermal_msg->height = ThermalCameraDev_->thermal_height_;
      float_thermal_msg->header.stamp = rclcpp::Time(timestamp);
      float_thermal_msg->data.resize(float_thermal_size);
      
      std::memcpy(&float_thermal_msg->data[0], float_capture_buffer, float_thermal_size);

      // Sending msg
      tempfloat_image_publisher_->publish(std::move(float_thermal_msg));
    }
    else
    {
      bad_float = true;
    }

    // Cleaning buffers
    if(rgb_capture_buffer) delete [] rgb_capture_buffer;
    if(float_capture_buffer) delete [] float_capture_buffer;
  }
}


node_interfaces::LifecycleNodeInterface::CallbackReturn thermalCameraNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On activate state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn thermalCameraNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On deactivate state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn thermalCameraNode::on_cleanup(const rclcpp_lifecycle::State & state)
{
  tempRGB_image_publisher_.reset();
  thermalImagepublisher_timer_.reset();
  thermalFeedPooler_timer_.reset();


  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On cleanup state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn thermalCameraNode::on_shutdown(const rclcpp_lifecycle::State & state)
{
  tempRGB_image_publisher_.reset();
  thermalImagepublisher_timer_.reset();
  thermalFeedPooler_timer_.reset();

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On on_shutdown state");

  ThermalCameraDev_->stopCamera();

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn thermalCameraNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On error state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
}


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exe;

  std::shared_ptr<thermalCameraNode> thermal_camera_node = std::make_shared<thermalCameraNode>("thermal_camera_node");
  exe.add_node(thermal_camera_node->get_node_base_interface());

  exe.spin();
  rclcpp::shutdown();

  return 0;
}