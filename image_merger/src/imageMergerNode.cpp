#include "imageMergerNode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>
#include <cmath>

imageMergerNode::imageMergerNode(const std::string & node_name, bool intra_process_comms)
  : LifecycleNode(node_name,rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  this->declare_parameter("config_file", "");
}

imageMergerNode::~imageMergerNode()
{
}

node_interfaces::LifecycleNodeInterface::CallbackReturn imageMergerNode::on_configure(const rclcpp_lifecycle::State & state)
{
    
  bool configure_exit_state = false;
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On configure state");
  
  std::string config_file = this->get_parameter("config_file").as_string();
  YAML::Node config       = YAML::LoadFile(config_file);
  if (!config["multi_img_publish_hz_rate"] || !config["output_topic_multi_img"] || !config["output_multi_img_height"] || !config["output_multi_img_width"] || !config["output_multi_img_channels"] || !config["input_realsense_RGB"] || !config["input_realsense_Depth"] || !config["input_thermal_float"] || !config["debug_mode"] || !config["input_rover_pose"] || !config["input_rover_gps"] ) 
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  
  // Read configuration
  float config_multi_img_publish_hz_rate       = config["multi_img_publish_hz_rate"].as<float>();
  std::string config_input_realsense_RGB       = config["input_realsense_RGB"].as<std::string>();
  std::string config_input_realsense_Depth     = config["input_realsense_Depth"].as<std::string>();
  std::string config_input_thermal_float       = config["input_thermal_float"].as<std::string>();
  std::string config_input_rover_pose          = config["input_rover_pose"].as<std::string>();
  std::string config_input_rover_gps           = config["input_rover_gps"].as<std::string>();
  std::string config_output_topic_multi_img    = config["output_topic_multi_img"].as<std::string>();
  output_multi_img_height_                     = config["output_multi_img_height"].as<int>();
  output_multi_img_width_                      = config["output_multi_img_width"].as<int>();
  output_multi_img_channels_                   = config["output_multi_img_channels"].as<int>();
  input_thermal_img_width_                     = config["input_thermal_img_width"].as<int>();
  input_thermal_img_height_                    = config["input_thermal_img_height"].as<int>();
  debug_mode_                                  = config["debug_mode"].as<bool>();
  int publish_rate                             = floor((1/config_multi_img_publish_hz_rate) * 1000);

  // Creating publisher and subscriber
  image_publisher_         = this->create_publisher<custom_msgs::msg::FusionMatrix>(config_output_topic_multi_img, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  publishMultiArray_timer_ = this->create_wall_timer(std::chrono::milliseconds(publish_rate), std::bind(&imageMergerNode::publishMultiArray, this));
  
  // Specify parallel callback group as thery are mutually exclusive by default
  // https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html
  rclcpp::SubscriptionOptions options;
  auto my_callback_group   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  options.callback_group   = my_callback_group;
  realsense_depth_sub_     = this->create_subscription<custom_msgs::msg::RealsenseDepthMatrix>(config_input_realsense_Depth, 10, std::bind(&imageMergerNode::subRealsenseDepth, this, std::placeholders::_1), options);
  realsense_rgb_sub_       = this->create_subscription<sensor_msgs::msg::Image>(config_input_realsense_RGB, 10, std::bind(&imageMergerNode::subRealsenseRGB, this, std::placeholders::_1), options);
  optris_temp_sub_         = this->create_subscription<custom_msgs::msg::OptrisTemperatureMatrix>(config_input_thermal_float, 10, std::bind(&imageMergerNode::subOptrisTemp, this, std::placeholders::_1), options);
  gps_sub_                 = this->create_subscription<sensor_msgs::msg::NavSatFix>(config_input_rover_gps, 10, std::bind(&imageMergerNode::subRoverGPS, this, std::placeholders::_1), options);
  rover_pose_sub_          = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(config_input_rover_pose, 10, std::bind(&imageMergerNode::subRoverPose, this, std::placeholders::_1), options);

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}


void imageMergerNode::publishMultiArray()
{
  // Image dimensions
  int multi_height_            = output_multi_img_height_;
  int multi_width_             = output_multi_img_width_;
  int multi_channels_          = output_multi_img_channels_; 

  if (!image_publisher_->is_activated()) 
  {
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Lifecycle publisher is currently inactive. Messages are not published.");
  }
  else
  {
    // 0 by default
    rover_pose msg_pose = {0};
    rover_gps msg_gps   = {0};
    
    if(!q_rover_pose.empty() && !q_rover_gps.empty())
    {  
      // Struct to store the rover_pose data
      pthread_mutex_lock(&rover_pose_mutex_);
      msg_pose = q_rover_pose.top();
      pthread_mutex_unlock(&rover_pose_mutex_);

      // Struct to store the rover_gps data
      pthread_mutex_lock(&rover_gps_mutex_);
      msg_gps = q_rover_gps.top();
      pthread_mutex_unlock(&rover_gps_mutex_);
    }
    
    if(!q_realsense_RGB_.empty() && !q_realsense_Depth_.empty() && !q_thermal_float_.empty())
    {  
      // Vector to store the image data
      pthread_mutex_lock(&rgb_mutex_);
      std::vector<std::vector<std::vector<unsigned char>>> rgb_img = q_realsense_RGB_.top();
      pthread_mutex_unlock(&rgb_mutex_);

      pthread_mutex_lock(&depth_mutex_);
      std::vector<std::vector<float>> depth_img                    = q_realsense_Depth_.top();
      pthread_mutex_unlock(&depth_mutex_);

      pthread_mutex_lock(&thermal_mutex_);
      std::vector<std::vector<float>> thermal_img                  = q_thermal_float_.top();
      pthread_mutex_unlock(&thermal_mutex_);
      
      if (debug_mode_)
      {
        showRGBImage(rgb_img);
        showcsvImage(thermal_img);
        showDepthImage(depth_img);
      }
      
      auto multi_array_msg = std::make_unique<custom_msgs::msg::FusionMatrix>();
      int multi_size = multi_height_ * multi_width_ * multi_channels_;

      // Nav2 related fields
      multi_array_msg->x_pos = msg_pose.x_pos;
      multi_array_msg->y_pos = msg_pose.y_pos;
      multi_array_msg->x     = msg_pose.x;
      multi_array_msg->y     = msg_pose.y;
      multi_array_msg->z     = msg_pose.z;
      multi_array_msg->w     = msg_pose.w;
      multi_array_msg->lat   = msg_gps.lat;         
      multi_array_msg->lon   = msg_gps.lon;
      multi_array_msg->local_heading = msg_pose.local_heading;

      // Message configuring
      multi_array_msg->layout.dim.emplace_back();
      multi_array_msg->layout.dim.emplace_back();
      multi_array_msg->layout.dim.emplace_back();
      multi_array_msg->seq                  = multi_seq_;
      multi_array_msg->header.stamp         = rclcpp::Clock(RCL_SYSTEM_TIME).now();
      multi_array_msg->header.frame_id      = "rgbdt";
      multi_array_msg->layout.dim[0].label  = "height";
      multi_array_msg->layout.dim[0].size   = multi_height_;
      multi_array_msg->layout.dim[0].stride = multi_size;
      multi_array_msg->layout.dim[1].label  = "width";
      multi_array_msg->layout.dim[1].size   = multi_width_;
      multi_array_msg->layout.dim[1].stride = multi_width_ * multi_channels_;
      multi_array_msg->layout.dim[2].label  = "channels";
      multi_array_msg->layout.dim[2].size   = multi_channels_;
      multi_array_msg->layout.dim[2].stride = multi_channels_;
      multi_array_msg->layout.data_offset   = 0;

      // Allocating memory for message
      multi_array_msg->data.resize(multi_size);
      
      float input_data = 0;
      int array_index  = 0;
      // Filling the message with data from the images
      for (int c = 0; c < multi_channels_; ++c) 
      {
        for (int i = 0; i < multi_height_; ++i) 
        {
          for (int j = 0; j < multi_width_; ++j) 
          {
              if (c < 3)
                input_data = rgb_img[i][j][c];
              else if (c == 3)
                input_data = depth_img[i][j];
              else if (c == 4)
              {
                // To only fill the size of the thermal img
                if((i < input_thermal_img_height_) && (j < input_thermal_img_width_))
                  input_data = thermal_img[i][j];
                else
                  input_data = 0;
              }
              array_index = i * multi_array_msg->layout.dim[1].stride  + j * multi_array_msg->layout.dim[2].stride + c;
              multi_array_msg->data[array_index] = static_cast<float>(input_data);
          }
        }
      }
      image_publisher_->publish(std::move(multi_array_msg));

      std::string log_msg = "Message published with seq: " + std::to_string(multi_seq_) + ".";
      RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), log_msg.c_str());
      multi_seq_ += 1;

      // Clearing stacks
      pthread_mutex_lock(&rgb_mutex_);
      clearqRGB(q_realsense_RGB_);
      pthread_mutex_unlock(&rgb_mutex_);

      pthread_mutex_lock(&depth_mutex_);
      clearqFloat(q_realsense_Depth_);
      pthread_mutex_unlock(&depth_mutex_);

      pthread_mutex_lock(&thermal_mutex_);
      clearqFloat(q_thermal_float_);
      pthread_mutex_unlock(&thermal_mutex_);

      pthread_mutex_lock(&rover_pose_mutex_);
      clearqroverPose(q_rover_pose);
      pthread_mutex_unlock(&rover_pose_mutex_);

      pthread_mutex_lock(&rover_gps_mutex_);
      clearqroverGPS(q_rover_gps);
      pthread_mutex_unlock(&rover_gps_mutex_);
    }
  }
}


/* Read Realsense Depth */
void imageMergerNode::subRealsenseDepth(custom_msgs::msg::RealsenseDepthMatrix msg)
{ 
  std::vector<std::vector<float>> img_Depth(msg.height, std::vector<float>(msg.width, 0));

  // Iterate through the Depth array
  for (int i = 0; i < msg.height; i++) 
  {
    for (int j = 0; j < msg.width; j++) 
    {
      // Calculate linear index 
      int linearIndex = i * msg.width + j;

      // Store data in the corresponding matrix
      img_Depth[i][j] = msg.data[linearIndex] * msg.depth_scale; // depth_scale * data = meters
    }
  }

  pthread_mutex_lock(&depth_mutex_);
  // Pushing into queue
  q_realsense_Depth_.push(img_Depth);
  pthread_mutex_unlock(&depth_mutex_);
}


/* READ REALSENSE RGB */
void imageMergerNode::subRealsenseRGB(sensor_msgs::msg::Image msg)
{
  // Create matrices for each channel
  std::vector<std::vector<std::vector<unsigned char>>> img_RGB(msg.height, std::vector<std::vector<unsigned char>>(msg.width, std::vector<unsigned char>(3)));
  
  // Creating cv::Mat for the opencv image
  cv::Mat cvImageFromMat = cv::Mat(cv::Size(msg.width, msg.height), CV_8UC3);

  // Iterate through the RGB array and store channel data in separate matrices
  for (int i = 0; i < msg.height; i++) 
  {
    for (int j = 0; j < msg.width; j++) 
    {
      for (int channel = 0; channel < 3; ++channel) 
      {
        // Calculate linear index for accessing RGB values
        int linearIndex = 3 * (i * msg.width + j);

        // Store channel data in the corresponding matrix
        img_RGB[i][j][channel] = msg.data[linearIndex + channel];
      }
    }
  }

  pthread_mutex_lock(&rgb_mutex_);
  // Pushing image into the queue
  q_realsense_RGB_.push(img_RGB);      
  pthread_mutex_unlock(&rgb_mutex_);
}

/* Reading Optris Data */
void imageMergerNode::subOptrisTemp(custom_msgs::msg::OptrisTemperatureMatrix msg)
{
  std::vector<std::vector<float>> img_thermal_float(msg.height, std::vector<float>(msg.width, 0));
  
  // Iterate through the Depth array
  for (int i = 0; i < msg.height; i++) 
  {
    for (int j = 0; j < msg.width; j++) 
    {
      // Calculate linear index 
      int linearIndex = (i * msg.width + j);

      // Store data in the corresponding matrix
      img_thermal_float[i][j] = msg.data[linearIndex];
    }
  }

  pthread_mutex_lock(&thermal_mutex_);
  q_thermal_float_.push(img_thermal_float);   // Pushing into queue
  pthread_mutex_unlock(&thermal_mutex_);
}


/* Read rover pose */
void imageMergerNode::subRoverPose(geometry_msgs::msg::PoseWithCovarianceStamped msg)
{ 
  // Declaring struct
  rover_pose msg_pose;

  // Getting x,y pose
  msg_pose.x_pos = msg.pose.pose.position.x; // x
  msg_pose.y_pos = msg.pose.pose.position.y; // y
  msg_pose.x     = msg.pose.pose.orientation.x;
  msg_pose.y     = msg.pose.pose.orientation.y;
  msg_pose.z     = msg.pose.pose.orientation.z;
  msg_pose.w     = msg.pose.pose.orientation.w;
  
  // Getting heading
  double siny_cosp       = 2 * (msg_pose.w * msg_pose.z + msg_pose.x * msg_pose.y);
  double cosy_cosp       = 1 - 2 * (msg_pose.y * msg_pose.y + msg_pose.z * msg_pose.z);
  msg_pose.local_heading = std::atan2(siny_cosp, cosy_cosp); // heading

  pthread_mutex_lock(&rover_pose_mutex_);
  // Pushing into queue
  q_rover_pose.push(msg_pose);
  pthread_mutex_unlock(&rover_pose_mutex_);
}

/* Read rover GPS */
void imageMergerNode::subRoverGPS(sensor_msgs::msg::NavSatFix msg)
{ 
  // Declaring struct
  rover_gps msg_gps;

  // Getting lat, lon
  msg_gps.lat = msg.latitude;
  msg_gps.lon = msg.longitude;

  pthread_mutex_lock(&rover_gps_mutex_);
  // Pushing into queue
  q_rover_gps.push(msg_gps);
  pthread_mutex_unlock(&rover_gps_mutex_);
}

void imageMergerNode::clearqRGB(std::stack<std::vector<std::vector<std::vector<unsigned char>>>>  &q)
{
  while (!q.empty())
  {
    q.pop();
  }
}

void imageMergerNode::clearqFloat(std::stack<std::vector<std::vector<float>>> &q)
{
  while (!q.empty())
  {
    q.pop();
  }
}

void imageMergerNode::clearqroverPose(std::stack<rover_pose> &q)
{
  while (!q.empty())
  {
    q.pop();
  }
}

void imageMergerNode::clearqroverGPS(std::stack<rover_gps> &q)
{
  while (!q.empty())
  {
    q.pop();
  }
}

node_interfaces::LifecycleNodeInterface::CallbackReturn imageMergerNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On activate state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn imageMergerNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On deactivate state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn imageMergerNode::on_cleanup(const rclcpp_lifecycle::State & state)
{

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On cleanup state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn imageMergerNode::on_shutdown(const rclcpp_lifecycle::State & state)
{

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On on_shutdown state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn imageMergerNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On error state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
}


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<imageMergerNode> image_merger_node = std::make_shared<imageMergerNode>("imager_merger_node");
  exe.add_node(image_merger_node->get_node_base_interface());

  exe.spin();
  rclcpp::shutdown();

  return 0;
}