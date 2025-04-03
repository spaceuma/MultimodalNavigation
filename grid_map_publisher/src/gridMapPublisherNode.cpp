#include "gridMapPublisherNode.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include "yaml-cpp/yaml.h"
#include <opencv2/opencv.hpp>

using namespace grid_map;

gridMapPublisherNode::gridMapPublisherNode(const std::string & node_name, bool intra_process_comms)
  : LifecycleNode(node_name,rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
{
  this->declare_parameter("config_file", "");
}

gridMapPublisherNode::~gridMapPublisherNode()
{
}

node_interfaces::LifecycleNodeInterface::CallbackReturn gridMapPublisherNode::on_configure(const rclcpp_lifecycle::State & state)
{
    
  bool configure_exit_state = false;
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On configure state");
  
  std::string config_file = this->get_parameter("config_file").as_string();
  YAML::Node config       = YAML::LoadFile(config_file);
  if (!config["input_seg_topic"] || !config["output_gridmap_topic"]|| !config["debug_mode"]) 
    return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;

  // Read configuration
  std::string config_seg_input_topic       = config["input_seg_topic"].as<std::string>();
  std::string config_grid_map_output_topic = config["output_gridmap_topic"].as<std::string>();
  debug_mode_                              = config["debug_mode"].as<bool>();

  // Creating publisher and subscriber
  grid_map_publisher_   = this->create_publisher<grid_map_msgs::msg::GridMap>(config_grid_map_output_topic, rclcpp::QoS(rclcpp::KeepLast(1)).transient_local());
  
  // Specify parallel callback group as thery are mutually exclusive by default
  // https://docs.ros.org/en/galactic/How-To-Guides/Using-callback-groups.html
  rclcpp::SubscriptionOptions options;
  auto my_callback_group   = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  options.callback_group   = my_callback_group;
  seg_sub_ = this->create_subscription<custom_msgs::msg::SegmentDemMatrix>(config_seg_input_topic, 10, std::bind(&gridMapPublisherNode::sub_seg, this, std::placeholders::_1), options);

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void gridMapPublisherNode::sub_seg(custom_msgs::msg::SegmentDemMatrix msg)
{
  if (!grid_map_publisher_->is_activated()) 
  {
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "Lifecycle publisher is currently inactive. Messages are not published.");
  }
  else
  {
    // Create grid map.
    grid_map::GridMap map({"elevation","color"});
    map.setFrameId("map");
    
    float grid_resolution = msg.map_resolution;
    map.setGeometry(grid_map::Length(msg.height_dem * grid_resolution, msg.width_dem * grid_resolution), grid_resolution);
    
    // Find msg associated colors
    int n_segment_classes = sizeof(msg.segment_class_ids) / sizeof(msg.segment_class_ids[0]);
    std::vector<int> r_color_list(n_segment_classes);
    std::vector<int> g_color_list(n_segment_classes);
    std::vector<int> b_color_list(n_segment_classes);
    for(int i = 0; i < n_segment_classes; ++i)
    {
      r_color_list[i] = msg.segment_r_color[i];
      g_color_list[i] = msg.segment_g_color[i];
      b_color_list[i] = msg.segment_b_color[i];
    }

    // Map related logs
    std::string map_log_msg = "Created grid map with size: " + std::to_string(map.getLength().x()) + " x " + std::to_string(map.getLength().y()) + " m ";
    map_log_msg = map_log_msg + "(" + std::to_string(map.getSize()(0)) + " x " + std::to_string(map.getSize()(1)) + " cells).";
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), map_log_msg.c_str());

    map_log_msg = "Receved DEM with height: " + std::to_string(msg.height_dem) + ", and width: " + std::to_string(msg.width_dem);
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), map_log_msg.c_str());

    // Initating vectors
    std::vector<std::vector<float>> img_DEM(msg.height_dem, std::vector<float>(msg.width_dem, 0));
    std::vector<std::vector<int>> img_SEG(msg.height_dem, std::vector<int>(msg.width_dem, 0));

    // Iterate through the get DEM array and segment values
    for (unsigned int i = 0; i < msg.height_dem; i++) 
    {
      for (unsigned int j = 0; j < msg.width_dem; j++) 
      {
        // Calculate linear index 
        int linearIndex = i * msg.width_dem + j;

        // Store data in the corresponding matrix
        img_DEM[i][j] = msg.data_dem[linearIndex];         // DEM
        img_SEG[i][j] = msg.data_segment_map[linearIndex]; // Segmentation class values
      }
    }
    
    // Reading msg and adapting it to grid_map convention
    unsigned int i = 0;
    unsigned int j = 0;
    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) 
    {
      grid_map::Position position;
      map.getPosition(*it, position);

      // Convert from grid 0.1 meters map to cells
      int grid_cell_x = round(position.x() * (1 / grid_resolution));
      int grid_cell_y = round(position.y() * (1 / grid_resolution));

      // Moving dem offset from center to corner
      unsigned int map_x = grid_cell_x + round(msg.height_dem / 2);
      unsigned int map_y = grid_cell_y + round(msg.width_dem  / 2);

      if (map_x < msg.height_dem && map_y < msg.width_dem)
      {
        // Defining elevation
        map.at("elevation",*it) = img_DEM[map_x][map_y];
        
        // Defining color according to class
        int segmentation_class  = img_SEG[map_x][map_y];
        
        Eigen::Vector3i colorVector(r_color_list[segmentation_class], g_color_list[segmentation_class], b_color_list[segmentation_class]);
        colorVectorToValue(colorVector, map.at("color", *it));
      }

      j = j + 1;
      if(j > msg.width_dem)
      {
        i = i + 1;
        j = 0;
      }
    }

    // Publish grid map.
    rclcpp::Time time = rclcpp::Clock(RCL_SYSTEM_TIME).now();
    map.setTimestamp(time.nanoseconds());
    std::unique_ptr<grid_map_msgs::msg::GridMap> grid_map_msg;
    grid_map_msg = grid_map::GridMapRosConverter::toMessage(map);
    grid_map_publisher_->publish(std::move(grid_map_msg));
    std::string log_msg = "Message published with seq: " + std::to_string(msg.seq) + ".";
    RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), log_msg.c_str());
  }
}


node_interfaces::LifecycleNodeInterface::CallbackReturn gridMapPublisherNode::on_activate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_activate(state);

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On activate state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn gridMapPublisherNode::on_deactivate(const rclcpp_lifecycle::State & state)
{
  LifecycleNode::on_deactivate(state);

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On deactivate state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn gridMapPublisherNode::on_cleanup(const rclcpp_lifecycle::State & state)
{

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On cleanup state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn gridMapPublisherNode::on_shutdown(const rclcpp_lifecycle::State & state)
{

  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On on_shutdown state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

node_interfaces::LifecycleNodeInterface::CallbackReturn gridMapPublisherNode::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_INFO(rclcpp::get_logger(__PRETTY_FUNCTION__), "On error state");

  return node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
}


int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exe;

  std::shared_ptr<gridMapPublisherNode> grid_map_publisher_node = std::make_shared<gridMapPublisherNode>("grid_map_publisher_node");
  exe.add_node(grid_map_publisher_node->get_node_base_interface());

  exe.spin();
  rclcpp::shutdown();

  return 0;
}