from typing import Optional
import rclpy
import rclpy.lifecycle

from custom_msgs.msg import SegmentationResult
from custom_msgs.msg import SegmentDemMatrix
from custom_msgs.msg import Nav2Dem

from common_libs.depth_transform import CameraTransform, MergedImage
from common_libs.utils import Stack
from .utils import multiSeg

import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")

import matplotlib
#matplotlib.use("TkAgg")

class LifecycleMapProcessor(rclpy.lifecycle.Node):

  def __init__(self, node_name, **kwargs):
    super().__init__(node_name, **kwargs)
    
    self._pub_seg          : Optional[rclpy.lifecycle.Publisher] = None
    self._pub_nav          : Optional[rclpy.lifecycle.Publisher] = None
    self._timer            : Optional[rclpy.timer.Timer]         = None
    self._input_img_stack  : Stack = Stack()

    self.declare_parameters(
      namespace='',
      parameters=[
        ('input_topic_segmentation',          rclpy.Parameter.Type.STRING),
        ('output_topic_traversability_map',   rclpy.Parameter.Type.STRING),
        ('output_topic_navigation',           rclpy.Parameter.Type.STRING),
        ('output_map_publish_hz_rate',        rclpy.Parameter.Type.DOUBLE),
        ('realsense_img_height',              rclpy.Parameter.Type.INTEGER),
        ('realsense_img_width',               rclpy.Parameter.Type.INTEGER),
        ('camera_physical_height',            rclpy.Parameter.Type.DOUBLE),
        ('camera_physical_inclination',       rclpy.Parameter.Type.DOUBLE),
        ('output_map_resolution',             rclpy.Parameter.Type.DOUBLE),
        ('max_map_distance',                  rclpy.Parameter.Type.DOUBLE),
        ('debug_mode',                        rclpy.Parameter.Type.BOOL),
        ])


  def received_segmentation_callback(self, seg_msg):

    # Reading msg parameters
    recv_seg_seq         = seg_msg.seq
    recv_seg_timestamp   = seg_msg.header.stamp

    # Reading segmentation parameters
    recv_seg_width       = seg_msg.segment_width
    recv_seg_height      = seg_msg.segment_height
    recv_seg_size        = seg_msg.segment_size
    recv_seg_class_names = seg_msg.segment_class_names
    recv_seg_class_ids   = seg_msg.segment_class_ids
    recv_r_color_list    = seg_msg.r_color
    recv_g_color_list    = seg_msg.g_color
    recv_b_color_list    = seg_msg.b_color
    recv_cosmap_list     = seg_msg.cost_list
    
    # Reading multi_img realated data
    multi_type           = seg_msg.header.frame_id
    multi_height         = seg_msg.layout.dim[0].size
    multi_width          = seg_msg.layout.dim[1].size
    multi_channels       = seg_msg.layout.dim[2].stride
    multi_stride         = seg_msg.layout.dim[1].stride

    # Nav2 msg
    rover_x_pose    = seg_msg.x_pos
    rover_y_pose    = seg_msg.y_pos
    rover_x_or      = seg_msg.x
    rover_y_or      = seg_msg.y
    rover_z_or      = seg_msg.z
    rover_w_or      = seg_msg.w
    rover_lat       = seg_msg.lat
    rover_lon       = seg_msg.lon
    rover_heading   = seg_msg.local_heading

    # Reading segmentation and depth matrix
    # By default, depth is in third channel and has same size as recv
    depth_channel        = 3
    segmentation_matrix  = np.zeros([recv_seg_height, recv_seg_width])
    depth_matrix         = np.zeros([multi_height, multi_width])
    for i in range(0, recv_seg_height):
      for j in range(0, recv_seg_width):
        # Preparing array indexes
        seg_array_index           = i * recv_seg_width + j
        depth_array_index         = i * multi_stride   + j * multi_channels + depth_channel
        
        # Filling matrices
        segmentation_matrix[i][j] = seg_msg.segment_data[seg_array_index]
        depth_matrix[i][j]        = seg_msg.data[depth_array_index]
      
    # Enqueueing message
    self.get_logger().info(f'Received image introduced into stack, with seq: {recv_seg_seq}.')
    stack_img = multiSeg(recv_seg_seq, recv_seg_timestamp, multi_type, depth_matrix, segmentation_matrix, recv_seg_class_ids, recv_seg_class_names, recv_r_color_list, recv_g_color_list, recv_b_color_list, recv_cosmap_list, rover_x_pose, rover_y_pose, rover_x_or, rover_y_or, rover_z_or, rover_w_or, rover_lat, rover_lon, rover_heading)
    self._input_img_stack.enqueue(stack_img)

    # Log msg
    info_msg = "Received message of type: " +  str(multi_type) + ", with seq: " + str(recv_seg_seq) + ", timestamp: "  \
      + str(recv_seg_timestamp) + ", and dim: [" +  str(multi_height) + ", " +  str(multi_width) + ", " + str(multi_channels) +"]."
    self.get_logger().info(info_msg)

    
  def publishMap(self):
    """Publish map when enabled."""
    
    map_msg  = None
    nav2_msg = None

    # Checking that Stack is not empty
    if len(self._input_img_stack) > 0:

      # First, we extract data from the stack
      stack_img      = self._input_img_stack.dequeue()
      depth_matrix   = stack_img.depth_matrix
      seg_matrix     = stack_img.seg_matrix

      # Calculating DEM from depth image 
      merged_image   = MergedImage()
      DEM, DEM_SEGMENT, x_DEM_array, y_DEM_array, z_DEM_array, segment_DEM_array, scaled_x_offset = \
        merged_image.computeSegmentationDEM(self.depth_camera, depth_matrix, seg_matrix, resolution = self.output_map_resolution, camera_angle = self.camera_physical_inclination, distance_limit = self.max_map_distance)

      # DEM sizes
      DEM_width  = DEM.shape[1]
      DEM_height = DEM.shape[0]
      DEM_size   = DEM_width * DEM_height

      # Filling map msg to send
      map_msg                     = SegmentDemMatrix()
      map_msg.seq                 = stack_img.seq
      map_msg.header.frame_id     = stack_img.multi_type
      map_msg.header.stamp        = stack_img.stamp
      map_msg.segment_class_ids   = stack_img.class_ids
      map_msg.segment_class_names = stack_img.class_names
      map_msg.segment_r_color     = stack_img.r_color
      map_msg.segment_g_color     = stack_img.g_color
      map_msg.segment_b_color     = stack_img.b_color
      map_msg.segment_cost_list   = stack_img.cost_list
      map_msg.height_dem          = DEM_height
      map_msg.width_dem           = DEM_width
      map_msg.height_seg          = DEM_height
      map_msg.width_seg           = DEM_width
      map_msg.map_resolution      = self.output_map_resolution

      # Nav2 msgs
      map_msg.x_pos = stack_img.x_pos
      map_msg.y_pos = stack_img.y_pos
      map_msg.x     = stack_img.x
      map_msg.y     = stack_img.y
      map_msg.z     = stack_img.z
      map_msg.w     = stack_img.w
      map_msg.lat   = stack_img.lat
      map_msg.lon   = stack_img.lon
      map_msg.local_heading = stack_img.heading
            
      # Allocating memory for message
      map_msg.data_dem         = [0.0] * DEM_size
      map_msg.data_segment_map = [0] * DEM_size

      for i in range(0, DEM_height):
        for j in range(0, DEM_width):
          array_index  = i * DEM_width + j
          map_msg.data_dem[array_index]          = DEM[i][j]
          map_msg.data_segment_map[array_index]  = DEM_SEGMENT[i][j].astype(np.int16)

      # Creating DEM_COSTMAP from DEM_SEGMENT
      DEM_COSTMAP = np.copy(DEM_SEGMENT)

      # Generating costmap depending on segmentation (MODIFY to make according to .txt)
      for i in range(0,len(stack_img.cost_list)):
        stack_img.cost_list
        DEM_COSTMAP[DEM_SEGMENT == stack_img.class_ids[i]] = stack_img.cost_list[i]

      """
      DEM_COSTMAP[DEM_SEGMENT == 0] = 0   # void, should be obstacle??
      DEM_COSTMAP[DEM_SEGMENT == 1] = 10  # Arid_soil
      DEM_COSTMAP[DEM_SEGMENT == 2] = 20  # dirt
      DEM_COSTMAP[DEM_SEGMENT == 3] = 30  # Grass
      DEM_COSTMAP[DEM_SEGMENT == 4] = 50  # Gravel
      DEM_COSTMAP[DEM_SEGMENT == 5] = 254 # Sky
      DEM_COSTMAP[DEM_SEGMENT == 6] = 254 # Vegetation
      """

      # Special cases in which DEM elevation is more important than segmentation
      # Only BASEPROD CASE number
      """
      3 Grass, 6 Rock, and 8 Vegetation 
      """
      for i in range(0, DEM_height):
        for j in range(0, DEM_width):
          if(DEM_SEGMENT[i][j] == 3 or DEM_SEGMENT[i][j] == 6 or DEM_SEGMENT[i][j] == 8):
            # If that classes are higher than 0.2 meters
            if(DEM[i][j] > 0.2):
              DEM_COSTMAP[i][j] = 254 

      # Creating copy of original costmap
      DEM_NAV2 = np.copy(DEM_COSTMAP).astype(np.int16)

      # Rotate 90 counterclockwise to indicate north as main direction of the perception
      nav2_dem_transposed = np.transpose(DEM_NAV2)
      nav2_dem_rotated    = np.flip(nav2_dem_transposed, axis = 0)

      # Creating numpy array of square size the maximum value
      dem_max_dim     = max(nav2_dem_rotated.shape)
      nav2_dem_padded = -np.ones((dem_max_dim, dem_max_dim))

      # Find embedding indexes for lower part of height and the middle of width
      height_start = nav2_dem_padded.shape[0]  - nav2_dem_rotated.shape[0]
      height_end   = nav2_dem_padded.shape[0]
      width_start  = (nav2_dem_padded.shape[1] - nav2_dem_rotated.shape[1]) // 2
      width_end    = width_start + nav2_dem_rotated.shape[1]

      # Embed the original matrix into the padded matrix
      nav2_dem_padded[height_start:height_end, width_start:width_end] = nav2_dem_rotated

      # Extracting size values
      nav2_dem_padded_height = nav2_dem_padded.shape[0]
      nav2_dem_padded_width  = nav2_dem_padded.shape[1]
      nav2_dem_padded_size   = nav2_dem_padded_height * nav2_dem_padded_width
      
      # Filling nav2_msg
      nav2_msg = Nav2Dem()
      nav2_msg.sizex      = nav2_dem_padded_width
      nav2_msg.sizey      = nav2_dem_padded_height
      nav2_msg.resolution = self.output_map_resolution

      # Indexing data for nav2_msg
      nav2_msg.dem = [0] * nav2_dem_padded_size
      for i in range(0, nav2_dem_padded_height):
        for j in range(0, nav2_dem_padded_width):
          array_index                = i * nav2_dem_padded_width + j
          nav2_msg.dem[array_index]  = nav2_dem_padded[i][j].astype(np.int16)

      if(self.debug_mode):
        plt.figure()
        plt.imshow(DEM_COSTMAP)
        plt.show()
        # merged_image.showColor3Dscatter(x_DEM_array, y_DEM_array, z_DEM_array, segment_DEM_array)

      # Nav2 msgs
      nav2_msg.x_pos = stack_img.x_pos
      nav2_msg.y_pos = stack_img.y_pos
      nav2_msg.x     = stack_img.x
      nav2_msg.y     = stack_img.y
      nav2_msg.z     = stack_img.z
      nav2_msg.w     = stack_img.w
      nav2_msg.lat   = stack_img.lat
      nav2_msg.lon   = stack_img.lon
      nav2_msg.local_heading = stack_img.heading
      
    if map_msg is not None and nav2_msg is not None:
      self.get_logger().info(f'Publishing map in topic, with seq: {map_msg.seq}.')
      self._pub_seg.publish(map_msg)
      self._pub_nav.publish(nav2_msg)

      # Check if stack is bigger than 10 images to clean it
      if len(self._input_img_stack) > 10:
        # We keep the most recent img
        self._input_img_stack.clear(num_to_keep = 1)


  def on_configure(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Configure the node, after a configuring transition is requested.
    """
    self.get_logger().info("on_configure() is called.")

    self.debug_mode                    = self.get_parameter('debug_mode').value

    # Extracting params
    config_input_segmentation          = self.get_parameter('input_topic_segmentation').value
    config_output_traverability_map    = self.get_parameter('output_topic_traversability_map').value
    config_output_publish_hz_rate      = self.get_parameter('output_map_publish_hz_rate').value
    config_output_topic_navigation     = self.get_parameter('output_topic_navigation').value

    # Defining camera transforms to be used in functions
    realsense_img_width    =  self.get_parameter('realsense_img_width').value
    realsense_img_height   =  self.get_parameter('realsense_img_height').value
    camera_physical_height =  self.get_parameter('camera_physical_height').value
    self.depth_camera      =  CameraTransform(CameraTransform.TYPE_REALSENSE, width = realsense_img_width, height = realsense_img_height, physical_height = camera_physical_height)
    
    # Configuring output map characteristics
    self.camera_physical_inclination   = self.get_parameter('camera_physical_inclination').value
    self.output_map_resolution         = self.get_parameter('output_map_resolution').value
    self.max_map_distance              = self.get_parameter('max_map_distance').value

    # Defining subscriber and publisher
    self._pub_seg     = self.create_lifecycle_publisher(SegmentDemMatrix, config_output_traverability_map, 10)
    self._pub_nav     = self.create_lifecycle_publisher(Nav2Dem, config_output_topic_navigation, 10)
    self._timer       = self.create_timer(1 / config_output_publish_hz_rate, self.publishMap)
    self.subscription = self.create_subscription(SegmentationResult, config_input_segmentation, self.received_segmentation_callback, 10)
    self.subscription  # prevent unused variable warning

    return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS



  def on_activate(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    # Log, only for demo purposes
    self.get_logger().info("on_activate() is called.")

    return super().on_activate(state)
  


  def on_deactivate(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    # Log, only for demo purposes
    self.get_logger().info("on_deactivate() is called.")

    return super().on_deactivate(state)



  def on_cleanup(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Cleanup the node, after a cleaning-up transition is requested.
    """

    self.destroy_timer(self._timer)
    self.destroy_publisher(self._pub_seg)
    self.destroy_publisher(self._pub_nav)

    self.get_logger().info('on_cleanup() is called.')
    return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS



  def on_shutdown(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Shutdown the node, after a shutting-down transition is requested.
    """
    self.destroy_timer(self._timer)
    self.destroy_publisher(self._pub_seg)
    self.destroy_publisher(self._pub_nav)

    self.get_logger().info('on_shutdown() is called.')
    return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS


def main():
  rclpy.init()

  executor = rclpy.executors.SingleThreadedExecutor()
  lc_node  = LifecycleMapProcessor('lc_map_processor')
  executor.add_node(lc_node)
  try:
    executor.spin()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    lc_node.destroy_node()

if __name__ == '__main__':
  main()