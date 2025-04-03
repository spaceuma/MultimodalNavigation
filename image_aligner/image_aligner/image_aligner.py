from typing import Optional
import rclpy
import rclpy.lifecycle

from custom_msgs.msg import FusionMatrix
from common_libs.depth_transform import CameraTransform, MergedImage
from common_libs.utils import Stack
from .utils import multiImg
from std_msgs.msg import MultiArrayDimension
from PIL import Image

import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")


class LifecycleImageAligner(rclpy.lifecycle.Node):

  def __init__(self, node_name, **kwargs):
    super().__init__(node_name, **kwargs)
    
    self._pub              : Optional[rclpy.lifecycle.Publisher] = None
    self._timer            : Optional[rclpy.timer.Timer]         = None
    self._input_img_stack  : Stack = Stack()

    self.declare_parameters(
      namespace='',
      parameters=[
        ('input_topic_multi_img',               rclpy.Parameter.Type.STRING),
        ('output_topic_aligned_img',            rclpy.Parameter.Type.STRING),
        ('output_aligned_img_publish_hz_rate',  rclpy.Parameter.Type.DOUBLE),
        ('output_aligned_img_height',           rclpy.Parameter.Type.INTEGER),
        ('output_aligned_img_width',            rclpy.Parameter.Type.INTEGER),
        ('output_aligned_img_channels',         rclpy.Parameter.Type.INTEGER),
        ('realsense_img_width',                 rclpy.Parameter.Type.INTEGER),
        ('realsense_img_height',                rclpy.Parameter.Type.INTEGER),
        ('thermal_img_width',                   rclpy.Parameter.Type.INTEGER),
        ('thermal_img_height',                  rclpy.Parameter.Type.INTEGER),
        ('camera_physical_height',              rclpy.Parameter.Type.DOUBLE),
        ('debug_mode',                          rclpy.Parameter.Type.BOOL),
        ])


  def received_image_callback(self, multi_msg):

    multi_seq       =  multi_msg.seq
    multi_timestamp =  multi_msg.header.stamp
    multi_type      =  multi_msg.header.frame_id
    multi_size      =  multi_msg.layout.dim[0].stride
    multi_height    =  multi_msg.layout.dim[0].size
    multi_width     =  multi_msg.layout.dim[1].size
    multi_channels  =  multi_msg.layout.dim[2].size

    # Nav2 msg
    rover_x_pose    = multi_msg.x_pos
    rover_y_pose    = multi_msg.y_pos
    rover_x_or      = multi_msg.x
    rover_y_or      = multi_msg.y
    rover_z_or      = multi_msg.z
    rover_w_or      = multi_msg.w
    rover_lat       = multi_msg.lat
    rover_lon       = multi_msg.lon
    rover_heading   = multi_msg.local_heading

    info_msg = "Received message of type: " +  str(multi_type) + ", with seq: " + str(multi_seq) + ", timestamp: "  \
      + str(multi_timestamp) + ", and dim: [" +  str(multi_height) + ", " +  str(multi_width) + ", " + str(multi_channels) +"]."
    self.get_logger().info(info_msg)

    multi_matrix   = np.zeros([multi_height, multi_width, multi_channels])
    
    for c in range(0, multi_channels):
      for i in range(0, multi_height):
        for j in range(0, multi_width):
          array_index  = i * multi_msg.layout.dim[1].stride  + j * multi_msg.layout.dim[2].stride + c
          multi_matrix[i][j][c] = multi_msg.data[array_index]

    # Enqueueing message
    self.get_logger().info(f'Received image introduced into stack, with seq: {multi_msg.seq}.')
    stack_img = multiImg(multi_seq, multi_timestamp, multi_type, multi_matrix, rover_x_pose, rover_y_pose, rover_x_or, rover_y_or, rover_z_or, rover_w_or, rover_lat, rover_lon, rover_heading)
    self._input_img_stack.enqueue(stack_img)

    
  def publishMultiArray(self):
    """Publish multiarray when enabled."""
    
    multi_msg = None

    # Checking that Stack is not empty
    if len(self._input_img_stack) > 0:
      # First, we compute the Aligned Fusion Matrix
      stack_img       = self._input_img_stack.dequeue()
      color_img       = stack_img.data[:, :, :3]
      depth_matrix    = stack_img.data[:, :, 3]
      thermal_matrix  = stack_img.data[:self.thermal_img_height, :self.thermal_img_width, 4]
      merged_image    = MergedImage()
      aligned_thermal = merged_image.alignThermalToDepth(self.depth_camera, self.thermal_camera, depth_matrix, thermal_matrix)
      
      if(self.debug_mode):
        merged_image.showAlignedOverlay(color_img, aligned_thermal)

      # Output aligned msg parameters
      msg_aligned_height     =  self.config_output_aligned_img_height
      msg_aligned_width      =  self.config_output_aligned_img_width
      msg_aligned_channels   =  self.config_output_aligned_img_channels
      msg_aligned_multi_size =  msg_aligned_height * msg_aligned_width * msg_aligned_channels

      # Filling aligned msg to send
      multi_msg = FusionMatrix()

      # Nav2 msgs
      multi_msg.x_pos = stack_img.x_pos
      multi_msg.y_pos = stack_img.y_pos
      multi_msg.x     = stack_img.x
      multi_msg.y     = stack_img.y
      multi_msg.z     = stack_img.z
      multi_msg.w     = stack_img.w
      multi_msg.lat   = stack_img.lat
      multi_msg.lon   = stack_img.lon
      multi_msg.local_heading = stack_img.heading

      # Img msgs
      multi_msg.layout.dim = []
      multi_msg.layout.dim.append(MultiArrayDimension())
      multi_msg.layout.dim.append(MultiArrayDimension())
      multi_msg.layout.dim.append(MultiArrayDimension())
      multi_msg.seq                  = stack_img.seq
      multi_msg.header.stamp         = stack_img.stamp
      multi_msg.header.frame_id      = stack_img.frame_id
      multi_msg.layout.dim[0].label  = "height"
      multi_msg.layout.dim[0].size   = msg_aligned_height
      multi_msg.layout.dim[0].stride = msg_aligned_multi_size
      multi_msg.layout.dim[1].label  = "width"
      multi_msg.layout.dim[1].size   = msg_aligned_width
      multi_msg.layout.dim[1].stride = msg_aligned_width * msg_aligned_channels
      multi_msg.layout.dim[2].label  = "channels"
      multi_msg.layout.dim[2].size   = msg_aligned_channels
      multi_msg.layout.dim[2].stride = msg_aligned_channels
      multi_msg.layout.data_offset   = 0
    
      # Allocating memory of float type for message
      multi_msg.data = [0.0] * msg_aligned_multi_size
      
      for c in range(0, msg_aligned_channels):
        for i in range(0, msg_aligned_height):
          for j in range(0, msg_aligned_width):
            if c < 3:
              input_data = color_img[i][j][c]
            elif c == 3:
              input_data = depth_matrix[i][j]
            elif c == 4:
              # Including aligned thermal image
              input_data = aligned_thermal[i][j]
            
            array_index  = i * multi_msg.layout.dim[1].stride  + j * multi_msg.layout.dim[2].stride + c
            multi_msg.data[array_index] = input_data
      
    if multi_msg is not None:
      self.get_logger().info(f'Publishing multi array in topic, with seq: {multi_msg.seq}.')
      self._pub.publish(multi_msg)

      # Check if stack is bigger than 10 images to clean it
      if len(self._input_img_stack) > 10:
        # We keep the most recent img
        self._input_img_stack.clear(num_to_keep = 1)


  def on_configure(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Configure the node, after a configuring transition is requested.
    """
    self.get_logger().info("on_configure() is called.")

    self.debug_mode = self.get_parameter('debug_mode').value

    # Extracting params
    config_input_topic_multi_img       = self.get_parameter('input_topic_multi_img').value
    config_output_topic_aligned_img    = self.get_parameter('output_topic_aligned_img').value
    config_multi_img_publish_hz_rate   = self.get_parameter('output_aligned_img_publish_hz_rate').value
    
    # Global size parameters
    self.config_output_aligned_img_height   = self.get_parameter('output_aligned_img_height').value
    self.config_output_aligned_img_width    = self.get_parameter('output_aligned_img_width').value
    self.config_output_aligned_img_channels = self.get_parameter('output_aligned_img_channels').value

    # Global Camera parameters
    self.realsense_img_width    =  self.get_parameter('realsense_img_width').value
    self.realsense_img_height   =  self.get_parameter('realsense_img_height').value
    self.thermal_img_width      =  self.get_parameter('thermal_img_width').value
    self.thermal_img_height     =  self.get_parameter('thermal_img_height').value
    self.camera_physical_height =  self.get_parameter('camera_physical_height').value

    # Defining camera transforms to be used in functions
    self.depth_camera    = CameraTransform(CameraTransform.TYPE_REALSENSE, width = self.realsense_img_width, height = self.realsense_img_height, physical_height = self.camera_physical_height)
    self.thermal_camera  = CameraTransform(CameraTransform.TYPE_THERMAL,   width = self.thermal_img_width,   height = self.thermal_img_height, physical_height = self.camera_physical_height)

    # Defining subscriber and publisher
    self._pub         = self.create_lifecycle_publisher(FusionMatrix, config_output_topic_aligned_img, 10)
    self._timer       = self.create_timer(1 / config_multi_img_publish_hz_rate, self.publishMultiArray)
    self.subscription = self.create_subscription(FusionMatrix, config_input_topic_multi_img, self.received_image_callback, 10)
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
    self.destroy_publisher(self._pub)

    self.get_logger().info('on_cleanup() is called.')
    return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS



  def on_shutdown(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Shutdown the node, after a shutting-down transition is requested.
    """
    self.destroy_timer(self._timer)
    self.destroy_publisher(self._pub)

    self.get_logger().info('on_shutdown() is called.')
    return rclpy.lifecycle.TransitionCallbackReturn.SUCCESS


def main():
  rclpy.init()

  executor = rclpy.executors.SingleThreadedExecutor()
  lc_node  = LifecycleImageAligner('lc_image_aligner')
  executor.add_node(lc_node)
  try:
    executor.spin()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    lc_node.destroy_node()

if __name__ == '__main__':
  main()