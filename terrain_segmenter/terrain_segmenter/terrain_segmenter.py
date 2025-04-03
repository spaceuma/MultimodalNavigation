from typing import Optional
from ament_index_python.packages import get_package_share_directory
import rclpy
import rclpy.lifecycle
from rclpy.qos import qos_profile_sensor_data
import PIL

from custom_msgs.msg import FusionMatrix, SegmentationResult
from sensor_msgs.msg import Image
from common_libs.utils import Stack

import torch
import torch.nn as nn
import torch.nn.functional as F
import lib_omniunet.utils.data_loading as dataloading
import lib_omniunet.utils.img_handler as img_handler
from lib_omniunet.model.omniunet_model import OmniUnet

import numpy as np
import matplotlib.pyplot as plt
import warnings
warnings.filterwarnings("ignore")

import matplotlib
# matplotlib.use("TkAgg")

class LifecycleTerrainSegmenter(rclpy.lifecycle.Node):

  def __init__(self, node_name, **kwargs):
    super().__init__(node_name, **kwargs)
    
    self._device         : str = "cpu"
    self._seg_stack      : Stack = Stack()
    self._pub            : Optional[rclpy.lifecycle.Publisher] = None
    self._timer          : Optional[rclpy.timer.Timer]         = None
    self._net            : Optional[nn.Module]                 = None
    self._classes_list   : Optional[list]                      = None
    self._classes_colors : Optional[list]                      = None
    self._cost_list      : Optional[list]                      = None

    self.declare_parameters(
      namespace='',
      parameters=[
        ('network_weights_file',   rclpy.Parameter.Type.STRING),
        ('seg_classes_file',       rclpy.Parameter.Type.STRING),
        ('seg_publish_hz_rate',    rclpy.Parameter.Type.DOUBLE),
        ('input_topic_multi_img',  rclpy.Parameter.Type.STRING),
        ('output_topic_seg_mask',  rclpy.Parameter.Type.STRING),
        ('network_max_channels',   rclpy.Parameter.Type.INTEGER),
        ('network_n_classes',      rclpy.Parameter.Type.INTEGER),
        ('output_topic_rgb_seg_mask',     rclpy.Parameter.Type.STRING),
        ('network_depth_mode',     rclpy.Parameter.Type.STRING),
        ('debug_mode',             rclpy.Parameter.Type.BOOL),
        ])


  def inference_network(self, multi_matrix, img_type):
    img_type_str = img_type.upper()

    if img_type_str == 'RGBDT':
      input_rgb     = dataloading.load_multi_matrix(multi_matrix[:, :, 0:3].astype(np.uint8), "rgb")
      input_depth   = dataloading.load_multi_matrix(multi_matrix[:, :, 3].astype(np.float32), "d")
      # input_thermal = dataloading.load_multi_matrix(multi_matrix[:, :, 4].astype(np.float32), "t")

      # Extracting lateral aligning frame mask from thermal 
      csv_thermal     = multi_matrix[:, :, 4].astype(np.float32)
      lat_mask        = (csv_thermal != 0.0).astype(np.uint8)

      # Applying mask to depth image
      input_depth_no_lat = input_depth * lat_mask

      # Applying mask to color image
      mask_color_image_array = np.stack((lat_mask, lat_mask, lat_mask), axis=-1) * 255
      mask_image             = PIL.Image.fromarray(mask_color_image_array, 'RGB')
      input_rgb_no_lat       = PIL.ImageChops.multiply(input_rgb, mask_image)

      # Creating multimodal input_img
      input_img              = dataloading.RgbdtDataset.build_rgbdt(input_rgb_no_lat, input_depth_no_lat, csv_thermal)
     
    self._net.eval()
    img = input_img.to(device = self._device, dtype = torch.float32)
    
    with torch.no_grad():
      mask_pred, omnivore_only = self._net(img)
      mask_pred_size = (input_img.shape[3], input_img.shape[4])
      mask_pred      = F.interpolate(mask_pred, size = mask_pred_size, mode = 'bicubic', align_corners = False)
      mask_pred      = torch.softmax(mask_pred, dim = 1).argmax(dim = 1)[0].float().cpu()
      mask_pred_matrix = mask_pred.cpu().detach().numpy()

      # Show mask for debugging
      if(self.debug_mode):
        # Interactive plt
        plt.close('all')
        plt.ion()
        fig = plt.figure()
        plt.imshow(mask_pred)
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.cla()

      self.get_logger().info(f'Segmentation mask correctly predicted.')
     

    return mask_pred_matrix, input_depth_no_lat, input_rgb_no_lat



  def received_image_callback(self, multi_msg):

    multi_seq       =  multi_msg.seq
    multi_timestamp =  multi_msg.header.stamp
    multi_type      =  multi_msg.header.frame_id
    multi_size      =  multi_msg.layout.dim[0].stride
    multi_height    =  multi_msg.layout.dim[0].size
    multi_width     =  multi_msg.layout.dim[1].size
    multi_channels  =  multi_msg.layout.dim[2].size

    info_msg = "Received message of type: " +  str(multi_type) + ", with seq: " + str(multi_seq) + ", timestamp: "  \
      + str(multi_timestamp) + ", and dim: [" +  str(multi_height) + ", " +  str(multi_width) + ", " + str(multi_channels) +"]."
    self.get_logger().info(info_msg)

    # We extract the input images for inferencing
    multi_matrix   = np.zeros([multi_height, multi_width, multi_channels])
    for c in range(0, multi_channels):
      for i in range(0, multi_height):
        for j in range(0, multi_width):
          array_index  = i * multi_msg.layout.dim[1].stride  + j * multi_msg.layout.dim[2].stride + c
          multi_matrix[i][j][c] = multi_msg.data[array_index]

    # We predict the segmentation mask
    mask_pred_matrix, _, _ = self.inference_network(multi_matrix, multi_type)

    # Preparing message
    output_msg = SegmentationResult()
    output_msg.seq                  = multi_seq
    output_msg.header.frame_id      = multi_type
    output_msg.header.stamp         = multi_timestamp
    output_msg.segment_width        = multi_width
    output_msg.segment_height       = multi_height
    output_msg.segment_size         = multi_width * multi_height
    output_msg.segment_class_names  = self._classes_list
    output_msg.segment_class_ids    = list(range(0, len(self._classes_list)))

    # Packing color msgs and cost
    list_length = len(self._classes_list)
    output_msg.r_color   = [0] * list_length
    output_msg.g_color   = [0] * list_length
    output_msg.b_color   = [0] * list_length
    output_msg.cost_list = [0] * list_length
    for i in range(0, list_length):
      output_msg.r_color[i]    = int(255 * self._classes_colors[i][0])
      output_msg.g_color[i]    = int(255 * self._classes_colors[i][1])
      output_msg.b_color[i]    = int(255 * self._classes_colors[i][2])
      output_msg.cost_list[i]  = int(self._cost_list[i])
    
    # Nav2 msgs
    output_msg.x_pos = multi_msg.x_pos
    output_msg.y_pos = multi_msg.y_pos
    output_msg.x     = multi_msg.x
    output_msg.y     = multi_msg.y
    output_msg.z     = multi_msg.z
    output_msg.w     = multi_msg.w
    output_msg.lat   = multi_msg.lat
    output_msg.lon   = multi_msg.lon
    output_msg.local_heading = multi_msg.local_heading

    # Including original input in the message for next node
    output_msg.layout  = multi_msg.layout
    output_msg.data    = multi_msg.data

    # Allocating memory of uint type for message
    output_msg.segment_data = [0] * output_msg.segment_size
    for i in range(0, multi_height):
      for j in range(0, multi_width):
        array_index  = i * output_msg.segment_width + j
        output_msg.segment_data[array_index] = mask_pred_matrix[i][j].astype(np.uint8)

    # Publishing Image topic with segmentation
    legend_mask                 = img_handler.plot_mask_and_legend(str(multi_seq), mask_pred_matrix, self._classes_list, self._classes_colors)
    legend_mask_resized         = legend_mask.resize((1280, 720),PIL.Image.ANTIALIAS)
    seg_img_msg = Image()
    seg_img_msg.header.stamp    = multi_timestamp
    seg_img_msg.header.frame_id = multi_type
    seg_img_msg.height          = legend_mask_resized.size[1]
    seg_img_msg.width           = legend_mask_resized.size[0]
    seg_img_msg.encoding        = "rgb8"
    seg_img_msg.is_bigendian    = 0
    seg_img_msg.step            = legend_mask_resized.size[0] * 3
    seg_img_msg.data            = np.array(legend_mask_resized).tobytes()

    if seg_img_msg is not None:
      self._pub_seg_img.publish(seg_img_msg)

    # Enqueueing message
    self.get_logger().info(f'Segmentation mask introduced into stack, with seq: {output_msg.seq}.')
    self._seg_stack.enqueue(output_msg)


  def publish_seg_mask(self):
    """Publish seg_mask when enabled."""
    
    msg = None
    if len(self._seg_stack) > 0:
      msg = self._seg_stack.dequeue()

    if msg is not None:
      self.get_logger().info(f'Publishing segmentation mask in topic, with seq: {msg.seq}.')
      self._pub.publish(msg)


  def on_configure(self, state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn:
    """
    Configure the node, after a configuring transition is requested.
    """
    self.get_logger().info("on_configure() is called.")

    self.debug_mode        = self.get_parameter('debug_mode').value

    # Extracting params
    network_weights_file   = self.get_parameter('network_weights_file').value
    seg_classes_file       = self.get_parameter('seg_classes_file').value
    seg_publish_hz_rate    = self.get_parameter('seg_publish_hz_rate').value
    input_topic_multi_img  = self.get_parameter('input_topic_multi_img').value
    output_topic_seg_mask  = self.get_parameter('output_topic_seg_mask').value
    output_topic_rgb_seg_mask  = self.get_parameter('output_topic_rgb_seg_mask').value
    network_max_channels   = self.get_parameter('network_max_channels').value
    network_n_classes      = self.get_parameter('network_n_classes').value
    network_depth_mode     = self.get_parameter('network_depth_mode').value

    # Defining configuration dirs
    pack_share_dir  = get_package_share_directory('terrain_segmenter')
    weigth_dir      = pack_share_dir + "/network_weights/" + network_weights_file
    class_dir       = pack_share_dir + "/config/" + seg_classes_file

    # Global network parameters to use in self class
    self._device          = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    self._net             = OmniUnet(max_channels = network_max_channels, n_classes = network_n_classes, depth_mode = network_depth_mode)
    self._classes_list, self._classes_colors = img_handler.parse_class_file(class_dir)
    self._cost_list = img_handler.parse_class_file_cost(class_dir)

    # Loading network weights
    loaded_model    = torch.load(weigth_dir,  map_location = self._device)
    self._net.load_state_dict(loaded_model, strict = True)
    self._net.to(device = self._device)

    # Deifining subscriber and publisher
    # https://github.com/ros2/rclpy/blob/rolling/rclpy/rclpy/qos.py
    custom_qos_profile = rclpy.qos.QoSProfile(history = rclpy.qos.QoSHistoryPolicy.KEEP_LAST, depth = 1, durability = rclpy.qos.QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE)
    self._pub         = self.create_lifecycle_publisher(SegmentationResult, output_topic_seg_mask, 10)
    self._pub_seg_img = self.create_lifecycle_publisher(Image, output_topic_rgb_seg_mask, custom_qos_profile)
    self._timer       = self.create_timer(1/seg_publish_hz_rate, self.publish_seg_mask)
    self.subscription = self.create_subscription(FusionMatrix, input_topic_multi_img, self.received_image_callback, 10)
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
  lc_node  = LifecycleTerrainSegmenter('lc_terrain_segmenter')
  executor.add_node(lc_node)
  try:
    executor.spin()
  except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
    lc_node.destroy_node()

if __name__ == '__main__':
  main()
