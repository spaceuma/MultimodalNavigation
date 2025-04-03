uint32 seq
std_msgs/Header header

# Information related to resolution
float32 map_resolution

# Information related to the DEM
uint32 height_dem   # image height, that is, number of rows
uint32 width_dem    # image width, that is, number of columns
float32[] data_dem

# Information related to segmentation_map
uint32 height_seg   # image height, that is, number of rows
uint32 width_seg
int32[] segment_class_ids       # Integer class IDs for each class
string[] segment_class_names    # String class IDs for each class
uint8[] segment_r_color                 # R channel color list
uint8[] segment_g_color                 # G channel color list
uint8[] segment_b_color                 # B channel color list
uint8[] segment_cost_list               # Associated traversability cost
int16[] data_segment_map

# Nav2 related msgs
# Pose
float32 x_pos
float32 y_pos

# IMU 
float32 x
float32 y
float32 z
float32 w
float32 local_heading

# GPS
float32 lat
float32 lon