uint32 seq
std_msgs/Header header

# Information of input multi_img
std_msgs/MultiArrayLayout layout
float32[] data

# Information of the segmentation image
uint32 segment_height           # Image height, that is, number of rows
uint32 segment_width            # Image width, that is, number of columns
uint32 segment_size
int32[] segment_class_ids       # Integer class IDs for each class
string[] segment_class_names    # String class IDs for each class
uint8[] r_color                 # R channel color list
uint8[] g_color                 # G channel color list
uint8[] b_color                 # B channel color list
uint8[] cost_list               # Associated traversability cost
uint8[] segment_data            # Actual matrix data, size is (height * width)

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