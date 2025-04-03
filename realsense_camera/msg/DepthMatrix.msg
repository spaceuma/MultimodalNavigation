std_msgs/Header header  # Header timestamp should be acquisition time of image
                        # Header frame_id should be optical frame of cameraa
uint32 height           # image height, that is, number of rows
uint32 width            # image width, that is, number of columns
uint16[] data           # actual matrix data, size is (height * width)+
uint16 depth_scale      # depth_scale * data = meters