class multiSeg:
  def __init__(self, seq, stamp, multi_type, depth_matrix, seg_matrix, class_ids, class_names, r_color, g_color, b_color, cost_list, x_pos, y_pos, x, y, z, w, lat, lon, heading):
    self.seq          = seq
    self.stamp        = stamp
    self.multi_type   = multi_type
    self.depth_matrix = depth_matrix
    self.seg_matrix   = seg_matrix
    self.class_ids    = class_ids
    self.class_names  = class_names
    self.r_color      = r_color
    self.g_color      = g_color
    self.b_color      = b_color
    self.cost_list    = cost_list

    # Nav2 msg
    self.x_pos    = x_pos
    self.y_pos    = y_pos
    self.x        = x
    self.y        = y
    self.z        = z
    self.w        = w
    self.lat      = lat
    self.lon      = lon
    self.heading  = heading