import os
import numpy as np
from PIL import Image

class multiImg:
  def __init__(self, seq, stamp, frame_id, data, x_pos, y_pos, x, y, z, w, lat, lon, heading):
    self.seq      = seq
    self.stamp    = stamp
    self.frame_id = frame_id
    self.data     = data

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

def load_img(filename): 
  ext = os.path.splitext(filename)[1]
  if ext in ['.npz', '.npy']:
      return Image.fromarray(np.load(filename))
  elif ext in ['.csv', '.txt']:
      return Image.fromarray(np.genfromtxt(filename, delimiter = ','))
  else:
      return Image.open(filename)