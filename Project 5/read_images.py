import cv2
import numpy as np
import glob
import matplotlib.pyplot as plt
from ReadCameraModel import *
from UndistortImage import *


fx , fy , cx , cy , camera_image , LUT = ReadCameraModel ( 'model/')

images = []
for file in glob.glob('stereo/centre/*.png'):
    tmp = cv2.imread(file,0)
    colour_img = cv2.cvtColor(tmp, cv2.COLOR_BAYER_GR2RGB)
    undistorted = UndistortImage(colour_img,LUT)
    images.append(undistorted)

np.save('image_list.npy',np.array(images))