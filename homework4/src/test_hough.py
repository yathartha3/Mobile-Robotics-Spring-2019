import numpy as np
import matplotlib.pyplot as plt

from skimage import data, color
from skimage.transform import hough_circle, hough_circle_peaks
from skimage.feature import canny
from skimage.draw import circle_perimeter
from skimage.util import img_as_ubyte
from skimage.io import imread, imshow


# Load picture and detect edges
# image = img_as_ubyte(data.coins()[160:230, 70:270])
#filename = "/home/parallels/catkin_ws/src/homework4/maps/circles_highres.png"
filename = "/home/parallels/catkin_ws/src/homework4/maps/circles_test.png"

image = imread(filename, as_gray=True)

edges = canny(image, sigma=1)

# Detect two radii
hough_radii = np.arange(30, 100, 2)  # These params work
hough_res = hough_circle(edges, hough_radii)

# Select the most prominent 5 circles
accums, cx, cy, radii = hough_circle_peaks(hough_res, hough_radii,
                                           total_num_peaks=10)

# Draw them
fig, ax = plt.subplots(ncols=1, nrows=1, figsize=(10, 4))
image = color.gray2rgb(image)*0

for center_y, center_x, radius in zip(cy, cx, radii):
    circy, circx = circle_perimeter(center_y, center_x, radius)
    image[circy, circx] = (220, 20, 20)

ax.imshow(image, cmap=plt.cm.gray)
plt.show()