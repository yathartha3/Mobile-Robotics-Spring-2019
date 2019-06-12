import numpy as np
import matplotlib.pyplot as plt

from skimage import data, color
from skimage.transform import hough_circle, hough_circle_peaks
from skimage.feature import canny
from skimage.draw import circle_perimeter
from skimage.util import img_as_ubyte
from skimage.io import imread, imshow

import skimage
import cv2 as cv


# Load picture and detect edges
# image = img_as_ubyte(data.coins()[160:230, 70:270])
#filename = "/home/parallels/catkin_ws/src/homework4/maps/circles_highres.png"
filename = "/home/parallels/catkin_ws/src/homework4/maps/circles_test.png"

image = imread(filename, as_gray=True)

# edges = canny(image, sigma=1)
# edges = skimage.filters.gaussian(edges, 5)
#
# imshow(edges)
# plt.show()
#
# # Detect two radii
# hough_radii = np.arange(30, 100, 2)  # These params work
# hough_res = hough_circle(edges, hough_radii)
#
# # Select the most prominent 5 circles
# accums, cx, cy, radii = hough_circle_peaks(hough_res, hough_radii,
#                                            total_num_peaks=10)
#
# # Draw them
# fig, ax = plt.subplots(ncols=1, nrows=1, figsize=(10, 4))
# image = color.gray2rgb(image)*0
#
# for center_y, center_x, radius in zip(cy, cx, radii):
#     circy, circx = circle_perimeter(center_y, center_x, radius)
#     image[circy, circx] = (220, 20, 20)
#
# ax.imshow(image, cmap=plt.cm.gray)
# plt.show()

############  same thing, but using CV2
print("same thing with CV2")

map = image
map = map.astype(np.uint8)
# kernel = np.ones((3, 3), np.float32) / 4
# map = cv.filter2D(map, -1, kernel)
circles = cv.HoughCircles(map, cv.HOUGH_GRADIENT, 1, 10,
                          param1=5, param2=5,
                          minRadius=23, maxRadius=27)
if circles is not None:
    circles = np.uint16(np.around(circles))
    for circle in circles[0, :]:
        center = (circle[0], circle[1])
        # test_circle = self.grid_to_metric_coords(circle[0], circle[1])
        # if not self.check_circle_in_list(test_circle):
        #     self.circles_pose.append(test_circle)
        # print (self.circles_pose)
        # circle center
        cv.circle(map, center, 1, (0, 100, 100), 3)
        # circle outline
        radius = circle[2]
        print "radius=", radius
        cv.circle(map, center, radius, (255, 0, 255), 3)

cv.imshow("detected circles", map)
cv.waitKey(3000)