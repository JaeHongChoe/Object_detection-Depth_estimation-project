import cv2
import os
import numpy

mtx = numpy.array([[349.766, 0, 319.369],[0, 350.415, 215.350],[0, 0, 1]])
dist = numpy.array([[ -0.334962, 0.100784, -0.000420, -0.000808 ]])

path = "./prepare_data/output_cali/data_10/data10_000000.png"


img = cv2.imread(path)
h, w = img.shape[:2]

mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, None, (w, h), 5)
print(mapx)
print(mapy)
remap = cv2.remap(img, mapx, mapy, cv2.INTER_LINEAR)

# cv2.imwrite("./img_calibration/output_cali/result.png", remap)