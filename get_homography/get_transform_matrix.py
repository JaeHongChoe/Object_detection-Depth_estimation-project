import os
import cv2
import matplotlib.pyplot as plt
import numpy
import math

camera_matrix = numpy.array([[349.766, 0, 319.369],[0, 350.415, 215.350],[0, 0, 1]])
dist = numpy.array([[ -0.334962, 0.100784, -0.000420, -0.000808 ]])


# height, width = image.shape[:2]

# plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
# plt.show()

# point_size = 9

# image_points = numpy.array([
#     [320, 335],
#     [320, 295],
#     [320, 280],
#     [582, 334],
#     [496, 294],
#     [425, 280],
#     [60, 335],
#     [171, 295],
#     [215, 280]
# ], dtype=numpy.float32)

# # x -> down, Y -> right, Z -> forward
# object_points = numpy.array([
#     [0.0, 0.0, 0.45],
#     [0.0, 0.0, 0.90],
#     [0.0, 0.0, 1.35],
#     [0.0, 0.45, 0.45],
#     [0.0, 0.45, 0.90],
#     [0.0, 0.45, 1.35],
#     [0.0, -0.45, 0.45],
#     [0.0, -0.45, 0.90],
#     [0.0, -0.45, 1.35],
# ], dtype=numpy.float32)


# print(image_points.shape)
# print(object_points.shape)

## check axis
# ret, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, None, useExtrinsicGuess=True, flags=cv2.SOLVEPNP_EPNP)
# img = cv2.drawFrameAxes(image, camera_matrix, None, rvec, tvec, 3, 5)
# plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
# plt.show()

#homo object point
# X -> left, Y -> forward, Z -> 1
# homo_object_point = numpy.append(object_points[:, 1:2], object_points[:, 2:3], axis=1)
# homo_object_point = numpy.append(homo_object_point, numpy.ones([1, point_size]).T, axis=1)
# # # print(homo_object_point)
# homography, _ = cv2.findHomography(image_points, homo_object_point)
# print(homography)


#---------perspective-----------------------------------
image = cv2.imread("./homo_v2.png", cv2.IMREAD_ANYCOLOR)
cv2.circle(image, (60, 335), 10, (0,0,255), -1)


bev_points = numpy.array([
    [270,360],
    [360,360],
    [360, 450],
    [270,450]
    # [180,270],
    # [270,270],
    # [360,270],
    # [180,360],
    # [180,450]
], dtype=numpy.float32)

bev_obpoints = numpy.array([
    [320, 295],
    [496, 294],
    [582, 334],
    [320, 335]
    # [215, 280],
    # [320, 280],
    # [425, 280],
    # [171, 295],
    # [60, 335]
], dtype=numpy.float32)

# mat = cv2.getPerspectiveTransform(bev_obpoints, bev_points)
# bev = cv2.warpPerspective(image, mat, (540, 540))

# bbox_point = numpy.array([60, 335, 1], dtype=numpy.float32)

# estimate = numpy.dot(mat, bbox_point)
# x, y, z = estimate[0], estimate[1], estimate[2]
# print("predict")
# print(x/z, y/z, z/z)

# cv2.circle(bev, (int(x/z), int(y/z)), 5, (255,0,0), -1)


# plt.figure(figsize=(10, 5))

# plt.subplot(1, 2, 1)
# plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

# plt.subplot(1, 2, 2)
# plt.imshow(cv2.cvtColor(bev, cv2.COLOR_BGR2RGB))
# plt.show()

#-------homo----------

homo_3d_points = numpy.array([
    [270,360,1],
    [360,360,1],
    [360,450,1],
    [270,450,1],
    [180,270,1],
    [270,270,1],
    [360,270,1],
    [180,360,1],
    [180,450,1]
], dtype=numpy.float32)

homo_points = numpy.array([
    [320, 295],
    [496, 294],
    [582, 334],
    [320, 335],
    [215, 280],
    [320, 280],
    [425, 280],
    [171, 295],
    [60, 335]
], dtype=numpy.float32)

homography, _ = cv2.findHomography(homo_points, homo_3d_points)
bev = cv2.warpPerspective(image, homography, (540, 540))


bbox_point = numpy.array([60, 335, 1], dtype=numpy.float32)

estimate = numpy.dot(homography, bbox_point)
x, y, z = estimate[0], estimate[1], estimate[2]
print("predict")
print(x/z, y/z, z/z)

cv2.circle(bev, (int(x/z), int(y/z)), 5, (255,0,0), -1)


plt.figure(figsize=(10, 5))

plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(bev, cv2.COLOR_BGR2RGB))
plt.show()

print(homography)