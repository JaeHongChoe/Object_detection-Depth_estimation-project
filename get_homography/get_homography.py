import os
import cv2
import matplotlib.pyplot as plt
import numpy
import math

camera_matrix = numpy.array([[343.353, 0, 340.519], [0, 344.648, 231.522],[0, 0, 1]])
dist = numpy.array([[-0.334698, 0.129289, -0.001919, 0.000753]])

image = cv2.imread("./homo_v4.png", cv2.IMREAD_ANYCOLOR)
# plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
# plt.show()

# #-------homo----------

# # x, y, 1
# homo_3d_points = numpy.array([
#     # [180, 0, 1],
#     [270, 0, 1],
#     [360, 0, 1],
#     [180, 90, 1],
#     [270, 90, 1],
#     [360, 90, 1],
#     [180, 180, 1],
#     [270, 180, 1],
#     [360, 180, 1],
#     [180, 270, 1],
#     [270, 270, 1],
#     [360, 270, 1],
#     [180, 360, 1],
#     [270, 360, 1],
#     [360, 360, 1],
#     [180, 450, 1],
#     [270, 450, 1],
#     [360, 450, 1],
#     [90,180, 1],
#     [450,180, 1]
# ], dtype=numpy.float32)

# # x, y
# homo_points = numpy.array([
#     [313,257],
#     [370,258],
#     [247,264],
#     [313,262],
#     [380,261],
#     [232,269],
#     [313,267],
#     [394,267],
#     [209, 277],
#     [314, 276],
#     [417,275],
#     [165,295],
#     [314,294],
#     [461,292],
#     [53,336],
#     [315,332],
#     [573,328],
#     [153,270],
#     [474,266]
# ], dtype=numpy.float32)

# x, y, 1
homo_3d_points = numpy.array([
    [270, 0, 1],
    [180, 90, 1],
    [270, 90, 1],
    [360, 90, 1],
    [180, 180, 1],
    [270, 180, 1],
    [360, 180, 1],
    [180, 270, 1],
    [270, 270, 1],
    [360, 270, 1],
    [180, 360, 1],
    [270, 360, 1],
    [360, 360, 1],
    [180, 450, 1],
    [270, 450, 1],
    [360, 450, 1],
    [90,180, 1],
    [450,180, 1]
], dtype=numpy.float32)

# x, y
homo_points = numpy.array([
    [319, 256],
    [255, 260],
    [319,260],
    [386,260],
    [241, 265],
    [321, 265],
    [401,265],
    [217, 274],
    [321, 274],
    [423, 273],
    [174, 289],
    [320,289],
    [469,289],
    [70,327],
    [322,325],
    [571,325],
    [160,265],
    [479,265]
], dtype=numpy.float32)

print(homo_3d_points.shape)
print(homo_points.shape)

# # cv2.circle(image, (int(318), int(243)), 3, (0,0,255), -1)
homography, _ = cv2.findHomography(homo_points, homo_3d_points, method=cv2.RANSAC)
# # homography = numpy.array([
# #     [-1.25283901e-01, -1.08711962e+00,  3.06323264e+02],
# #     [-1.04794588e-02, -2.29153664e+00,  6.06564461e+02],
# #     [-2.54978132e-05, -4.04856877e-03,  1.00000000e+00]
# # ])
bev = cv2.warpPerspective(image, homography, (540, 540))

for coor in homo_points:
    a = numpy.append(coor, [1])
    estimate = numpy.dot(homography, a)
    x, y, z = estimate[0], estimate[1], estimate[2]
    print(a, " -> ", x/z, y/z)

    cv2.circle(bev, (int(x/z), int(y/z)), 3, (255,0,0), -1)
    cv2.circle(image, (int(a[0]), int(a[1])), 3, (255,0,0), -1)

# a = 461
# b = 292
# bbox_point = numpy.array([a, b, 1], dtype=numpy.float32)

# estimate = numpy.dot(homography, bbox_point)
# x, y, z = estimate[0], estimate[1], estimate[2]
# print("predict")
# print(x/z, y/z, z/z)

# estimate_x = x/z
# estimate_y = y/z

# vector_x = estimate_x - 270
# vector_y = estimate_y - 270

# angle_rad = math.atan2(vector_y, vector_x)
# angle_deg = math.degrees(angle_rad)

# print(angle_deg)


# # cv2.circle(bev, (int(x/z), int(y/z)), 2, (255,0,0), -1)
# # cv2.circle(image, (int(a), int(b)), 2, (255,0,0), -1)



plt.figure(figsize=(10, 5))

plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(bev, cv2.COLOR_BGR2RGB))
plt.show()

print(homography)

"""
[[-1.25283901e-01, -1.08711962e+00,  3.06323264e+02],
 [-1.04794588e-02, -2.29153664e+00,  6.06564461e+02],
 [-2.54978132e-05, -4.04856877e-03,  1.00000000e+00]]
"""

"""v4
[[-1.36650485e-01, -1.13733437e+00, 3.12113770e+02],
 [ 6.53381187e-04, -2.45035619e+00, 6.27102327e+02],
 [ 3.93808370e-06, -4.24034141e-03, 1.00000000e+00]]

"""