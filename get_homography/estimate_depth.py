import cv2
import matplotlib.pyplot as plt
import numpy

## 1차
# homo_mat = numpy.array([
#     [-2.10492618e-01, -1.25468176e+00, 3.33525242e+02],
#     [-2.32671202e-02, -2.95749888e+00, 7.41289630e+02],
#     [-4.43564281e-05, -4.64696947e-03, 1.00000000e+00]
# ])

homo_mat = numpy.array([
    [-1.71148456e-01, -1.23451612e+00,  3.18476861e+02],
    [-4.27573615e-02, -2.63585686e+00,  6.30546015e+02],
    [-8.57910470e-05, -4.54943074e-03,  1.00000000e+00]
])



image = cv2.imread("./homo_img.png", cv2.IMREAD_ANYCOLOR)
bbox_point = numpy.array([60, 335, 1], dtype=numpy.float32)


cv2.circle(image, (60, 335), 10, (0,0,255), -1)
bev_image = cv2.warpPerspective(image, homo_mat, (540, 540))

estimate_coordinate = numpy.dot(homo_mat, bbox_point)
x, y, z = estimate_coordinate[0], estimate_coordinate[1], estimate_coordinate[2]
distance = int((540 - y/z) / 2)
print("distance : ", distance)

cv2.circle(bev_image, (int(x/z), int(y/z)), 5, (255,0,0), -1)

# vertical lines
for x in range(0, 540, 90):
    cv2.line(bev_image, (x, 0), (x, 540), (255, 255, 255), 1)

# horizon lines
for y in range(0, 540, 90):
    cv2.line(bev_image, (0, y), (540, y), (255, 255, 255), 1)



# visualization
plt.figure(figsize=(10, 5))

plt.subplot(1, 2, 1)
plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

plt.subplot(1, 2, 2)
plt.imshow(cv2.cvtColor(bev_image, cv2.COLOR_BGR2RGB))
plt.show()
