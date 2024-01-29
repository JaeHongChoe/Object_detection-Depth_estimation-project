import os
import cv2
import numpy

h, w = 480, 640
mtx = numpy.array([[349.766, 0, 319.369],[0, 350.415, 215.350],[0, 0, 1]])
dist = numpy.array([[ -0.334962, 0.100784, -0.000420, -0.000808 ]])
mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, None, (w, h), 5)

# folder_num = 14
# video_path = f"./prepare_data/videos/data_{folder_num}.mp4"
# image_save_path = f"./prepare_data/output_cali/data_{folder_num}/"

video_path = "./prepare_data/test.mp4"
image_save_path = "./prepare_data/output_cali/test/"

if not os.path.exists(image_save_path):
    os.makedirs(image_save_path)
    # print(f"create data_{folder_num} folder")

cap = cv2.VideoCapture(video_path)

fps = cap.get(cv2.CAP_PROP_FPS)
save_fps = 10
save_interval = int(round(fps / save_fps))

frame_num = 0
save_num = 0

while True:
    ret, frame = cap.read()

    if not ret:
        break

    if frame_num % save_interval == 0:
        
        remap = cv2.remap(frame, mapx, mapy, cv2.INTER_LINEAR)

        # cv2.imwrite(image_save_path + f'data{folder_num}_{save_num:06d}.png', remap)
        # print("save : " , f'data{folder_num}_{save_num:06d}.png')

        #test
        cv2.imwrite(image_save_path + f'test_{save_num:06d}.png', remap)
        print("save : " , f'test_{save_num:06d}.png')

        save_num += 1
    frame_num += 1  

cap.release()