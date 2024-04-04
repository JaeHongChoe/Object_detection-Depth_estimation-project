# Object Detection & Depth Estimation
> **객체 인식과 depth 추정을 통한 차선 인식 주행 프로젝트**
> 
> 1:10 스케일 모형 차(Xycar)를 활용한 객체 인식 및 depth 추정을 통한 차선 인식 주행 프로젝트
>
> 개발 기간 : 2024.01.16 ~ 2024.01.26


## Table of Contents
1. 프로젝트 소개
2. 환경 설정
3. 실행 방법
4. 전체 과정
5. 파일 구조
6. 주행 영상
7. Stacks

## 1. 프로젝트 소개 
![image](https://github.com/nahye03/Object_detection-Depth_estimation-project/assets/54797864/234390a0-4166-499d-8ba4-e42409b9b53b)

- 카메라 센서로 차선을 인식하여 Xycar가 주어진 코스를 완수한다
- 카메라 센서에서 획득한 이미지를 object detection 하여 인식한 표지판, 신호등에 맞는 반응을 한다
- 라이다 센서를 활용하여 돌발 장애물, 차량 장애물이 있을 시 정지하거나 회피한다
- 그렇지 않을 경우 과태료가 부과된다
- 차량 주변의 객체 위치 정보를 Bird’s Eye View로 표현한다

## 2. 환경 설정
- ROS melodic
- Ubuntu 18.04 LTS
- OpenCV 4.5.5
- python3
- torch >= 1.7.0

## 3. 실행 방법
- roslaunch로 YOLOV7.launch 파일을 실행시키면 된다
```
roslaunch yolov7 YOLOV7.launch
```

## 4. 전체 과정
### 4-1. Camera Calibration
- 체스보드를 사용하여 캘리브레이션을 수행할 데이터 획득
- GML C++ Camera calibraion tool을 사용하여 camera intrinsic matrix과 distortion coefficient를 구한다
- 이를 활용하여 이미지의 왜곡을 제거한다


![image](https://github.com/nahye03/Object_detection-Depth_estimation-project/assets/54797864/0742b594-55aa-475b-bf4f-740f3eb97d70)

### 4-2. Dataset
- 직접 이미지를 촬영하여 data로 사용하였다
- 이때 사용된 이미지는 위의 calibration이 적용된 이미지이다
- CVAT tool을 사용하여 labeling을 수행한다

#### class
- 총 9개의 class
- left, right, stop, crosswalk, green_light, yellow_light, red_light, car, ignore

#### labeling 결과 annotation 파일
- annotation 파일은 txt 파일이다
- (class_id, bbox_x, bbox_y, bbox_width, bbox_height)로 구성되어 있다
- 이때 bounding box 정보인  bbox_x, bbox_y, bbox_width, bbox_height는 normalize된 값이다
- bbox_x : bounding box의 중심 x 좌표
- bbox_y : bounding box의 중심 y 좌표
- bbox_width : object의 width
- bbox_height : object의 height

#### dataset 폴더 구조
```
└─ tstl_dataset
     ├── tstl_train    
     |	├── Annotations <-- 라벨링 데이터
     |	├── ImageSets   <-- 이미지 데이터 파일명 list        
     |	├── JPEGImages	<-- 이미지 데이터
     |	└── tstl.txt	<-- class 정보 
     ├── tstl_eval 
     |	├── Annotations
     |	├── ImageSets       
     |	├── JPEGImages
     |	└── tstl.txt
     └── tstl_test
     	├── ImageSets       
     	└── JPEGImages
```
#### labeling 이미지

![image](https://github.com/nahye03/Object_detection-Depth_estimation-project/assets/54797864/1fe9bf2c-41b0-4a33-bd11-1b7e2fe3be4c)

### 4-3. YOLO v7 tiny 모델 학습
- https://github.com/WongKinYiu/yolov7
- yolo v7 모델을 전이학습하기 위해 1차로 AiHub dataset으로 학습을 진행하였다
- 그 후 획득한 데이터로 전이학습을 하였다
- 학습한 모델을 자이카가 탑재하기 위해서 onnx, tensorrt로 변환한다
- 위의 github에서 제공한 코드를 사용하여 변환하였다

#### 결과
- 예측 결과를 보면 대부분 90% 이상 confidence score를 보이며 객체 탐지에 대한 좋은 정확도와 안정성을 확인할 수 있다
- 또한, YOLOv7은 40에 가까운 FPS를 유지하며 기존 yolov3보다 대략 2~3배 높은 FPS 성능을 보여주며 실시간에 가까운 차선 인지와 객체 탐지가 가능하였다

![image](https://github.com/nahye03/Object_detection-Depth_estimation-project/assets/54797864/1f766a37-f86b-42bb-a45d-bb7823084146)
![image](https://github.com/nahye03/Object_detection-Depth_estimation-project/assets/54797864/0e00062c-d4f9-4860-b715-eeedd4e99ddd)

### 4-4. depth 추정
- 모델을 통해 예측한 object와 자이카 간의 거리를 계산하기 하였다 (object의 bounding box 밑면의 중심과 자이카 앞 간의 거리)
- world 좌표와 image 좌표쌍을 구해 homography matrix를 구하는 방법으로 depth를 추정한다
- 거리는 x, y 좌표를 모두 이용하여 두 점간의 거리를 구하는 방법을 사용하여 계산하였다

#### 결과 (Bird’s Eye View)
![image](https://github.com/nahye03/Object_detection-Depth_estimation-project/assets/54797864/95cb2ae2-7192-4679-87ff-fe2261be6d0b)

### 4-5. 주행 알고리즘
- 기본적으로 차선을 인식하여 주행한다
- 돌발 장애물과 차량 장애물이 나타나면 LiDAR를 사용하여 정지하거나 회피 주행을 수행한다
- 정지선을 인식하면 3초 정지하였다가 주행한다
  - 정지선의 경우 검출된 직선의 기울기와 길이를 통해 인식하였다
- object를 인식한 경우 그 object에 맞게 주행한다
    - 초록불 (green light) : 전진
    - 빨간불 (red light) : 정지 (초록불이 들어올 때까지 정지)
    - 정지 표지판 (stop) : 3초 정지 후 전진
    - 횡단보도 표지판 (crosswalk) : 3초 정지 후 전진
    - 오른쪽 표지판 (right) : 2초 동안 오른쪽으로 회전 후 전진
    - 왼쪽 표지판 (left) : 2초 동안 왼쪽으로 회전 후 전진

## 5. 파일 구조
- get_homography
  - homography matrix를 구하는 부분
- prepare_dataset
  - 비디오로 촬영한 영상을 이미지로 변환하는 부분
  - 이때 camera calibration을 적용하여 왜곡을 제거한다
- lane_detection_offline-main
  - 주행을 수행하는 부분
  - 카메라, LiDAR, YOLO로부터 메세지를 받아 처리하는 부분
- yolov7
  - 카메라에서 획득한 이미지에서 object detection을 수행하는 부분
  - yolo v7 모델과 weight 정보 파일이 있다
  - 예측한 object의 bounding box를 통해 자이카와의 depth 계산
  - object_id, bounding box 정보, depth 정보를 메세지로 발행한다

## 6. 주행 영상
### 전체 주행 영상
[![Video Label](http://img.youtube.com/vi/bgrc_A-WkS0/0.jpg)](https://youtu.be/bgrc_A-WkS0)

### object detection
[![Video Label](http://img.youtube.com/vi/hz_guQW5C3s/0.jpg)](https://youtu.be/hz_guQW5C3s)

### Bird’s Eye View
[![Video Label](http://img.youtube.com/vi/SFLkjMkyQ8o/0.jpg)](https://youtu.be/SFLkjMkyQ8o)

## 7. Stacks
### Environment
<img src="https://img.shields.io/badge/ubuntu-E95420?style=for-the-badge&logo=ubuntu&logoColor=white"> <img src="https://img.shields.io/badge/visualstudiocode-007ACC?style=for-the-badge&logo=visualstudiocode&logoColor=white">
<img src="https://img.shields.io/badge/git-F04032?style=for-the-badge&logo=git&logoColor=white"> 
<img src="https://img.shields.io/badge/github-181717?style=for-the-badge&logo=github&logoColor=white"> 

### Config
<img src="https://img.shields.io/badge/yaml-CB171E?style=for-the-badge&logo=yaml&logoColor=white">

### Development
<img src="https://img.shields.io/badge/cplusplus-00599C?style=for-the-badge&logo=cplusplus&logoColor=white"> <img src="https://img.shields.io/badge/cmake-064F8C?style=for-the-badge&logo=cmake&logoColor=white"> <img src="https://img.shields.io/badge/python-3776AB?style=for-the-badge&logo=python&logoColor=white"> <img src="https://img.shields.io/badge/ros-22314E?style=for-the-badge&logo=ros&logoColor=white"> <img src="https://img.shields.io/badge/opencv-5C3EE8?style=for-the-badge&logo=opencv&logoColor=white"> <img src="https://img.shields.io/badge/PyTorch-EE4C2C?style=for-the-badge&logo=pytorch&logoColor=white">
