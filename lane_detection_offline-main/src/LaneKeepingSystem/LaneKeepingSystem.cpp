#include "LaneKeepingSystem/LaneKeepingSystem.hpp"
#define Record 0
#if Record
#include <ctime>
#endif

namespace Xycar {
template <typename PREC>
LaneKeepingSystem<PREC>::LaneKeepingSystem()
{
    std::string configPath;
    mNodeHandler.getParam("config_path", configPath);
    YAML::Node config = YAML::LoadFile(configPath);

    mPID = new PIDController<PREC>(config["PID"]["P_GAIN"].as<PREC>(), config["PID"]["I_GAIN"].as<PREC>(), config["PID"]["D_GAIN"].as<PREC>());
    mMovingAverage = new MovingAverageFilter<PREC>(config["MOVING_AVERAGE_FILTER"]["SAMPLE_SIZE"].as<uint32_t>());
    mLaneDetector = new LaneDetector<PREC>(config);
 
    setParams(config);

    mPublisher = mNodeHandler.advertise<xycar_msgs::xycar_motor>(mPublishingTopicName, mQueueSize);
    // cam image init
    mSubscriber = mNodeHandler.subscribe(mSubscribedTopicName, mQueueSize, &LaneKeepingSystem::imageCallback, this);
    // YOLO init
    mSubscriberYolo = mNodeHandler.subscribe(mSubscribedTopicYolo, mQueueSize, &LaneKeepingSystem::yoloCallback, this);
    //LiDAR init
    mSubscriberLiDAR = mNodeHandler.subscribe(mSubscribedTopicLiDAR, mQueueSize, &LaneKeepingSystem::lidarCallback, this);


    cv::initUndistortRectifyMap(mtx, dist, cv::Mat(), cv::Mat(), cv::Size(640, 480), CV_32FC1, mapx, mapy);
}    // mapx, mapy = cv2.initUndistortRectifyMap(mtx, dist, None, None, (w, h), 5)


template <typename PREC>
void LaneKeepingSystem<PREC>::setParams(const YAML::Node& config)
{
    mPublishingTopicName = config["TOPIC"]["PUB_NAME"].as<std::string>();
    mSubscribedTopicName = config["TOPIC"]["SUB_NAME"].as<std::string>();
    mSubscribedTopicYolo = config["TOPIC"]["YOLO_NAME"].as<std::string>();
    mSubscribedTopicLiDAR = config["TOPIC"]["LIDAR_NAME"].as<std::string>();
    mQueueSize = config["TOPIC"]["QUEUE_SIZE"].as<uint32_t>();
    mXycarSpeed = config["XYCAR"]["START_SPEED"].as<PREC>();
    mXycarMaxSpeed = config["XYCAR"]["MAX_SPEED"].as<PREC>();
    mXycarMinSpeed = config["XYCAR"]["MIN_SPEED"].as<PREC>();
    mXycarSpeedControlThreshold = config["XYCAR"]["SPEED_CONTROL_THRESHOLD"].as<PREC>();
    mAccelerationStep = config["XYCAR"]["ACCELERATION_STEP"].as<PREC>();
    mDecelerationStep = config["XYCAR"]["DECELERATION_STEP"].as<PREC>();    
    mDebugging = config["DEBUG"].as<bool>();


#if (Record)
    outputVideo.open(getfilename(),  cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 30.0, cv::Size(640, 480), true);
#endif
}

template <typename PREC>
LaneKeepingSystem<PREC>::~LaneKeepingSystem()
{
    delete mPID;
    delete mMovingAverage;
    delete mLaneDetector;
    // delete your LaneDetector if you add your LaneDetector.
}

template <typename PREC>
void LaneKeepingSystem<PREC>::run()
{
    ros::Rate rate(kFrameRate);  
    while (ros::ok())
    {     
        ros::spinOnce();      
        if (mFrame.empty())
            continue;

        double center = mLaneDetector->processImage(mFrame,mYolo);        
        double error = (center - mLaneDetector->getWidth()/2);
        //mMovingAverage->addSample(error);                
        // auto steeringAngle = std::max(std::min(kXycarSteeringAangleLimit,(int32_t)mPID->getControlOutput(error)),-1*kXycarSteeringAangleLimit);
        auto steeringAngle = (int32_t)mPID->getControlOutput(error);
        temp_mDecelerationStep = std::round(std::abs(error)/10)*mDecelerationStep;

        std::cout << "stopline_flag >>>>>>: " << mLaneDetector->stopline_flag << std::endl;
        std::cout << "--------steeringAngle: " << steeringAngle << std::endl;

        if(avoid_flag == 1){ //stop
            if(abs(steeringAngle) < 10 ){
                driveSign(0, 0);
                continue;
            }else{
                avoid_flag = 3;
            }
        }
    
        
        else if(avoid_flag == 0){
            if(abs(steeringAngle) < 10 ){//left

                driveSign(50, 5);
                sleep(1);
                driveSign(10, 5);
                sleep(1);
                driveSign(-35, 5);
                sleep(1);
                continue;
            }
            else{
                avoid_flag = 3;
            }
        }
        else if(avoid_flag == 2){
            // if(!mYolo.bounding_boxes.empty() && mYolo.bounding_boxes[0].id != 7)
            if(abs(steeringAngle) < 10 ){//right
                driveSign(-25, 5);
                sleep(1);
                driveSign(20, 5);
                sleep(1);
                driveSign(50, 5);
                sleep(1);
                continue;
                }
            else{
                avoid_flag = 3;
            }
        } 

        if(mLaneDetector->stopline_flag == 1 && mYolo.bounding_boxes.empty()){
        // if(mLaneDetector->stopline_flag == 1){
                        
            start_time = clock();
            end_time = clock();
            
            while(true){
                end_time = clock();
                angle = 0;
                speed = 0;
                driveSign(angle, speed);

                std::cout << " stop line loop" <<std::endl;

                if(((end_time - start_time) / (double)CLOCKS_PER_SEC * 1000) > 2000){
                    break;
                }
                
                // sleep(0.1);
            }

            start_time = clock();
            end_time = clock();

            while(true){
                end_time = clock();
                angle = 8;
                speed = 5;
                driveSign(angle, speed);
                std::cout << " stop line and run loop" <<std::endl;


                if(((end_time - start_time) / (double)CLOCKS_PER_SEC * 1000) > 600){
                    break;
                }
                // sleep(1);
            }
            std::cout << " stop line " <<std::endl;
            mLaneDetector->stopline_flag = false;                       
        }

        if(!mYolo.bounding_boxes.empty()){
            switch (mYolo.bounding_boxes[0].id) {
                case 0: // left
                    if(mYolo.bounding_boxes[0].depth < 90){
                        angle = -50;
                        speed = 8;
                        // start_time = time(nullptr);
                        // end_time = start_time + 3; // (mYolo.bounding_boxes[0].depth * 0.05);
                        // while(time(nullptr) < end_time){
                        //     // if((end_time - start_time) < 3){
                        //     //     angle = -50;
                        //     // }else{
                        //     //     angle = angle -25;
                        //     // }
                        //     driveSign(angle, speed);

                        //     std::cout << " left loop" <<std::endl;

                        //     sleep(1);
                        // }
                        start_time = clock();
                        end_time = clock();

                        while(true){
                            end_time = clock();
                            driveSign(angle, speed);
                            std::cout << "left loop" <<std::endl;


                            if(((end_time - start_time) / (double)CLOCKS_PER_SEC * 1000) > 2050){
                                break;
                            }
                            // sleep(1);
                        }
                        // driveSign(angle, speed);
                        std::cout << " left " <<std::endl;
                        
                    }else{
                        drive(steeringAngle);
                    }
                    break;
                case 1: // right
                    angle = 70;
                    speed = 8.5;

                    start_time = clock();
                    end_time = clock();

                    while(true){
                        end_time = clock();
                        driveSign(angle, speed);
                        std::cout << "right loop" <<std::endl;

                        if(((end_time - start_time) / (double)CLOCKS_PER_SEC * 1000) > 2600){
                            break;
                        }
                        // sleep(1);
                    }
                    
                    // driveSign(angle, speed);
                    std::cout << " right " <<std::endl;
                    
                    break;

                case 2: // stop
                    if(mYolo.bounding_boxes[0].depth < 60){
                        // start_time = time(nullptr);
                        // end_time = start_time + 3; //(mYolo.bounding_boxes[0].depth * 0.05);
                        
                        // while(time(nullptr) < end_time){
                        //     angle = 0;
                        //     speed = 0;
                        //     driveSign(angle, speed);

                        //     std::cout << " stop loop" <<std::endl;
                        //     sleep(1);
                        // }

                        // start_time = time(nullptr);
                        // end_time = start_time + 1;

                        // while(time(nullptr) < end_time){
                        //     angle = 8;
                        //     speed = 6;
                        //     driveSign(angle, speed);

                        //     std::cout << " stop and run loop" <<std::endl;
                        //     sleep(1);
                        // }
                        // mLaneDetector->stopline_flag = false; 

                        start_time = clock();
                        end_time = clock();
                        
                        while(true){
                            end_time = clock();
                            angle = 0;
                            speed = 0;
                            driveSign(angle, speed);

                            std::cout << " stop loop" <<std::endl;

                            if(((end_time - start_time) / (double)CLOCKS_PER_SEC * 1000) > 1800){
                                break;
                            }
                            
                            // sleep(0.1);
                        }

                        start_time = clock();
                        end_time = clock();

                        while(true){
                            end_time = clock();
                            angle = 8;
                            speed = 5;
                            driveSign(angle, speed);
                            std::cout << " stop and run loop" <<std::endl;


                            if(((end_time - start_time) / (double)CLOCKS_PER_SEC * 1000) > 800){
                                break;
                            }
                            // sleep(1);
                        }
                        std::cout << " stop " <<std::endl;
                        mLaneDetector->stopline_flag = false;
                    }
                    else{
                        drive(steeringAngle);
                    }
                    mLaneDetector->stopline_flag = false;
                    break;

                case 3: // crosswalk
                    if(mYolo.bounding_boxes[0].depth <60){
                        // start_time = time(nullptr);
                        // end_time = start_time + 3; //(mYolo.bounding_boxes[0].depth * 0.05);
                        
                        // while(time(nullptr) < end_time){
                        //     angle = 0;
                        //     speed = 0;
                        //     driveSign(angle, speed);

                        //     std::cout << " crosswalk loop" <<std::endl;
                        //     sleep(1);
                        // }

                        // start_time = time(nullptr);
                        // end_time = start_time + 1;

                        // while(time(nullptr) < end_time){
                        //     angle = 8;
                        //     speed = 6;
                        //     driveSign(angle, speed);

                        //     std::cout << " crosswalk and run loop" <<std::endl;
                        //     sleep(1);
                        // }
                        // mLaneDetector->stopline_flag = false; 

                        start_time = clock();
                        end_time = clock();
                        
                        while(true){
                            end_time = clock();
                            angle = 0;
                            speed = 0;
                            driveSign(angle, speed);

                            std::cout << " cross line loop" <<std::endl;

                            if(((end_time - start_time) / (double)CLOCKS_PER_SEC * 1000) > 1800){
                                break;
                            }
                            
                            // sleep(0.1);
                        }

                        start_time = clock();
                        end_time = clock();

                        while(true){
                            end_time = clock();
                            angle = 8;
                            speed = 5;
                            driveSign(angle, speed);
                            std::cout << " cross and run loop" <<std::endl;


                            if(((end_time - start_time) / (double)CLOCKS_PER_SEC * 1000) > 800){
                                break;
                            }
                            // sleep(1);
                        }
                        mLaneDetector->stopline_flag = false;
                        std::cout << " cross " <<std::endl;
                    }
                    else{
                        drive(steeringAngle);
                    }
                    mLaneDetector->stopline_flag = false;
                    break;

                case 4: // green
                    // angle = 8;
                    // speed = 5;
                    // driveSign(angle, speed);
                    drive(steeringAngle);
                    std::cout << " green " <<std::endl;
                    mLaneDetector->stopline_flag = false;
                    break;

                case 6: // red
                    if(mYolo.bounding_boxes[0].depth < 60){
                        angle = 0;
                        speed = 0;
                        driveSign(angle, speed);
                        std::cout << " red " <<std::endl;
                        break;
                    }
                    // else{
                    //     drive(steeringAngle);
                    // }
                    mLaneDetector->stopline_flag = false;
                    

                    
                default:
                    // if(mLaneDetector->stopline_flag == 1){
                        
                    //     start_time = time(nullptr);
                    //     end_time = start_time + 3;
                        
                    //     while(time(nullptr) < end_time){
                    //         angle = 0;
                    //         speed = 0;
                    //         driveSign(angle, speed);

                    //         std::cout << " stop line loop" <<std::endl;
                    //         sleep(1);
                    //     }

                    //     start_time = time(nullptr);
                    //     end_time = start_time + 1;

                    //     while(time(nullptr) < end_time){
                    //         angle = 0;
                    //         speed = 5;
                    //         driveSign(angle, speed);

                    //         std::cout << " stop line and run loop" <<std::endl;
                    //         sleep(1);
                    //     }
                    //     std::cout << " stop line " <<std::endl;
                    //     mLaneDetector->stopline_flag = false;                       
                    // }
                    // else{
                        drive(steeringAngle);
                        std::cout << " drive " <<std::endl;
                    // }
                    break;               
            }
        }else{
            
            drive(steeringAngle);
                    
        }

        if (mDebugging)
        {       
            // cv::imshow("frame", mFrame);
            // cv::imshow("roi", mLaneDetector->getDebugROI());
            cv::imshow("Debug", mLaneDetector->getDebugFrame());

#if Record
            outputVideo.write( mLaneDetector->getDebugFrame());
            // outputVideo.write(mFrame);
#endif
            cv::waitKey(1);
        }
    }
    // outputVideo.release();
}

template <typename PREC>
void LaneKeepingSystem<PREC>::imageCallback(const sensor_msgs::Image& message)
{
    cv::Mat src = cv::Mat(message.height, message.width, CV_8UC3, const_cast<uint8_t*>(&message.data[0]), message.step);
    cv::cvtColor(src, mFrame, cv::COLOR_RGB2BGR);   

    cv::remap(mFrame, mFrame, mapx, mapy, cv::INTER_LINEAR);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::yoloCallback(const yolov7::BoundingBoxes& boundingMsg)
{
    mYolo = boundingMsg;
    // int obj_id;
    // std::cout << " test " << std::endl;
    // std::cout << mYolo << std::endl;
    // std::cout << boundingMsg.bounding_boxes << std::endl;
    
    // for(const auto& bbox : boundingMsg.bounding_boxes) {
    //     std::cout << bbox << std::endl;
    //     obj_id = bbox.id;
    // }
}


template <typename PREC>
void LaneKeepingSystem<PREC>::lidarCallback(const sensor_msgs::LaserScan& lidarMsg){

    int left_cnt = 0;
    int right_cnt = 0;

	for (int i = 0; i <= 56; i++) {
        if(lidarMsg.ranges[i] < 0.4 && lidarMsg.ranges[i] > 0.1){
            left_cnt += 1;
        }
        
	}

	for (int i = 505; i >= 448; i--) {
        if(lidarMsg.ranges[i] < 0.4 && lidarMsg.ranges[i] > 0.1){
            right_cnt += 1;
        }
	}

    std::cout << left_cnt  << "---" << right_cnt << std::endl;
    
    //sleep(1);

    if((left_cnt + right_cnt) > 50){
        avoid_flag = 1;
    }
    else if(left_cnt >= 10){
        avoid_flag = 0;
    }
    else if(right_cnt >= 10){
        avoid_flag = 2;
    }
    else{
        avoid_flag = 3;
    }

    std::cout << "avoid_flag : " << avoid_flag << std::endl;
}

template <typename PREC>
void LaneKeepingSystem<PREC>::speedControl(PREC steeringAngle)
{
    if (std::abs(steeringAngle) > mXycarSpeedControlThreshold)
    {
        mXycarSpeed -= temp_mDecelerationStep;
        mXycarSpeed = std::max(mXycarSpeed, mXycarMinSpeed);
        return;
    }

    mXycarSpeed += mAccelerationStep;
    mXycarSpeed = std::min(mXycarSpeed, mXycarMaxSpeed);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::drive(PREC steeringAngle)
{
    xycar_msgs::xycar_motor motorMessage;
    // motorMessage.angle = std::round(steeringAngle);
    motorMessage.angle = steeringAngle;

    speedControl(steeringAngle);
    mLaneDetector->setYOffset(mXycarSpeed);
    motorMessage.speed = std::round(mXycarSpeed);
    mPublisher.publish(motorMessage);
}

template <typename PREC>
void LaneKeepingSystem<PREC>::driveSign(PREC steeringAngle,PREC speed )
{
    xycar_msgs::xycar_motor motorMessage;
    // motorMessage.angle = std::round(steeringAngle);
    motorMessage.angle = steeringAngle;

    motorMessage.speed = std::round(speed);
    mPublisher.publish(motorMessage);
}




template <typename PREC>
std::string LaneKeepingSystem<PREC>::getfilename(){
    std::string str_buf;            
    time_t curTime = time(NULL); 
    struct tm* pLocal = localtime(&curTime);
    
    str_buf="/home/nvidia/xycar_ws/src/lane_detection_offline-main/"+std::to_string(pLocal->tm_year + 1900)+std::to_string(pLocal->tm_mon + 1)+std::to_string(pLocal->tm_mday)+ "_" + std::to_string(pLocal->tm_hour) + std::to_string(pLocal->tm_min) + std::to_string(pLocal->tm_sec)+".mp4";
    return str_buf;
}


template class LaneKeepingSystem<float>;
template class LaneKeepingSystem<double>;
} // namespace Xycar
