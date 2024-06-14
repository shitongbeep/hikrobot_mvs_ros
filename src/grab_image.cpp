#include "camera.h"

#include <iostream>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <pthread.h>
#include <ros/ros.h>

#include "MvCameraControl.h"
#include "shm_timer.hpp"

image_transport::Publisher pub_img;
unsigned char* pData = nullptr;
unsigned int nDataSize = 0u;
MV_FRAME_OUT_INFO_EX stImageInfo = {0};

bool liv_sync = false;

void* WorkThread(void* cam_handel);

int main(int argc, char** argv) {
  ros::init(argc, argv, "grab_image");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  ros::Rate loop_rate(10);

  std::string image_topic;
  nh.param("TopicName", image_topic, std::string("/camera/image"));
  nh.param("SyncEnable", liv_sync, false);
  pub_img = it.advertise(image_topic, 1);

  mvs_camera::Camera hik_camera(nh);

  ROS_WARN("Start Grabbing...\n");
  hik_camera.nRet = MV_CC_StartGrabbing(hik_camera.handle);
  if (MV_OK != hik_camera.nRet) {
    ROS_WARN("Start Grabbing fail.\n");
    return -1;
  } else {
    ROS_WARN("Start Grabbing succ.\n");
  }

  ROS_WARN("Getting PayloadSize...\n");
  MVCC_INTVALUE stParam;
  memset(&stParam, 0, sizeof(MVCC_INTVALUE));
  hik_camera.nRet = MV_CC_GetIntValue(hik_camera.handle, "PayloadSize", &stParam);
  if (MV_OK != hik_camera.nRet) {
    ROS_WARN("Get PayloadSize fail! nRet [0x%x]\n", hik_camera.nRet);
    return 1;
  } else {
    ROS_INFO("Get PayloadSize: %d, MALLOC %d Byte\n", stParam.nCurValue, stParam.nCurValue);
  }

  pData = (unsigned char*)malloc(sizeof(unsigned char) * stParam.nCurValue);
  if (pData == nullptr) {
    ROS_WARN("MALLOC %d Byte Fail\n", stParam.nCurValue);
    std::free(pData);
    return 2;
  }
  nDataSize = stParam.nCurValue;
  ROS_WARN("Get PayloadSize Succ. Malloc Space Succ.\n");

  ROS_WARN("Getting First RGB Image...\n");

  memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
  hik_camera.nRet = MV_CC_GetImageForBGR(hik_camera.handle, pData, stParam.nCurValue, &stImageInfo, 100);
  if (MV_OK != hik_camera.nRet) {
    ROS_ERROR("No data");
    std::free(pData);
    pData = nullptr;
    return 3;
  }
  ROS_WARN("RGB Image Size [%d, %d]...\n", stImageInfo.nHeight, stImageInfo.nWidth);

  pthread_t nThreadID;
  hik_camera.nRet = pthread_create(&nThreadID, NULL, WorkThread, hik_camera.handle);
  if (hik_camera.nRet != 0) {
    ROS_WARN("thread create failed.ret = %d\n", hik_camera.nRet);
    return -2;
  }

  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  hik_camera.nRet = MV_CC_StopGrabbing(hik_camera.handle);
  if (MV_OK != hik_camera.nRet) {
    ROS_WARN("MV_CC_StopGrabbing fail! nRet [%x]\n", hik_camera.nRet);
    return -3;
  }

  hik_camera.nRet = MV_CC_CloseDevice(hik_camera.handle);
  if (MV_OK != hik_camera.nRet) {
    ROS_WARN("MV_CC_CloseDevice fail! nRet [%x]\n", hik_camera.nRet);
    return -4;
  }

  hik_camera.nRet = MV_CC_DestroyHandle(hik_camera.handle);
  if (MV_OK != hik_camera.nRet) {
    ROS_WARN("MV_CC_DestroyHandle fail! nRet [%x]\n", hik_camera.nRet);
    return -5;
  }

  return 0;
}

void* WorkThread(void* cam_handel) {
  int nRet = MV_OK;

  int64_t last_seq = 0;
  int image_count = 0;
  shm_timer::open();
  int64_t last_image_time_ns = 0;
  while (ros::ok()) {
    memset(&stImageInfo, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    nRet = MV_CC_GetImageForBGR(cam_handel, pData, nDataSize, &stImageInfo, 1000);
    if (nRet == MV_OK) {
      ROS_WARN("Get RGB Image Size [%d, %d]...\n", stImageInfo.nHeight, stImageInfo.nWidth);
      // convert data to image
      cv::Mat srcImage;
      srcImage =
          cv::Mat(stImageInfo.nHeight, stImageInfo.nWidth, CV_8UC3, pData);
      sensor_msgs::ImagePtr msg =
          cv_bridge::CvImage(std_msgs::Header(), "rgb8", srcImage).toImageMsg();

      // sync camera with lidar
      if (!liv_sync) {
        msg->header.stamp = ros::Time::now();
      } else {
        ros::Time now_time = ros::Time::now();
        ros::Time rcv_time;
        if (shm_timer::lidar_time_mem_data == nullptr) {
          rcv_time = ros::Time::now();
          shm_timer::open();
        } else {
          if (shm_timer::lidar_time_mem_data->base_time == 0) {
            // 数据无效，使用当前时间
            rcv_time = now_time;
          } else {
            // 数据有效
            if (last_seq != shm_timer::lidar_time_mem_data->seq) {
              // 数据变更，看下lidar基准时间离哪个frame接受时间比较近
              printf("last_image_time_ns %ld now %ld base time %ld \n", last_image_time_ns, (int64_t)now_time.toNSec(),
                     shm_timer::lidar_time_mem_data->base_time);
              if (abs(last_image_time_ns - shm_timer::lidar_time_mem_data->base_time) >
                  abs((int64_t)now_time.toNSec() - shm_timer::lidar_time_mem_data->base_time)) {
                // 当前帧更近
                image_count = 0;
              } else {
                // 离上一帧更近，则当前帧是1
                image_count = 1;
              }
              last_image_time_ns = (int64_t)now_time.toNSec();
              last_seq = shm_timer::lidar_time_mem_data->seq;
            }
            printf("base_time %ld image_count %d\n", shm_timer::lidar_time_mem_data->base_time, image_count);
            rcv_time.fromNSec(shm_timer::lidar_time_mem_data->base_time + image_count * 100000000);
            image_count++;
          }
        }

        std::string debug_msg;
        debug_msg = " GetOneFrame,nFrameNum[" +
                    std::to_string(stImageInfo.nFrameNum) + "], FrameTime:" +
                    std::to_string(rcv_time.toSec()) + " now: " +
                    std::to_string(now_time.toSec());
        ROS_INFO_STREAM(debug_msg.c_str());

        msg->header.stamp = rcv_time;
      }

      // publish image
      pub_img.publish(msg);
      ros::spinOnce();
      srcImage.release();
    } else {
      ROS_WARN("MV_CC_GetImageForBGR Fail\n");
      break;
    }
  }

  if (pData) {
    free(pData);
    pData = NULL;
  }

  return 0;
}