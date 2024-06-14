#pragma once

#include <iostream>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>

#include "CameraParams.h"
#include "MvErrorDefine.h"

namespace mvs_camera {

class Camera {
 public:
  Camera(ros::NodeHandle& node);
  ~Camera();

  // camera handle
  void* handle = nullptr;
  int nRet = MV_OK;

 private:
  bool SetCamera();

 private:
  // param
  std::string SerialNumber = "";
  int32_t CameraMode;  // 0: USB3 Camera; 1: IP Camera

  int32_t width;
  int32_t height;
  int32_t OffsetX;
  int32_t OffsetY;

  bool TriggerEnable = false;
  int32_t FrameRate = 10;
  int32_t TriggerMode;
  int32_t TriggerSource;
  int32_t LineSelector;

  int32_t BurstFrameCount;
  // Exposure Auto Mode, 0 stands for Off, 1 stands for Once, 2 stands for Continues
  const std::string ExposureAutoStr[3] = {"Off", "Once", "Continues"};
  int32_t ExposureAutoMode = 2;
  int32_t ExposureTime = 5000;
  int32_t AutoExposureTimeLower = 65;
  int32_t AutoExposureTimeUpper = 10000;

  const std::string GammaSlectorStr[3] = {"User", "sRGB", "Off"};
  int32_t GammaSlector = 1;
  float Gamma = 0.7;

  const std::string GainAutoStr[3] = {"Off", "Once", "Continues"};
  int32_t GainAuto;

  bool SaturationEnable;
  int32_t Saturation;
};

}  // namespace mvs_camera