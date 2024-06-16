#include "camera.h"

#include "MvCameraControl.h"

namespace mvs_camera {
Camera::Camera(ros::NodeHandle& nh) {
  handle = NULL;

  // set param from yaml
  nh.param("SerialNumber", SerialNumber, std::string());

  nh.param("Width", Width, -1);
  nh.param("Height", Height, -1);
  nh.param("OffsetX", OffsetX, 0);
  nh.param("OffsetY", OffsetY, 0);

  nh.param("TriggerEnable", TriggerEnable, false);
  nh.param("FrameRate", FrameRate, 10);
  nh.param("TriggerMode", TriggerMode, 1);
  nh.param("TriggerSource", TriggerSource, 0);
  nh.param("LineSelector", LineSelector, 2);

  nh.param("GammaSlector", GammaSlector, 1);
  nh.param("Gamma", Gamma, (float)0.7);
  nh.param("GainAuto", GainAuto, 2);

  nh.param("ExposureAutoMode", ExposureAutoMode, 2);
  nh.param("ExposureTime", ExposureTime, 5000);
  nh.param("AutoExposureTimeLower", AutoExposureTimeLower, 65);
  nh.param("AutoExposureTimeUpper", AutoExposureTimeUpper, 10000);

  nh.param("SaturationEnable", SaturationEnable, true);
  nh.param("Saturation", Saturation, 128);

  nh.param("SubSample", SubSample, 1);

  //********** 枚举设备 ********************************/
  MV_CC_DEVICE_INFO_LIST stDeviceList;
  memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
  nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
    exit(-1);
  }
  int nIndex = 0;
  if (stDeviceList.nDeviceNum > 0) {
    for (int i = 0; i < stDeviceList.nDeviceNum; i++) {
      ROS_WARN("[device %d]:\n", i);
      MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
      if (pDeviceInfo == nullptr) {
        ROS_ERROR("Null MV_CC_DEVICE_INFO!\n");
        continue;
      }
      // select device and publish deivce info
      std::string expected_serial_number;
      if (pDeviceInfo->nTLayerType == MV_GIGE_DEVICE) {
        expected_serial_number = std::string(
            (char*)pDeviceInfo->SpecialInfo.stGigEInfo.chSerialNumber);
        ROS_INFO("Found IP Device: %s\n", expected_serial_number.c_str());
      } else if (pDeviceInfo->nTLayerType == MV_USB_DEVICE) {
        expected_serial_number = std::string(
            (char*)pDeviceInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        ROS_INFO("Found USB Device: %s\n", expected_serial_number.c_str());
      } else {
        ROS_INFO("Found Not Support Device Type: %d\n", pDeviceInfo->nTLayerType);
        continue;
      }

      if (!SerialNumber.empty() && SerialNumber == expected_serial_number) {
        ROS_WARN("Found User defined SerialNumber: %s\n", SerialNumber.c_str());
        nIndex = i;
        break;
      } else if (SerialNumber.empty()) {
        ROS_WARN("User NOT defined SerialNumber, Found %s\n", expected_serial_number.c_str());
        nIndex = i;
        break;
      } else {
        ROS_WARN("Pass Not User defined SerialNumber: %s\n", expected_serial_number.c_str());
      }
    }
  } else {
    ROS_ERROR("Find No Devices!\n");
    exit(-1);
  }

  nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);

  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
    exit(-1);
  }

  nRet = MV_CC_OpenDevice(handle);
  if (MV_OK != nRet) {
    ROS_ERROR("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
    exit(-1);
  } else {
    ROS_INFO("Successfully open camera!!!");
  }

  if (!SetCamera()) {
    ROS_ERROR("SetCamera fail! nRet [%x]\n", nRet);
    exit(-1);
  }

  ROS_WARN("MVS CAMERA INITIALIZED!!!");
}

bool Camera::SetCamera() {
  // set Image Size
  if (Height > 0) {
    nRet = MV_CC_SetIntValue(handle, "Height", Height);
    if (MV_OK == nRet) {
      ROS_INFO("Set Image Height: %d\n", Height);
    } else {
      ROS_WARN("Set Image Height: %d\n Fail", Height);
      return false;
    }
  }
  if (Width > 0) {
    nRet = MV_CC_SetIntValue(handle, "Width", Width);
    if (MV_OK == nRet) {
      ROS_INFO("Set Image Width: %d\n", Width);
    } else {
      ROS_WARN("Set Image Width: %d\n Fail", Width);
      return false;
    }
  }
  if (OffsetX > 0) {
    nRet = MV_CC_SetIntValue(handle, "OffsetX", OffsetX);
    if (MV_OK == nRet) {
      ROS_INFO("Set Image OffsetX: %d\n", OffsetX);
    } else {
      ROS_WARN("Set Image OffsetX: %d\n Fail", OffsetX);
      return false;
    }
  }
  if (OffsetY > 0) {
    nRet = MV_CC_SetIntValue(handle, "OffsetY", OffsetY);
    if (MV_OK == nRet) {
      ROS_INFO("Set Image OffsetY: %d\n", OffsetY);
    } else {
      ROS_WARN("Set Image OffsetY: %d\n Fail", OffsetY);
      return false;
    }
  }
  nRet = MV_CC_SetEnumValue(handle, "BinningHorizontal", SubSample);
  nRet = MV_CC_SetEnumValue(handle, "BinningVertical", SubSample);
  if (MV_OK == nRet) {
    ROS_INFO("Set Image Sub Sample: %d\n", SubSample);
  } else {
    ROS_WARN("Set Image Sub Sample: %d Fail! nRet [%x]\n", SubSample, nRet);
    return false;
  }

  // set Trigger Mode
  if (TriggerEnable) {
    ROS_INFO("Setting Hard Trigger Mode ...\n");
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 1);
    ROS_INFO("Setting Hard Trigger Source: Line %d\n", TriggerSource);
    if (TriggerSource == 0) {
      nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE0);
    } else if (TriggerSource == 1) {
      nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE1);
    } else if (TriggerSource == 2) {
      nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE2);
    } else if (TriggerSource == 3) {
      nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_LINE3);
    } else {
      ROS_WARN("Unsupport Trigger Source: Line %d\n", TriggerSource);
    }
    if (MV_OK != nRet) {
      ROS_WARN("MV_CC_SetTriggerSource fail! nRet [%x]\n", nRet);
      return false;
    } else {
      ROS_INFO("MV_CC_SetTriggerSource Succ\n");
    }
  } else {
    ROS_INFO("Setting Soft Trigger Mode ...\n");
    nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
    nRet = MV_CC_SetEnumValue(handle, "TriggerSource", MV_TRIGGER_SOURCE_SOFTWARE);
    nRet = MV_CC_SetFrameRate(handle, FrameRate);
    if (MV_OK == nRet) {
      ROS_INFO("Set Frame Rate: %d hz", FrameRate);
    } else {
      ROS_WARN("Fail to set Frame Rate\n");
    }
    nRet = MV_CC_SetExposureAutoMode(handle, 2);
    if (MV_OK == nRet) {
      std::string msg =
          "Set ExposureAutoMode: " + ExposureAutoStr[ExposureAutoMode];
      ROS_INFO(msg.c_str());
    } else {
      ROS_ERROR("Fail to set Exposure auto mode: %s\n", ExposureAutoStr[ExposureAutoMode].c_str());
      return false;
    }

    if (ExposureAutoMode == 0) {
      nRet = MV_CC_SetExposureTime(handle, ExposureTime);
      if (MV_OK == nRet) {
        std::string msg =
            "Set Exposure Time: " + std::to_string(ExposureTime) + "ms";
        ROS_INFO(msg.c_str());
      } else {
        ROS_ERROR("Fail to set Exposure Time");
        return false;
      }
    } else {
      nRet = MV_CC_SetAutoExposureTimeLower(handle, AutoExposureTimeLower);
      if (MV_OK == nRet) {
        std::string msg =
            "Set Exposure Time Lower: " + std::to_string(AutoExposureTimeLower) + "ms";
        ROS_INFO(msg.c_str());
      } else {
        ROS_ERROR("Fail to set Exposure Time Lower");
        return false;
      }

      nRet = MV_CC_SetAutoExposureTimeUpper(handle, AutoExposureTimeUpper);
      if (MV_OK == nRet) {
        std::string msg =
            "Set Exposure Time Upper: " + std::to_string(AutoExposureTimeUpper) + "ms";
        ROS_INFO(msg.c_str());
      } else {
        ROS_ERROR("Fail to set Exposure Time Upper");
        return false;
      }
    }
  }
  if (MV_OK != nRet) {
    ROS_ERROR("Set Trigger Mode fail\n");
    return false;
  }

  //*Image Format
  // 0x01100003:Mono10
  // 0x010C0004:Mono10Packed
  // 0x01100005:Mono12
  // 0x010C0006:Mono12Packed
  // 0x01100007:Mono16
  // 0x02180014:RGB8Packed
  // 0x02100032:YUV422_8
  // 0x0210001F:YUV422_8_UYVY
  // 0x01080008:BayerGR8
  // 0x01080009:BayerRG8
  // 0x0108000A:BayerGB8
  // 0x0108000B:BayerBG8
  // 0x0110000e:BayerGB10
  // 0x01100012:BayerGB12
  // 0x010C002C:BayerGB12Packed
  nRet = MV_CC_SetEnumValue(handle, "PixelFormat", 0x02180014);
  if (nRet != MV_OK) {
    ROS_ERROR("Pixel setting can't work.\n");
    return false;
  }

  nRet = MV_CC_SetEnumValue(handle, "GainAuto", GainAuto);
  if (MV_OK == nRet) {
    std::string msg = "Set Gain Auto: " + GainAutoStr[GainAuto];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gain auto mode");
    return false;
  }

  nRet = MV_CC_SetGammaSelector(handle, GammaSlector);
  if (MV_OK == nRet) {
    std::string msg = "Set GammaSlector: " + GammaSlectorStr[GammaSlector];
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set GammaSlector");
    return false;
  }

  nRet = MV_CC_SetGamma(handle, Gamma);
  if (MV_OK == nRet) {
    std::string msg = "Set Gamma: " + std::to_string(Gamma);
    ROS_INFO(msg.c_str());
  } else {
    ROS_ERROR("Fail to set Gamma");
    return false;
  }

  ROS_WARN("Finish all params set!!!");

  return true;
}

Camera::~Camera() {}

}  // namespace mvs_camera