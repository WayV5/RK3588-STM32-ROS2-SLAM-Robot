/**************************************************************************/
/*                                                                        */
/* Copyright (c) 2013-2023 Orbbec 3D Technology, Inc                      */
/*                                                                        */
/* PROPRIETARY RIGHTS of Orbbec 3D Technology are involved in the         */
/* subject matter of this material. All manufacturing, reproduction, use, */
/* and sales rights pertaining to this subject matter are governed by the */
/* license agreement. The recipient of this software implicitly accepts   */
/* the terms of the license.                                              */
/*                                                                        */
/**************************************************************************/

#include <rclcpp/rclcpp.hpp>
#include <nlohmann/json.hpp>
#include <thread>

#include "astra_camera/utils.h"
#include "astra_camera/ob_camera_node.h"

namespace astra_camera {

OBCameraParams OBCameraNode::getCameraParams() {
  if (camera_params_.has_value()) {
    return camera_params_.value();
  }
  auto pid = device_info_.getUsbProductId();
  if (pid != DABAI_DCW_DEPTH_PID && pid != DABAI_DW_PID) {
    OBCameraParams params;
    int data_size = sizeof(OBCameraParams);
    device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&params, &data_size);
    camera_params_ = params;
    return params;
  } else {
    OBCameraParamsData camera_params_data;
    int data_size = sizeof(camera_params_data);
    memset(&camera_params_data, 0, data_size);
    int color_width = width_[COLOR];
    int color_height = height_[COLOR];
    int depth_width = width_[DEPTH];
    int depth_height = height_[DEPTH];
    if (color_width == 1024 && color_height == 768) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_1024_768;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (color_width == 512 && color_height == 384) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_512_384;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (color_width == 640 && color_height == 480) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_640_480;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (color_width == 320 && color_height == 240) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_320_240;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (color_width == 160 && color_height == 120) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_160_120;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (color_width == 480 && color_height == 360) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_480_360;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if (color_width == 240 && color_height == 180) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_240_180;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_DEFAULT;
    } else if ((color_width == 640 && color_height == 360) &&
               (depth_width == 640 && depth_height == 360) && pid == DABAI_DCW_DEPTH_PID) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_640_360;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_640_360;
    } else if ((color_width == 320 && color_height == 180) &&
               (depth_width == 320 && depth_height == 180) && pid == DABAI_DCW_DEPTH_PID) {
      camera_params_data.depthRes = XN_CAMERA_PARAMS_DEPTH_RES_320_180;
      camera_params_data.colorRes = XN_CAMERA_PARAMS_COLOR_RES_320_180;
    } else {
      OBCameraParams params;
      data_size = sizeof(OBCameraParams);
      device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&params, &data_size);
      camera_params_ = params;
      return params;
    }
    device_->getProperty(openni::OBEXTENSION_ID_CAM_PARAMS, (uint8_t*)&camera_params_data,
                         &data_size);
    camera_params_ = camera_params_data.params;
    return camera_params_data.params;
  }
}

sensor_msgs::msg::CameraInfo OBCameraNode::OBCameraParamsToCameraInfo(
    const OBCameraParams& params) {
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  camera_info.d.resize(5, 0.0);
  camera_info.d[0] = params.r_k[0];
  camera_info.d[1] = params.r_k[1];
  camera_info.d[2] = params.r_k[3];
  camera_info.d[3] = params.r_k[4];
  camera_info.d[4] = params.r_k[2];

  camera_info.k.fill(0.0);
  camera_info.k[0] = params.r_intr_p[0];
  camera_info.k[2] = params.r_intr_p[2];
  camera_info.k[4] = params.r_intr_p[1];
  camera_info.k[5] = params.r_intr_p[3];
  camera_info.k[8] = 1.0;

  camera_info.r.fill(0.0);
  camera_info.r[0] = 1.0;
  camera_info.r[4] = 1.0;
  camera_info.r[8] = 1.0;

  camera_info.p.fill(0.0);
  camera_info.p[0] = camera_info.k[0];
  camera_info.p[2] = camera_info.k[2];
  camera_info.p[3] = 0.0;
  camera_info.p[5] = camera_info.k[4];
  camera_info.p[6] = camera_info.k[5];
  camera_info.p[7] = 0.0;
  camera_info.p[10] = 1.0;
  camera_info.p[11] = 0.0;
  return camera_info;
}

double OBCameraNode::getFocalLength(const stream_index_pair& stream_index, int y_resolution) {
  if (!streams_.count(stream_index)) {
    return 0.0;
  }
  auto stream = streams_.at(stream_index);
  if (stream == nullptr) {
    return 0.0;
  }
  return static_cast<double>(y_resolution) / (2 * tan(stream->getVerticalFieldOfView() / 2));
}

CameraInfo OBCameraNode::getIRCameraInfo(int width, int height, double f) {
  if (ir_info_manager_ && ir_info_manager_->isCalibrated()) {
    auto camera_info = ir_info_manager_->getCameraInfo();
    if (camera_info.width != static_cast<uint32_t>(width) ||
        camera_info.height != static_cast<uint32_t>(height)) {
      RCLCPP_WARN(logger_,
                  "Image resolution doesn't match calibration of the IR camera. Using default "
                  "parameters.");
      RCLCPP_INFO(logger_, "camera info width = %d, height = %d", camera_info.width,
                  camera_info.height);
      return getDefaultCameraInfo(width, height, f);
    } else {
      return camera_info;
    }
  }
  auto camera_info = getDefaultCameraInfo(width, height, f);
  auto camera_params = getCameraParams();
  if (isValidCameraParams(camera_params)) {
    camera_info.d.resize(5, 0.0);

    camera_info.k.fill(0.0);
    camera_info.k[0] = camera_params.r_intr_p[0];
    camera_info.k[2] = camera_params.r_intr_p[2];
    camera_info.k[4] = camera_params.r_intr_p[1];
    camera_info.k[5] = camera_params.r_intr_p[3];
    camera_info.k[8] = 1.0;
    // left camera is depth camera, right is rgb camera
    if (!depth_registration_) {
      // if depth is not registered, then rgb is registered to depth
      camera_info.k[0] = camera_params.l_intr_p[0];
      camera_info.k[2] = camera_params.l_intr_p[2];
      camera_info.k[4] = camera_params.l_intr_p[1];
      camera_info.k[5] = camera_params.l_intr_p[3];
      camera_info.k[8] = 1.0;
    } else {
      camera_info.k[0] = camera_params.r_intr_p[0];
      camera_info.k[2] = camera_params.r_intr_p[2];
      camera_info.k[4] = camera_params.r_intr_p[1];
      camera_info.k[5] = camera_params.r_intr_p[3];
      camera_info.k[8] = 1.0;
    }

    camera_info.r.fill(0.0);
    camera_info.r[0] = 1.0;
    camera_info.r[4] = 1.0;
    camera_info.r[8] = 1.0;

    camera_info.p.fill(0.0);
    camera_info.p[0] = camera_info.k[0];
    camera_info.p[2] = camera_info.k[2];
    camera_info.p[5] = camera_info.k[4];
    camera_info.p[6] = camera_info.k[5];
    camera_info.p[10] = 1.0;
    if (device_info_.getUsbProductId() != DABAI_DCW_DEPTH_PID &&
        device_info_.getUsbProductId() != DABAI_DW_PID) {
      /* 02122020 Scale IR Params */
      double scaling = static_cast<double>(width) / 640;
      camera_info.k[0] *= scaling;  // fx
      camera_info.k[2] *= scaling;  // cx
      camera_info.k[4] *= scaling;  // fy
      camera_info.k[5] *= scaling;  // cy
      camera_info.p[0] *= scaling;  // fx
      camera_info.p[2] *= scaling;  // cx
      camera_info.p[5] *= scaling;  // fy
      camera_info.p[6] *= scaling;  // cy

      /* 02122020 end */
    }
  }
  return camera_info;
}

CameraInfo OBCameraNode::getDepthCameraInfo() {
  // The depth image has essentially the same intrinsics as the IR image, BUT the
  // principal point is offset by half the size of the hardware correlation window
  // (probably 9x9 or 9x7 in 640x480 mode). See
  // http://www.ros.org/wiki/kinect_calibration/technical
  int width = width_[DEPTH];
  int height = height_[DEPTH];
  double f = getFocalLength(DEPTH, height);
  double scaling = (double)width / 640;
  auto camera_info = getIRCameraInfo(width, height, f);
  auto camera_params = getCameraParams();
  if (!isValidCameraParams(camera_params)) {
    if (depth_registration_ || enable_colored_point_cloud_) {
      camera_info.k[0] = camera_params.r_intr_p[0];
      camera_info.k[2] = camera_params.r_intr_p[2];
      camera_info.k[4] = camera_params.r_intr_p[1];
      camera_info.k[5] = camera_params.r_intr_p[3];
      auto pid = device_info_.getUsbProductId();
      if (pid != DABAI_DCW_DEPTH_PID && pid != DABAI_DW_PID) {
        camera_info.k[0] *= scaling;  // fx
        camera_info.k[2] *= scaling;  // cx
        camera_info.k[4] *= scaling;  // fy
        camera_info.k[5] *= scaling;  // cy
      }
    }
  }
  camera_info.k[2] -= depth_ir_x_offset_ * scaling;
  camera_info.k[5] -= depth_ir_y_offset_ * scaling;
  camera_info.k[2] -= depth_ir_x_offset_ * scaling;
  camera_info.k[6] -= depth_ir_y_offset_ * scaling;

  // TODO: Could put this in projector frame so as to encode the baseline in P[3]
  return camera_info;
}

CameraInfo OBCameraNode::getColorCameraInfo() {
  int width = width_[COLOR];
  int height = height_[COLOR];
  if (color_info_manager_ && color_info_manager_->isCalibrated()) {
    auto camera_info = color_info_manager_->getCameraInfo();
    if (camera_info.width != static_cast<uint32_t>(width) ||
        camera_info.height != static_cast<uint32_t>(height)) {
      RCLCPP_WARN(logger_,
                  "Image resolution doesn't match calibration of the RGB camera. Using default "
                  "parameters.");
      double color_focal_length = getFocalLength(COLOR, height);
      return getDefaultCameraInfo(width, height, color_focal_length);
    } else {
      return camera_info;
    }
  }
  // If uncalibrated, fill in default values
  auto camera_params = getCameraParams();
  if (isValidCameraParams(camera_params)) {
    auto default_camera_info = OBCameraParamsToCameraInfo(camera_params);
    CameraInfo camera_info;
    camera_info.d.resize(5, 0.0);
    camera_info.k.fill(0.0);
    camera_info.r.fill(0.0);
    camera_info.p.fill(0.0);
    camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    camera_info.width = width;
    camera_info.height = height;

    for (int i = 0; i < 9; i++) {
      camera_info.k[i] = default_camera_info.k[i];
      camera_info.r[i] = default_camera_info.r[i];
    }

    for (int i = 0; i < 12; i++) {
      camera_info.p[i] = default_camera_info.p[i];
    }
    if (device_info_.getUsbProductId() != DABAI_DCW_DEPTH_PID) {
      /*02112020 color camera param change according to resolution */
      double scaling = (double)width / 640;
      camera_info.k[0] *= scaling;  // fx
      camera_info.k[2] *= scaling;  // cx
      camera_info.k[4] *= scaling;  // fy
      camera_info.k[5] *= scaling;  // cy
      camera_info.p[0] *= scaling;  // fx
      camera_info.p[2] *= scaling;  // cx
      camera_info.p[5] *= scaling;  // fy
      camera_info.p[6] *= scaling;  // cy

      /* 02112020 end*/
    }
    return camera_info;
    // end if (isValidCameraParams(camera_params))
  } else {
    double color_focal_length = getFocalLength(COLOR, height);
    return getDefaultCameraInfo(width, height, color_focal_length);
  }
}

}  // namespace astra_camera
