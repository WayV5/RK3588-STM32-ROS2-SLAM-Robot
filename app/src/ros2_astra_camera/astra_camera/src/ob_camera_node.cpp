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

#include "astra_camera/ob_camera_node.h"
#include "astra_camera/utils.h"
#include <cv_bridge/cv_bridge.h>
namespace astra_camera {

void OBCameraNode::init() {
  is_running_.store(true);
  std::scoped_lock<decltype(device_lock_)> lock(device_lock_);
  setupConfig();
  setupTopics();
  startStreams();
  run_streaming_poller_ = true;
  poll_stream_thread_ = std::make_shared<std::thread>([this]() { pollFrame(); });
  if (enable_point_cloud_) {
    point_cloud_processor_ = std::make_unique<PointCloudXyzNode>(node_, parameters_);
  }
  if (enable_colored_point_cloud_) {
    colored_point_cloud_processor_ = std::make_unique<PointCloudXyzrgbNode>(node_, parameters_);
  }
  is_initialized_ = true;
}

OBCameraNode::OBCameraNode(rclcpp::Node* node, std::shared_ptr<openni::Device> device,
                           std::shared_ptr<Parameters> parameters, bool use_uvc_camera)
    : node_(node),
      device_(std::move(device)),
      parameters_(std::move(parameters)),
      logger_(node->get_logger()),
      use_uvc_camera_(use_uvc_camera) {
  init();
}

OBCameraNode::~OBCameraNode() { clean(); }

void OBCameraNode::clean() {
  is_running_.store(false);
  std::scoped_lock<decltype(device_lock_)> lock(device_lock_);
  if (tf_thread_ && tf_thread_->joinable()) {
    tf_thread_->join();
  }
  run_streaming_poller_ = false;
  if (poll_stream_thread_ && poll_stream_thread_->joinable()) {
    poll_stream_thread_->join();
  }
  stopStreams();
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (streams_[stream_index]) {
      streams_[stream_index]->destroy();
    }
  }
  if (device_ && device_->isValid()) {
    device_->close();
  }
}

void OBCameraNode::pollFrame() {
  openni::VideoFrameRef frame;
  while (rclcpp::ok() && run_streaming_poller_ && is_running_) {
    std::unique_lock<decltype(stream_lock_)> lock(stream_lock_);
    using namespace std::chrono_literals;
    auto has_stream_started = stream_started_cv_.wait_for(lock, 1000ms, [this]() {
      return std::any_of(
          IMAGE_STREAMS.begin(), IMAGE_STREAMS.end(),
          [this](const auto& stream_index) { return stream_started_[stream_index]; });
    });
    if (!has_stream_started) {
      std::this_thread::sleep_for(1000ms);
      continue;
    }
    openni::VideoStream* streams[3];
    std::map<int, stream_index_pair> idx_map;
    int stream_count = 0;
    for (const auto& stream_index : IMAGE_STREAMS) {
      if (enable_stream_[stream_index] && streams_[stream_index].get()) {
        streams[stream_count] = streams_[stream_index].get();
        idx_map[stream_count] = stream_index;
        stream_count++;
      }
    }
    int ready_stream = -1;
    const static int timeout_ms(2000);
    auto status =
        openni::OpenNI::waitForAnyStream(streams, stream_count, &ready_stream, timeout_ms);
    if (status != openni::STATUS_OK) {
      continue;
    }
    CHECK(ready_stream != -1);
    auto stream_index = idx_map[ready_stream];
    status = streams[ready_stream]->readFrame(&frame);
    if (status != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_, "read " << stream_name_[stream_index] << " stream failed "
                                           << openni::OpenNI::getExtendedError());
      continue;
    }
    onNewFrameCallback(frame, stream_index);
  }
}

void OBCameraNode::stopStreams() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (stream_started_[stream_index]) {
      CHECK_NOTNULL(streams_[stream_index]);
      streams_[stream_index]->stop();
      RCLCPP_INFO_STREAM(logger_, "Stopped stream " << stream_name_[stream_index]);
      stream_started_[stream_index] = false;
    }
  }

  if (use_uvc_camera_) {
    CHECK_NOTNULL(uvc_camera_driver_);
    uvc_camera_driver_->stopStreaming();
    stream_started_[COLOR] = false;
  }
}

void OBCameraNode::setupDevices() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    stream_started_[stream_index] = false;
    if (enable_stream_[stream_index] && device_->hasSensor(stream_index.first)) {
      auto stream = std::make_shared<openni::VideoStream>();
      auto status = stream->create(*device_, stream_index.first);
      if (status != openni::STATUS_OK) {
        std::stringstream ss;
        ss << "Couldn't create depth video stream: " << openni::OpenNI::getExtendedError();
        RCLCPP_ERROR_STREAM(logger_, ss.str());
        throw std::runtime_error(ss.str());
      }
      CHECK_NOTNULL(stream);
      streams_[stream_index] = stream;
    } else {
      if (streams_[stream_index]) {
        streams_[stream_index].reset();
      }
      enable_stream_[stream_index] = false;
    }
  }
  device_info_ = device_->getDeviceInfo();
}

void OBCameraNode::setupVideoMode() {
  if (!use_uvc_camera_ && enable_stream_[INFRA1] && enable_stream_[COLOR]) {
    RCLCPP_WARN_STREAM(logger_,
                       "Infrared and Color streams are enabled. "
                       "Infrared stream will be disabled.");
    enable_stream_[INFRA1] = false;
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    supported_video_modes_[stream_index] = std::vector<openni::VideoMode>();
    if (device_->hasSensor(stream_index.first) && enable_stream_[stream_index]) {
      if (use_uvc_camera_ && stream_index == COLOR) {
        continue;
      }
      auto stream = streams_[stream_index];
      const auto& sensor_info = stream->getSensorInfo();
      const auto& supported_video_modes = sensor_info.getSupportedVideoModes();
      int size = supported_video_modes.getSize();
      for (int i = 0; i < size; i++) {
        supported_video_modes_[stream_index].emplace_back(supported_video_modes[i]);
      }
      openni::VideoMode video_mode, default_video_mode;
      video_mode.setResolution(width_[stream_index], height_[stream_index]);
      default_video_mode.setResolution(width_[stream_index], height_[stream_index]);
      video_mode.setFps(fps_[stream_index]);
      video_mode.setPixelFormat(format_[stream_index]);
      default_video_mode.setPixelFormat(format_[stream_index]);
      bool is_supported_mode = false;
      bool is_default_mode_supported = false;
      for (const auto& item : supported_video_modes_[stream_index]) {
        if (video_mode == item) {
          is_supported_mode = true;
          stream_video_mode_[stream_index] = video_mode;
          break;
        }
        if (default_video_mode.getResolutionX() == item.getResolutionX() &&
            default_video_mode.getResolutionY() == item.getResolutionY() &&
            default_video_mode.getPixelFormat() == item.getPixelFormat()) {
          default_video_mode.setFps(item.getFps());
          is_default_mode_supported = true;
        }
      }
      if (!is_supported_mode) {
        RCLCPP_WARN_STREAM(logger_, "Video mode " << video_mode << " is not supported. ");
        if (is_default_mode_supported) {
          RCLCPP_WARN_STREAM(logger_, "Default video mode " << default_video_mode
                                                            << " is supported. "
                                                               "Stream will be enabled.");
          stream_video_mode_[stream_index] = default_video_mode;
          video_mode = default_video_mode;
          is_supported_mode = true;
        } else {
          RCLCPP_WARN_STREAM(logger_, "Default video mode " << default_video_mode
                                                            << "is not supported. "
                                                               "Stream will be disabled.");
          enable_stream_[stream_index] = false;
          RCLCPP_INFO_STREAM(logger_, "Supported video modes: ");
          for (const auto& item : supported_video_modes_[stream_index]) {
            RCLCPP_INFO_STREAM(logger_, item);
          }
        }
      }
      if (is_supported_mode) {
        RCLCPP_INFO_STREAM(logger_,
                           "set " << stream_name_[stream_index] << " video mode " << video_mode);
        images_[stream_index] = cv::Mat(height_[stream_index], width_[stream_index],
                                        image_format_[stream_index], cv::Scalar(0, 0, 0));
      }
    }
  }
}

void OBCameraNode::setupCameraInfoManager() {
  if (!use_uvc_camera_ && !color_info_url_.empty()) {
    color_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
        node_, "rgb_camera", color_info_url_);
  }
  if (!ir_info_url_.empty()) {
    ir_info_manager_ =
        std::make_unique<camera_info_manager::CameraInfoManager>(node_, "ir_camera", ir_info_url_);
  }
}

void OBCameraNode::setupUVCCamera() {
  if (use_uvc_camera_) {
    auto color_camera_info = getColorCameraInfo();
    auto serial_number = getSerialNumber();
    uvc_camera_driver_ =
        std::make_shared<UVCCameraDriver>(node_, parameters_, color_camera_info, serial_number);
  } else {
    uvc_camera_driver_ = nullptr;
  }
}

void OBCameraNode::startStreams() {
  setupVideoMode();
  int color_width = width_[COLOR];
  int color_height = height_[COLOR];
  setImageRegistrationMode(depth_registration_);
  setDepthColorSync(color_depth_synchronization_);
  if (depth_registration_ || enable_colored_point_cloud_) {
    setDepthToColorResolution(color_width, color_height);
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index] && !stream_started_[stream_index]) {
      if (use_uvc_camera_ && stream_index == COLOR) {
        continue;
      }
      CHECK(stream_video_mode_.count(stream_index));
      auto video_mode = stream_video_mode_.at(stream_index);
      CHECK(streams_.count(stream_index));
      streams_[stream_index]->setVideoMode(video_mode);
      streams_[stream_index]->setMirroringEnabled(false);
      auto status = streams_[stream_index]->start();
      if (status != openni::STATUS_OK) {
        std::stringstream ss;
        ss << "Failed to start " << stream_name_[stream_index] << " stream. "
           << openni::OpenNI::getExtendedError();
        RCLCPP_ERROR_STREAM(logger_, ss.str());
        throw std::runtime_error(ss.str());
      }
      stream_started_[stream_index] = true;
      RCLCPP_INFO_STREAM(logger_, stream_name_[stream_index] << " is started");
    }
  }
  if (use_uvc_camera_) {
    CHECK_NOTNULL(uvc_camera_driver_);
    RCLCPP_INFO_STREAM(logger_, "Start UVC camera");
    uvc_camera_driver_->startStreaming();
  }
}

void OBCameraNode::setupConfig() {
  stream_name_[DEPTH] = "depth";
  unit_step_size_[DEPTH] = sizeof(uint16_t);
  format_[DEPTH] = openni::PIXEL_FORMAT_DEPTH_1_MM;
  image_format_[DEPTH] = CV_16UC1;
  encoding_[DEPTH] = sensor_msgs::image_encodings::TYPE_16UC1;

  stream_name_[COLOR] = "color";
  unit_step_size_[COLOR] = 3;
  format_[COLOR] = openni::PIXEL_FORMAT_RGB888;
  image_format_[COLOR] = CV_8UC3;
  encoding_[COLOR] = sensor_msgs::image_encodings::RGB8;

  stream_name_[INFRA1] = "ir";
  unit_step_size_[INFRA1] = sizeof(uint8_t);
  format_[INFRA1] = openni::PIXEL_FORMAT_GRAY8;
  image_format_[INFRA1] = CV_8UC1;
  encoding_[INFRA1] = sensor_msgs::image_encodings::MONO8;
  for (const auto& stream_index : IMAGE_STREAMS) {
    stream_started_[stream_index] = false;
  }
}

void OBCameraNode::getParameters() {
  setAndGetNodeParameter(parameters_, camera_name_, "camera_name", camera_name_);
  camera_link_frame_id_ = camera_name_ + "_link";
  for (const auto& stream_index : IMAGE_STREAMS) {
    frame_id_[stream_index] = camera_name_ + "_" + stream_name_[stream_index] + "_frame";
    optical_frame_id_[stream_index] =
        camera_name_ + "_" + stream_name_[stream_index] + "_optical_frame";
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    std::string param_name = stream_name_[stream_index] + "_width";
    setAndGetNodeParameter(parameters_, width_[stream_index], param_name, IMAGE_WIDTH);
    param_name = stream_name_[stream_index] + "_height";
    setAndGetNodeParameter(parameters_, height_[stream_index], param_name, IMAGE_HEIGHT);
    param_name = stream_name_[stream_index] + "_fps";
    setAndGetNodeParameter(parameters_, fps_[stream_index], param_name, IMAGE_FPS);
    param_name = "enable_" + stream_name_[stream_index];
    setAndGetNodeParameter(parameters_, enable_stream_[stream_index], param_name, false);
    param_name = stream_name_[stream_index] + "_frame_id";
    setAndGetNodeParameter(parameters_, frame_id_[stream_index], param_name,
                           frame_id_[stream_index]);
    param_name = stream_name_[stream_index] + "_optical_frame_id";
    setAndGetNodeParameter(parameters_, optical_frame_id_[stream_index], param_name,
                           optical_frame_id_[stream_index]);
    param_name = stream_name_[stream_index] + "_qos";
    setAndGetNodeParameter<std::string>(parameters_, image_qos_[stream_index], param_name,
                                        "default");
    param_name = stream_name_[stream_index] + "_camera_info_qos";
    setAndGetNodeParameter<std::string>(parameters_, camera_info_qos_[stream_index], param_name,
                                        "default");
  }
  for (const auto& stream_index : IMAGE_STREAMS) {
    depth_aligned_frame_id_[stream_index] = optical_frame_id_[COLOR];
  }
  setAndGetNodeParameter(parameters_, publish_tf_, "publish_tf", true);
  setAndGetNodeParameter(parameters_, tf_publish_rate_, "tf_publish_rate", 10.0);
  setAndGetNodeParameter(parameters_, camera_link_frame_id_, "camera_link_frame_id",
                         DEFAULT_BASE_FRAME_ID);
  setAndGetNodeParameter(parameters_, depth_registration_, "depth_registration", false);
  setAndGetNodeParameter(parameters_, color_depth_synchronization_, "color_depth_synchronization",
                         false);
  setAndGetNodeParameter(parameters_, color_roi_.x, "color_roi_x", -1);
  setAndGetNodeParameter(parameters_, color_roi_.y, "color_roi_y", -1);
  setAndGetNodeParameter(parameters_, color_roi_.width, "color_roi_width", -1);
  setAndGetNodeParameter(parameters_, color_roi_.height, "color_roi_height", -1);
  setAndGetNodeParameter(parameters_, depth_roi_.x, "depth_roi_x", -1);
  setAndGetNodeParameter(parameters_, depth_roi_.y, "depth_roi_y", -1);
  setAndGetNodeParameter(parameters_, depth_roi_.width, "depth_roi_width", -1);
  setAndGetNodeParameter(parameters_, depth_roi_.height, "depth_roi_height", -1);
  setAndGetNodeParameter(parameters_, depth_scale_, "depth_scale", 1);
  setAndGetNodeParameter(parameters_, enable_point_cloud_, "enable_point_cloud", true);
  setAndGetNodeParameter(parameters_, enable_colored_point_cloud_, "enable_colored_point_cloud",
                         false);
  setAndGetNodeParameter<std::string>(parameters_, point_cloud_qos_, "point_cloud_qos", "default");
  setAndGetNodeParameter(parameters_, enable_publish_extrinsic_, "enable_publish_extrinsic", false);
  setAndGetNodeParameter<std::string>(parameters_, ir_info_url_, "ir_info_url", "");
  setAndGetNodeParameter<std::string>(parameters_, color_info_url_, "color_info_url", "");
  if (enable_colored_point_cloud_) {
    depth_registration_ = true;
  }
}

void OBCameraNode::setupTopics() {
  getParameters();
  setupDevices();
  setupCameraCtrlServices();
  setupPublishers();
  setupVideoMode();
  getCameraParams();
  setupCameraInfoManager();
  setupUVCCamera();
  publishStaticTransforms();
}

void OBCameraNode::setupPublishers() {
  for (const auto& stream_index : IMAGE_STREAMS) {
    if (enable_stream_[stream_index]) {
      std::string name = stream_name_[stream_index];
      std::string topic = name + "/image_raw";
      auto image_qos = image_qos_[stream_index];
      auto image_qos_profile = getRMWQosProfileFromString(image_qos);
      image_publishers_[stream_index] = node_->create_publisher<sensor_msgs::msg::Image>(
          topic,
          rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(image_qos_profile), image_qos_profile));
      topic = name + "/camera_info";
      auto camera_info_qos = camera_info_qos_[stream_index];
      auto camera_info_qos_profile = getRMWQosProfileFromString(camera_info_qos);
      camera_info_publishers_[stream_index] = node_->create_publisher<CameraInfo>(
          topic, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(camera_info_qos_profile),
                             camera_info_qos_profile));
    }
  }
  if (enable_publish_extrinsic_) {
    extrinsics_publisher_ = node_->create_publisher<Extrinsics>("extrinsic/depth_to_color",
                                                                rclcpp::QoS{1}.transient_local());
  }
}

void OBCameraNode::publishStaticTF(const rclcpp::Time& t, const std::vector<float>& trans,
                                   const tf2::Quaternion& q, const std::string& from,
                                   const std::string& to) {
  CHECK_EQ(trans.size(), 3u);
  geometry_msgs::msg::TransformStamped msg;
  msg.header.stamp = t;
  msg.header.frame_id = from;
  msg.child_frame_id = to;
  msg.transform.translation.x = trans.at(2) / 1000.0;
  msg.transform.translation.y = -trans.at(0) / 1000.0;
  msg.transform.translation.z = -trans.at(1) / 1000.0;
  msg.transform.rotation.x = q.getX();
  msg.transform.rotation.y = q.getY();
  msg.transform.rotation.z = q.getZ();
  msg.transform.rotation.w = q.getW();
  static_tf_msgs_.push_back(msg);
}

void OBCameraNode::calcAndPublishStaticTransform() {
  tf2::Quaternion quaternion_optical, zero_rot;
  zero_rot.setRPY(0.0, 0.0, 0.0);
  quaternion_optical.setRPY(-M_PI / 2, 0.0, -M_PI / 2);
  std::vector<float> zero_trans = {0, 0, 0};

  auto rotation = camera_params_->r2l_r;
  auto transition = camera_params_->r2l_t;
  auto Q = rotationMatrixToQuaternion(rotation);
  for (int i = 0; i < 9; i++) {
    if (std::isnan(rotation[i])) {
      Q.setRPY(0, 0, 0);
      break;
    }
  }
  Q = quaternion_optical * Q * quaternion_optical.inverse();
  std::vector<float> trans = {transition[0], transition[1], transition[2]};
  if (std::isnan(transition[0]) || std::isnan(transition[1]) || std::isnan(transition[2])) {
    trans[0] = 0;
    trans[1] = 0;
    trans[2] = 0;
  }
  rclcpp::Time tf_timestamp = node_->now();

  publishStaticTF(tf_timestamp, trans, Q, frame_id_[DEPTH], frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, trans, Q, camera_link_frame_id_, frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[COLOR],
                  optical_frame_id_[COLOR]);
  publishStaticTF(tf_timestamp, zero_trans, quaternion_optical, frame_id_[DEPTH],
                  optical_frame_id_[DEPTH]);
  publishStaticTF(tf_timestamp, zero_trans, zero_rot, camera_link_frame_id_, frame_id_[DEPTH]);
  auto ex_msg = obExtrinsicsToMsg(rotation, transition, "depth_to_color_extrinsics");
  ex_msg.header.stamp = node_->now();
  if (enable_publish_extrinsic_ && extrinsics_publisher_) {
    extrinsics_publisher_->publish(ex_msg);
  }
}

void OBCameraNode::publishDynamicTransforms() {
  RCLCPP_WARN(logger_, "Publishing dynamic camera transforms (/tf) at %g Hz", tf_publish_rate_);
  static std::mutex mu;
  std::unique_lock<std::mutex> lock(mu);
  while (rclcpp::ok() && is_running_) {
    tf_cv_.wait_for(lock, std::chrono::milliseconds((int)(1000.0 / tf_publish_rate_)),
                    [this] { return (!(is_running_)); });
    {
      rclcpp::Time t = node_->now();
      for (auto& msg : static_tf_msgs_) {
        msg.header.stamp = t;
      }
      CHECK_NOTNULL(dynamic_tf_broadcaster_);
      dynamic_tf_broadcaster_->sendTransform(static_tf_msgs_);
    }
  }
}

void OBCameraNode::publishStaticTransforms() {
  if (!publish_tf_) {
    return;
  }
  static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(node_);
  dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);
  calcAndPublishStaticTransform();
  if (tf_publish_rate_ > 0) {
    tf_thread_ = std::make_shared<std::thread>([this]() { publishDynamicTransforms(); });
  } else {
    static_tf_broadcaster_->sendTransform(static_tf_msgs_);
  }
}

void OBCameraNode::setImageRegistrationMode(bool enable) {
  if (!device_->isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
    RCLCPP_WARN_STREAM(logger_, "Current do not support " << magic_enum::enum_name(
                                    openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR));
    return;
  }
  auto mode = enable ? openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR : openni::IMAGE_REGISTRATION_OFF;
  auto rc = device_->setImageRegistrationMode(mode);
  if (rc != openni::STATUS_OK) {
    RCLCPP_ERROR(logger_, "Enabling image registration mode failed: \n%s\n",
                 openni::OpenNI::getExtendedError());
  }
}

void OBCameraNode::onNewFrameCallback(const openni::VideoFrameRef& frame,
                                      const stream_index_pair& stream_index) {
  int width = frame.getWidth();
  int height = frame.getHeight();
  CHECK(images_.count(stream_index));
  auto& image = images_.at(stream_index);
  if (image.size() != cv::Size(width, height)) {
    image.create(height, width, image.type());
  }
  image.data = (uint8_t*)frame.getData();
  auto& camera_info_publisher = camera_info_publishers_.at(stream_index);
  auto& image_publisher = image_publishers_.at(stream_index);
  cv::Mat scaled_image;
  if (stream_index == DEPTH) {
    cv::resize(image, scaled_image, cv::Size(width * depth_scale_, height * depth_scale_), 0, 0,
               cv::INTER_NEAREST);
  }
  auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), encoding_.at(stream_index),
                                      stream_index == DEPTH ? scaled_image : image)
                       .toImageMsg();
  CHECK_NOTNULL(image_msg);
  auto timestamp = node_->now();
  image_msg->header.stamp = timestamp;
  image_msg->header.frame_id =
      depth_registration_ ? depth_aligned_frame_id_[stream_index] : optical_frame_id_[stream_index];
  image_msg->width = stream_index == DEPTH ? width * depth_scale_ : width;
  image_msg->height = stream_index == DEPTH ? height * depth_scale_ : height;
  image_msg->step = image_msg->width * unit_step_size_[stream_index];
  image_msg->is_bigendian = false;
  image_publisher->publish(*image_msg);
  sensor_msgs::msg::CameraInfo camera_info;
  if (stream_index == DEPTH) {
    camera_info = getDepthCameraInfo();
  } else if (stream_index == COLOR) {
    camera_info = getColorCameraInfo();
  } else if (stream_index == INFRA1 || stream_index == INFRA2) {
    double f = getFocalLength(stream_index, width);
    camera_info = getIRCameraInfo(width, height, f);
  }
  if (camera_info.width != static_cast<uint32_t>(image_msg->width) ||
      camera_info.height != static_cast<uint32_t>(image_msg->height)) {
    camera_info.width = image_msg->width;
    camera_info.height = image_msg->height;
  }
  camera_info.header.stamp = timestamp;
  camera_info.header.frame_id =
      depth_registration_ ? depth_aligned_frame_id_[stream_index] : optical_frame_id_[stream_index];

  camera_info_publisher->publish(camera_info);
}

void OBCameraNode::setDepthColorSync(bool data) {
  auto rc = device_->setDepthColorSyncEnabled(data);
  if (rc != openni::STATUS_OK) {
    RCLCPP_ERROR_STREAM(logger_, "Enabling depth color synchronization failed: "
                                     << openni::OpenNI::getExtendedError());
  }
}

void OBCameraNode::setDepthToColorResolution(int width, int height) {
  const auto pid = device_info_.getUsbProductId();
  if (pid != DABAI_DCW_DEPTH_PID && pid != DABAI_DW_PID) {
    return;
  }
  if (!depth_registration_) {
    return;
  }
  if (width * 9 == height * 16) {
    // 16:9
    auto status = device_->setProperty(XN_MODULE_PROPERTY_D2C_RESOLUTION, RGBResolution16_9);
    if (status != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_, "setProperty XN_MODULE_PROPERTY_D2C_RESOLUTION "
                                       << openni::OpenNI::getExtendedError());
    }
  } else if (width * 3 == height * 4) {
    // 4:3
    auto status = device_->setProperty(XN_MODULE_PROPERTY_D2C_RESOLUTION, RGBResolution4_3);
    if (status != openni::STATUS_OK) {
      RCLCPP_ERROR_STREAM(logger_, "setProperty XN_MODULE_PROPERTY_D2C_RESOLUTION "
                                       << openni::OpenNI::getExtendedError());
    }
  } else {
    RCLCPP_ERROR_STREAM(logger_, "NOT 16x9 or 4x3 resolution");
  }
}

}  // namespace astra_camera
