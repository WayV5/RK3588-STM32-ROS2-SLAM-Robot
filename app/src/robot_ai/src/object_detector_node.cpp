// object_detector_node.cpp — /camera/color/image_raw → NPU → /detections
#include <chrono>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

#include "robot_ai/rknn_engine.hpp"
#include "robot_ai/yolo_postprocess.hpp"

namespace {

const char* kCocoNames[80] = {
  "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
  "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
  "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
  "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
  "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
  "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
  "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
  "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
  "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
  "toothbrush"
};

}  // namespace

class ObjectDetectorNode : public rclcpp::Node {
public:
  explicit ObjectDetectorNode(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
    : rclcpp::Node("object_detector", opts)
  {
    model_path_  = declare_parameter("model_path", "model/yolov8n_fp16.rknn");
    conf_thresh_ = declare_parameter("conf_threshold", 0.5f);
    nms_thresh_  = declare_parameter("nms_threshold", 0.45f);
    pub_annot_   = declare_parameter("publish_annotated_image", true);

    RCLCPP_INFO(get_logger(), "Loading RKNN model: %s", model_path_.c_str());
    if (!engine_.init(model_path_)) {
      RCLCPP_ERROR(get_logger(), "Failed to init RKNN engine");
      return;
    }
    model_w_ = engine_.input_width();
    model_h_ = engine_.input_height();
    RCLCPP_INFO(get_logger(), "Model: %dx%d, %d outputs, quant=%d",
                model_w_, model_h_, engine_.num_outputs(),
                engine_.is_quantized() ? 1 : 0);

    sub_img_ = create_subscription<sensor_msgs::msg::Image>(
      "/camera/color/image_raw", rclcpp::SensorDataQoS(),
      std::bind(&ObjectDetectorNode::on_image, this, std::placeholders::_1));

    pub_dets_ = create_publisher<vision_msgs::msg::Detection2DArray>(
      "/detections", 10);

    if (pub_annot_)
      pub_annot_img_ = create_publisher<sensor_msgs::msg::Image>("/detection_image", 10);

    RCLCPP_INFO(get_logger(), "Ready");
  }

private:
  void on_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
    cv::Mat frame;
    try {
      frame = cv_bridge::toCvCopy(msg, "rgb8")->image;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what());
      return;
    }

    // Letterbox to model size (pad black for rga compatibility)
    float scale = std::min(static_cast<float>(model_w_) / frame.cols,
                           static_cast<float>(model_h_) / frame.rows);
    cv::Mat resized;
    cv::resize(frame, resized, cv::Size(), scale, scale, cv::INTER_LINEAR);
    int dw = model_w_ - resized.cols;
    int dh = model_h_ - resized.rows;
    cv::Mat letterbox;
    cv::copyMakeBorder(resized, letterbox, dh / 2, dh - dh / 2, dw / 2, dw - dw / 2,
                       cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

    // Inference
    auto t0 = std::chrono::steady_clock::now();
    if (!engine_.run(letterbox.data)) {
      RCLCPP_ERROR(get_logger(), "Inference failed");
      return;
    }
    double ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - t0).count();

    // Build per-output buffer pointers + dims for postprocess
    int n_out = engine_.num_outputs();
    std::vector<float*> bufs(n_out);
    std::vector<int>    dims(n_out * 4);
    for (int i = 0; i < n_out; i++) {
      bufs[i] = reinterpret_cast<float*>(engine_.output_buf(i)->buf);
      const auto& attr = engine_.output_attr(i);
      dims[i * 4 + 0] = attr.dims[0];
      dims[i * 4 + 1] = attr.dims[1];
      dims[i * 4 + 2] = attr.dims[2];
      dims[i * 4 + 3] = attr.dims[3];
    }

    auto dets = robot_ai::yolo_postprocess(
        bufs.data(), dims.data(),
        engine_.is_quantized(), nullptr, nullptr,
        model_w_, model_h_, frame.cols, frame.rows,
        conf_thresh_, nms_thresh_);

    // Publish Detection2DArray
    vision_msgs::msg::Detection2DArray arr;
    arr.header = msg->header;
    arr.header.frame_id = "camera_color_frame";
    for (const auto& d : dets) {
      vision_msgs::msg::Detection2D det2d;
      det2d.header = arr.header;
      det2d.bbox.center.position.x = (d.x1 + d.x2) / 2.0;
      det2d.bbox.center.position.y = (d.y1 + d.y2) / 2.0;
      det2d.bbox.size_x = d.x2 - d.x1;
      det2d.bbox.size_y = d.y2 - d.y1;

      vision_msgs::msg::ObjectHypothesisWithPose hyp;
      hyp.hypothesis.class_id = std::to_string(d.class_id);
      hyp.hypothesis.score = d.confidence;
      det2d.results.push_back(hyp);
      arr.detections.push_back(det2d);
    }
    pub_dets_->publish(arr);

    // Annotated image (only if someone subscribed)
    if (pub_annot_ && pub_annot_img_->get_subscription_count() > 0) {
      for (const auto& d : dets) {
        cv::rectangle(frame, cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1)),
                      cv::Point(static_cast<int>(d.x2), static_cast<int>(d.y2)),
                      cv::Scalar(0, 255, 0), 2);
        const char* label = (d.class_id < 80) ? kCocoNames[d.class_id] : "???";
        char buf[64];
        snprintf(buf, sizeof(buf), "%s %.2f", label, d.confidence);
        cv::putText(frame, buf, cv::Point(static_cast<int>(d.x1), static_cast<int>(d.y1) - 5),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 1);
      }
      auto anno = cv_bridge::CvImage(msg->header, "rgb8", frame).toImageMsg();
      pub_annot_img_->publish(*anno);
    }

    RCLCPP_INFO(get_logger(), "%zu dets, %.1f ms", dets.size(), ms);
  }

  robot_ai::RknnEngine engine_;
  int model_w_ = 640, model_h_ = 640;
  float conf_thresh_ = 0.5f, nms_thresh_ = 0.45f;
  std::string model_path_;
  bool pub_annot_ = true;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_img_;
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_dets_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_annot_img_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  fprintf(stderr, "--- creating node ---\n");
  auto node = std::make_shared<ObjectDetectorNode>();
  fprintf(stderr, "--- node created, spinning ---\n");
  rclcpp::spin(node);
  fprintf(stderr, "--- spin done ---\n");
  rclcpp::shutdown();
  return 0;
}
