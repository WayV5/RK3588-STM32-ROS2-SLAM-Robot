// yolo_postprocess.hpp — YOLOv8 detection decode + NMS
#pragma once

#include <cstdint>
#include <vector>

namespace robot_ai {

struct Detection {
  float x1, y1, x2, y2;
  int   class_id;
  float confidence;
};

// Decode YOLOv8 output (1, 84, 8400) → detection list
// Rescales coords from model input size to original img_width x img_height
std::vector<Detection> yolo_postprocess(
    const float* output,       // 84*8400 flat float array
    int model_w, int model_h,  // model input size (640x640)
    int img_w, int img_h,      // original image size
    float conf_threshold = 0.5f,
    float nms_threshold  = 0.45f);

}  // namespace robot_ai
