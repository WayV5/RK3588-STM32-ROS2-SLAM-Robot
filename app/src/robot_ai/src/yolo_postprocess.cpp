// yolo_postprocess.cpp — YOLOv8 detection decode + NMS
#include "robot_ai/yolo_postprocess.hpp"

#include <algorithm>
#include <cmath>

namespace robot_ai {

// Build grid cell positions for one feature map
static void make_grid(int w, int h, int base, float grid_out[]) {
  for (int gy = 0; gy < h; gy++) {
    for (int gx = 0; gx < w; gx++) {
      int offset = base + (gy * w + gx);
      grid_out[offset * 2 + 0] = static_cast<float>(gx);
      grid_out[offset * 2 + 1] = static_cast<float>(gy);
    }
  }
}

std::vector<Detection> yolo_postprocess(
    const float* output,
    int model_w, int model_h,
    int img_w, int img_h,
    float conf_threshold,
    float nms_threshold)
{
  constexpr int kClasses = 80;
  constexpr int kBoxes   = 8400;
  constexpr int kDims    = 4 + kClasses;

  // Build grid
  std::vector<float> grid(kBoxes * 2);
  const int hw[3][2] = {{80, 80}, {40, 40}, {20, 20}};
  int base = 0;
  for (int i = 0; i < 3; i++) {
    make_grid(hw[i][1], hw[i][0], base, grid.data());
    base += hw[i][0] * hw[i][1];
  }

  std::vector<Detection> detections;
  for (int i = 0; i < kBoxes; i++) {
    const float* row = output + i;  // column-major: output[box_idx + kBoxes * dim]

    // stride determined by feature map
    float stride;
    if      (i < 6400) stride = 8.0f;
    else if (i < 8000) stride = 16.0f;
    else               stride = 32.0f;

    // Decode box
    float cx = (row[kBoxes * 0] + grid[i * 2 + 0]) / static_cast<float>(model_w) * stride * model_w;
    float cy = (row[kBoxes * 1] + grid[i * 2 + 1]) / static_cast<float>(model_h) * stride * model_h;
    float w  = std::exp(row[kBoxes * 2]) * stride * static_cast<float>(model_w);
    float h  = std::exp(row[kBoxes * 3]) * stride * static_cast<float>(model_h);
    float x1 = cx - w / 2.0f;
    float y1 = cy - h / 2.0f;
    float x2 = cx + w / 2.0f;
    float y2 = cy + h / 2.0f;

    // Max class score
    float max_conf = 0.0f;
    int best_class = 0;
    for (int c = 0; c < kClasses; c++) {
      float score = row[kBoxes * (4 + c)];
      if (score > max_conf) { max_conf = score; best_class = c; }
    }
    if (max_conf < conf_threshold) continue;

    // Rescale to original image size
    Detection det;
    det.x1 = x1 * static_cast<float>(img_w) / static_cast<float>(model_w);
    det.y1 = y1 * static_cast<float>(img_h) / static_cast<float>(model_h);
    det.x2 = x2 * static_cast<float>(img_w) / static_cast<float>(model_w);
    det.y2 = y2 * static_cast<float>(img_h) / static_cast<float>(model_h);
    det.class_id   = best_class;
    det.confidence = max_conf;
    detections.push_back(det);
  }

  // NMS per-class
  std::vector<Detection> nms_result;
  for (int c = 0; c < kClasses; c++) {
    std::vector<int> idx;
    for (size_t i = 0; i < detections.size(); i++)
      if (detections[i].class_id == c) idx.push_back(static_cast<int>(i));
    if (idx.empty()) continue;

    std::sort(idx.begin(), idx.end(), [&](int a, int b) {
      return detections[a].confidence > detections[b].confidence;
    });

    while (!idx.empty()) {
      int best = idx[0];
      nms_result.push_back(detections[best]);
      std::vector<int> remaining;
      for (size_t j = 1; j < idx.size(); j++) {
        int other = idx[j];
        float xx1 = std::max(detections[best].x1, detections[other].x1);
        float yy1 = std::max(detections[best].y1, detections[other].y1);
        float xx2 = std::min(detections[best].x2, detections[other].x2);
        float yy2 = std::min(detections[best].y2, detections[other].y2);
        float iw  = std::max(0.0f, xx2 - xx1);
        float ih  = std::max(0.0f, yy2 - yy1);
        float iou = (iw * ih) / ((detections[best].x2 - detections[best].x1) *
                                 (detections[best].y2 - detections[best].y1) +
                                 (detections[other].x2 - detections[other].x1) *
                                 (detections[other].y2 - detections[other].y1) - iw * ih);
        if (iou <= nms_threshold) remaining.push_back(other);
      }
      idx = remaining;
    }
  }
  return nms_result;
}

}  // namespace robot_ai
