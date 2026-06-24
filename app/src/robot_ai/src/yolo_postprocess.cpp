// yolo_postprocess.cpp — YOLOv8 detection decode + DFL + NMS
// Matches rknn_model_zoo-2.3.0 optimized ONNX output format
#include "robot_ai/yolo_postprocess.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <set>
#include <vector>

namespace robot_ai {

static constexpr int kClasses  = 80;
static constexpr int kMaxBoxes = 128;

// ── DFL (Distribution Focal Loss) on CPU ────────────────────────
static void compute_dfl(const float* tensor, int dfl_len, float box[4]) {
  for (int b = 0; b < 4; b++) {
    float exp_sum = 0.0f;
    float acc_sum = 0.0f;
    for (int i = 0; i < dfl_len; i++) {
      float e = std::exp(tensor[i + b * dfl_len]);
      exp_sum += e;
      acc_sum += e * static_cast<float>(i);
    }
    box[b] = acc_sum / exp_sum;
  }
}

static inline int clamp_i(float v, int lo, int hi) {
  int iv = static_cast<int>(v);
  return iv < lo ? lo : (iv > hi ? hi : iv);
}

// ── Process one branch (one scale) ───────────────────────────────
static int process_branch_fp32(
    const float* box_tensor,     // [1, 64, H, W]  NCHW
    const float* score_tensor,   // [1, 80, H, W]  NCHW
    const float* score_sum,      // [1, 1,  H, W]  NCHW  (nullptr if only 2 oups/branch)
    int grid_h, int grid_w,
    int stride, int dfl_len,
    float conf_threshold,
    std::vector<float>& boxes,
    std::vector<float>& scores,
    std::vector<int>&   class_ids)
{
  int valid = 0;
  int grid_len = grid_h * grid_w;

  for (int gy = 0; gy < grid_h; gy++) {
    for (int gx = 0; gx < grid_w; gx++) {
      int offset = gy * grid_w + gx;

      // Quick reject via score_sum
      if (score_sum && score_sum[offset] < conf_threshold)
        continue;

      // Find max class
      float max_score = 0.0f;
      int   max_cls   = -1;
      for (int c = 0; c < kClasses; c++) {
        float s = score_tensor[offset + c * grid_len];
        if (s > conf_threshold && s > max_score) {
          max_score = s;
          max_cls   = c;
        }
      }
      if (max_cls < 0) continue;

      // DFL + decode box
      float before_dfl[64];  // 4 * dfl_len = 64
      for (int k = 0; k < 4 * dfl_len; k++)
        before_dfl[k] = box_tensor[offset + k * grid_len];

      float box[4];
      compute_dfl(before_dfl, dfl_len, box);

      float x1 = (-box[0] + static_cast<float>(gx) + 0.5f) * static_cast<float>(stride);
      float y1 = (-box[1] + static_cast<float>(gy) + 0.5f) * static_cast<float>(stride);
      float x2 = ( box[2] + static_cast<float>(gx) + 0.5f) * static_cast<float>(stride);
      float y2 = ( box[3] + static_cast<float>(gy) + 0.5f) * static_cast<float>(stride);

      boxes.push_back(x1);
      boxes.push_back(y1);
      boxes.push_back(x2 - x1);
      boxes.push_back(y2 - y1);
      scores.push_back(max_score);
      class_ids.push_back(max_cls);
      valid++;
    }
  }
  return valid;
}

// ── IoU for NMS ──────────────────────────────────────────────────
static float calc_iou(const float* a, const float* b) {
  float xx1 = std::max(a[0], b[0]);
  float yy1 = std::max(a[1], b[1]);
  float xx2 = std::min(a[0] + a[2], b[0] + b[2]);
  float yy2 = std::min(a[1] + a[3], b[1] + b[3]);
  float iw  = std::max(0.0f, xx2 - xx1);
  float ih  = std::max(0.0f, yy2 - yy1);
  float i   = iw * ih;
  float u   = a[2] * a[3] + b[2] * b[3] - i;
  return (u > 0.0f) ? i / u : 0.0f;
}

// ── Per-class NMS with score sorting ─────────────────────────────
static void nms_per_class(const std::vector<float>& boxes,
                          const std::vector<float>& scores,
                          const std::vector<int>&   class_ids,
                          int total,
                          float nms_threshold,
                          std::vector<Detection>& out) {
  std::vector<int> idx(total);
  for (int i = 0; i < total; i++) idx[i] = i;
  std::sort(idx.begin(), idx.end(), [&](int a, int b) {
    return scores[a] > scores[b];
  });

  std::vector<bool> suppressed(total, false);

  for (int i = 0; i < total; i++) {
    int p = idx[i];
    if (suppressed[p]) continue;

    out.push_back({boxes[p*4+0], boxes[p*4+1], boxes[p*4+0] + boxes[p*4+2],
                   boxes[p*4+1] + boxes[p*4+3], class_ids[p], scores[p]});

    for (int j = i + 1; j < total; j++) {
      int q = idx[j];
      if (suppressed[q]) continue;
      if (class_ids[q] != class_ids[p]) continue;
      if (calc_iou(&boxes[p * 4], &boxes[q * 4]) > nms_threshold)
        suppressed[q] = true;
    }
  }
}

// ── Public entry point ────────────────────────────────────────────
std::vector<Detection> yolo_postprocess(
    float* const* output_bufs,
    const int*    output_dims,
    bool          is_quant,
    const int32_t* deq_zp,
    const float*   deq_scale,
    int model_w, int model_h,
    int img_w, int img_h,
    float conf_threshold,
    float nms_threshold)
{
  (void)is_quant;
  (void)deq_zp;
  (void)deq_scale;

  const int n_output    = 9;
  const int n_branches  = 3;
  const int out_per_br  = n_output / n_branches;

  // dfl_len from first box tensor: dims[1] = 64 → 64/4 = 16
  int dfl_len = output_dims[0 * 4 + 1] / 4;

  std::vector<float> all_boxes, all_scores;
  std::vector<int>   all_classes;

  for (int b = 0; b < n_branches; b++) {
    int box_idx   = b * out_per_br;
    int score_idx = b * out_per_br + 1;
    int sum_idx   = b * out_per_br + 2;

    int grid_h = output_dims[box_idx * 4 + 2];
    int grid_w = output_dims[box_idx * 4 + 3];
    int stride = model_h / grid_h;

    float* box_tensor   = output_bufs[box_idx];
    float* score_tensor = output_bufs[score_idx];
    float* sum_tensor   = (out_per_br >= 3) ? output_bufs[sum_idx] : nullptr;

    process_branch_fp32(box_tensor, score_tensor, sum_tensor,
                        grid_h, grid_w, stride, dfl_len, conf_threshold,
                        all_boxes, all_scores, all_classes);
  }

  int total = static_cast<int>(all_classes.size());
  std::vector<Detection> results;

  if (total > 0) {
    nms_per_class(all_boxes, all_scores, all_classes, total, nms_threshold, results);

    float sx = static_cast<float>(img_w) / static_cast<float>(model_w);
    float sy = static_cast<float>(img_h) / static_cast<float>(model_h);
    for (auto& d : results) {
      d.x1 *= sx; d.y1 *= sy;
      d.x2 *= sx; d.y2 *= sy;
    }
  }
  return results;
}

}  // namespace robot_ai
