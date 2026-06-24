// yolo_postprocess.hpp — YOLOv8 detection decode (optimized ONNX, DFL on CPU)
#pragma once

#include <cstdint>
#include <vector>

namespace robot_ai {

struct Detection {
  float x1, y1, x2, y2;
  int   class_id;
  float confidence;
};

// Post-process optimized YOLOv8 RKNN output (9 tensors: 3 scales × {box, class, score_sum})
//
// output_bufs: array of 9 float* pointers to per-tensor FP32 data
// output_dims: per-tensor dimensions (output_dims[i*4+0..3] = n,c,h,w)
// is_quant:   true if INT8 quantized (uses zp/scale for dequant), false if FP32
// deq_zp:     zero-point array (per output), only used when is_quant
// deq_scale:  scale array (per output), only used when is_quant
// model_w/h:  model input size (640)
// img_w/h:    original image size
//
std::vector<Detection> yolo_postprocess(
    float* const* output_bufs,
    const int*    output_dims,    // [n_output * 4]: n,c,h,w per tensor
    bool          is_quant,
    const int32_t* deq_zp,
    const float*   deq_scale,
    int model_w, int model_h,
    int img_w, int img_h,
    float conf_threshold = 0.5f,
    float nms_threshold  = 0.45f);

}  // namespace robot_ai
