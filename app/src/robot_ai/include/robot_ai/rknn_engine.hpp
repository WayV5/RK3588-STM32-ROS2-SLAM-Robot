// rknn_engine.hpp — RAII wrapper for RKNN C API
#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "robot_ai/rknn_api.h"

namespace robot_ai {

class RknnEngine {
public:
  RknnEngine() = default;
  ~RknnEngine();

  bool init(const std::string& model_path, rknn_core_mask core_mask = RKNN_NPU_CORE_AUTO);
  bool run(const uint8_t* img_data);

  int input_width()  const { return input_attr_.dims[2]; }
  int input_height() const { return input_attr_.dims[1]; }
  int num_outputs() const { return io_num_.n_output; }

  const rknn_output*      output_buf(int i) const { return &outputs_[i]; }
  const rknn_tensor_attr& output_attr(int i) const { return output_attrs_[i]; }
  bool is_quantized() const { return is_quant_; }

  void release();

private:
  rknn_context          ctx_ = 0;
  rknn_input            inputs_[1] = {};
  rknn_output           outputs_[9] = {};
  rknn_tensor_attr      output_attrs_[9] = {};
  rknn_tensor_attr      input_attr_ = {};
  rknn_input_output_num io_num_ = {};
  uint8_t*              input_buf_ = nullptr;
  bool initialized_ = false;
  bool is_quant_ = false;
};

}  // namespace robot_ai
