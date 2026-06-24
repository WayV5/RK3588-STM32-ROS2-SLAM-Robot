// rknn_engine.cpp — RAII wrapper for RKNN C API
#include "robot_ai/rknn_engine.hpp"

#include <cstdio>
#include <cstring>

namespace robot_ai {

RknnEngine::~RknnEngine() { release(); }

bool RknnEngine::init(const std::string& model_path, rknn_core_mask core_mask) {
  FILE* fp = fopen(model_path.c_str(), "rb");
  if (!fp) {
    fprintf(stderr, "[rknn_engine] Cannot open model: %s\n", model_path.c_str());
    return false;
  }
  fseek(fp, 0, SEEK_END);
  size_t model_size = ftell(fp);
  rewind(fp);
  auto* model_data = new uint8_t[model_size];
  if (fread(model_data, 1, model_size, fp) != model_size) {
    fclose(fp);
    delete[] model_data;
    return false;
  }
  fclose(fp);

  int ret = rknn_init(&ctx_, model_data, model_size, 0, nullptr);
  delete[] model_data;
  if (ret < 0) {
    fprintf(stderr, "[rknn_engine] rknn_init failed: %d\n", ret);
    return false;
  }

  rknn_set_core_mask(ctx_, core_mask);

  ret = rknn_query(ctx_, RKNN_QUERY_IN_OUT_NUM, &io_num_, sizeof(io_num_));
  if (ret < 0) {
    fprintf(stderr, "[rknn_engine] query IN_OUT_NUM failed\n");
    return false;
  }

  memset(&input_attr_, 0, sizeof(input_attr_));
  input_attr_.index = 0;
  ret = rknn_query(ctx_, RKNN_QUERY_INPUT_ATTR, &input_attr_, sizeof(input_attr_));
  if (ret < 0) {
    fprintf(stderr, "[rknn_engine] query INPUT_ATTR failed\n");
    return false;
  }

  // Detect quantization by checking input type
  is_quant_ = (input_attr_.type == RKNN_TENSOR_UINT8 || input_attr_.type == RKNN_TENSOR_INT8);

  fprintf(stderr, "[rknn_engine] Input attr: fmt=%d type=%d n_dims=%d dims=[%d,%d,%d,%d] size=%u zp=%d scale=%f\n",
         input_attr_.fmt, input_attr_.type, input_attr_.n_dims,
         input_attr_.dims[0], input_attr_.dims[1], input_attr_.dims[2], input_attr_.dims[3],
         input_attr_.size, input_attr_.zp, input_attr_.scale);

  // Always send UINT8 NHWC input — NPU handles internal float16 conversion
  // Reading input_attr_ gives native tensor type (FLOAT16 for fp models),
  // but rknn_inputs_set expects UINT8 and converts via mean/std automatically
  memset(inputs_, 0, sizeof(inputs_));
  inputs_[0].index = 0;
  inputs_[0].fmt = RKNN_TENSOR_NHWC;
  inputs_[0].type = RKNN_TENSOR_UINT8;
  inputs_[0].size = input_attr_.dims[0] * input_attr_.dims[1] * input_attr_.dims[2] * input_attr_.dims[3];

  // Query all output attributes and pre-allocate want_float for FP32 output
  memset(output_attrs_, 0, sizeof(output_attrs_));
  memset(outputs_, 0, sizeof(outputs_));
  for (int i = 0; i < io_num_.n_output; i++) {
    output_attrs_[i].index = i;
    ret = rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &output_attrs_[i], sizeof(output_attrs_[i]));
    if (ret < 0) {
      fprintf(stderr, "[rknn_engine] query OUTPUT_ATTR[%d] failed\n", i);
      return false;
    }
    outputs_[i].want_float = 1;  // always get FP32 output
  }

  // Allocate aligned input buffer for NHWC uint8 (cv::Mat data may not be aligned)
  input_buf_ = new uint8_t[inputs_[0].size];
  if (!input_buf_) {
    fprintf(stderr, "[rknn_engine] alloc input buffer failed\n");
    return false;
  }

  printf("[rknn_engine] Init OK: %dx%d, %d inputs, %d outputs (quant=%d)\n",
         input_width(), input_height(), io_num_.n_input, io_num_.n_output, is_quant_ ? 1 : 0);
  initialized_ = true;
  return true;
}

bool RknnEngine::run(const uint8_t* img_data) {
  if (!initialized_) return false;

  // Free previous output buffers to avoid memory leak (each output_get allocates new)
  rknn_outputs_release(ctx_, io_num_.n_output, outputs_);

  // Copy to aligned buffer (NPU DMA requires proper alignment)
  memcpy(input_buf_, img_data, inputs_[0].size);
  inputs_[0].buf = input_buf_;
  int ret = rknn_inputs_set(ctx_, 1, inputs_);
  if (ret < 0) { fprintf(stderr, "[rknn_engine] inputs_set failed: %d\n", ret); return false; }

  ret = rknn_run(ctx_, nullptr);
  if (ret < 0) { fprintf(stderr, "[rknn_engine] run failed: %d\n", ret); return false; }

  ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs_, nullptr);
  if (ret < 0) { fprintf(stderr, "[rknn_engine] outputs_get failed: %d\n", ret); return false; }

  return true;
}

void RknnEngine::release() {
  if (input_buf_) { delete[] input_buf_; input_buf_ = nullptr; }
  // outputs released in each run(), just in case:
  if (initialized_ && ctx_ != 0) {
    rknn_outputs_release(ctx_, io_num_.n_output, outputs_);
  }
  if (ctx_ != 0) { rknn_destroy(ctx_); ctx_ = 0; }
  initialized_ = false;
}

}  // namespace robot_ai
