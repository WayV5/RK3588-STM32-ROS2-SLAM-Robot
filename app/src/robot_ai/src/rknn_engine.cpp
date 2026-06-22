// rknn_engine.cpp — RAII wrapper for RKNN C API
#include "robot_ai/rknn_engine.hpp"

#include <cstdio>
#include <cstring>

namespace robot_ai {

RknnEngine::~RknnEngine() { release(); }

bool RknnEngine::init(const std::string& model_path, int core_mask) {
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

  memset(inputs_, 0, sizeof(inputs_));
  inputs_[0].index = 0;
  inputs_[0].fmt = RKNN_TENSOR_NHWC;
  inputs_[0].type = RKNN_TENSOR_UINT8;
  inputs_[0].size = input_attr_.size;

  memset(outputs_, 0, sizeof(outputs_));
  for (int i = 0; i < io_num_.n_output; i++) {
    outputs_[i].want_float = 1;
  }

  printf("[rknn_engine] Init OK: %dx%d, %d inputs, %d outputs\n",
         input_width(), input_height(), io_num_.n_input, io_num_.n_output);
  initialized_ = true;
  return true;
}

bool RknnEngine::run(const uint8_t* img_data, std::vector<float>& results) {
  if (!initialized_) return false;

  inputs_[0].buf = const_cast<uint8_t*>(img_data);
  int ret = rknn_inputs_set(ctx_, 1, inputs_);
  if (ret < 0) { fprintf(stderr, "[rknn_engine] inputs_set failed: %d\n", ret); return false; }

  ret = rknn_run(ctx_, nullptr);
  if (ret < 0) { fprintf(stderr, "[rknn_engine] run failed: %d\n", ret); return false; }

  ret = rknn_outputs_get(ctx_, io_num_.n_output, outputs_, nullptr);
  if (ret < 0) { fprintf(stderr, "[rknn_engine] outputs_get failed: %d\n", ret); return false; }

  for (int i = 0; i < io_num_.n_output; i++) {
    rknn_tensor_attr attr;
    memset(&attr, 0, sizeof(attr));
    attr.index = i;
    rknn_query(ctx_, RKNN_QUERY_OUTPUT_ATTR, &attr, sizeof(attr));
    int n_elems = 1;
    for (int d = 0; d < attr.n_dims; d++) n_elems *= attr.dims[d];
    float* out_ptr = reinterpret_cast<float*>(outputs_[i].buf);
    results.insert(results.end(), out_ptr, out_ptr + n_elems);
  }

  rknn_outputs_release(ctx_, io_num_.n_output, outputs_);
  return true;
}

void RknnEngine::release() {
  if (ctx_ != 0) { rknn_destroy(ctx_); ctx_ = 0; }
  initialized_ = false;
}

}  // namespace robot_ai
