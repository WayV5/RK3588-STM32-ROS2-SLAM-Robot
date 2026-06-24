#!/usr/bin/env python3
"""YOLOv8n CPU vs NPU benchmark — onnxruntime CPU inference on RK3588.

Requires: pip3 install onnxruntime
Usage:  python3 bench_cpu.py
"""
import time
import numpy as np
import onnxruntime as ort

MODEL = "/home/ww/code/RK3588-STM32-ROS2-SLAM-Robot/sdk/rk3588_sdk/external/rknn_model_zoo-2.3.0/examples/yolov8/model/yolov8n.onnx"

def main():
    print(f"Loading: {MODEL}")
    sess = ort.InferenceSession(MODEL, providers=['CPUExecutionProvider'])
    input_name = sess.get_inputs()[0].name
    input_shape = sess.get_inputs()[0].shape
    print(f"  Input: {input_name} {input_shape}")

    # Warmup
    dummy = np.random.randint(0, 256, (1, 3, 640, 640), dtype=np.float32) / 255.0
    for _ in range(3):
        sess.run(None, {input_name: dummy})

    # Benchmark
    n_runs = 20
    times = []
    for _ in range(n_runs):
        t0 = time.perf_counter()
        sess.run(None, {input_name: dummy})
        times.append(time.perf_counter() - t0)

    avg = np.mean(times) * 1000
    std = np.std(times) * 1000
    fps = 1000.0 / avg
    print(f"\nCPU (onnxruntime): {avg:.0f} ms +-{std:.0f} ms  -> {fps:.1f} FPS")
    print(f"NPU (RKNN FP16):   ~110 ms                -> ~9.0 FPS")
    print(f"\nNPU speedup: {avg/110:.1f}x")

if __name__ == "__main__":
    main()
