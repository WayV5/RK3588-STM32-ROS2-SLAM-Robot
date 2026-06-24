#!/usr/bin/env python3
"""Convert YOLOv8n ONNX → RKNN for RK3588 NPU.

Requires rknn_model_zoo-2.3.0 optimized ONNX (3-branch output format).
Use model zoo's download_model.sh to get the ONNX first.

Strategy:
  1. FP16 (no calibration data) — simple, fast
  2. INT8 later (need COCO subset for calibration)

Usage:
    cd tools/npu
    python3 convert_yolov8_to_rknn.py --onnx yolov8n.onnx --output yolov8n_fp16_rk3588.rknn
"""

import argparse
import os
import sys
import numpy as np

from rknn.api import RKNN

MODEL_DIR = "../../app/src/robot_ai/model"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--int8", action="store_true",
                   help="INT8 quantized (needs calibration dataset)")
    p.add_argument("--dataset", default=None,
                   help="Calibration dataset file for INT8")
    p.add_argument("--onnx", default="yolov8n.onnx")
    p.add_argument("--output", default=None)
    return p.parse_args()


def main():
    args = parse_args()
    onnx_path = args.onnx
    if args.output:
        rknn_path = args.output
    else:
        tag = "int8" if args.int8 else "fp16"
        rknn_path = f"yolov8n_{tag}_rk3588.rknn"

    if not os.path.exists(onnx_path):
        sys.exit(f"ONNX not found: {onnx_path}")

    rknn = RKNN(verbose=True)

    # ── Step 1: Config ────────────────────────────────────────
    print(f"\n[1/5] Configuring RKNN (target=rk3588)")
    rknn.config(
        mean_values=[[0, 0, 0]],
        std_values=[[255, 255, 255]],
        target_platform="rk3588",
    )

    # ── Step 2: Load ONNX ────────────────────────────────────
    print(f"\n[2/5] Loading ONNX: {onnx_path}")
    ret = rknn.load_onnx(model=onnx_path)
    if ret != 0:
        sys.exit(f"load_onnx failed: {ret}")

    # ── Step 3: Build RKNN ───────────────────────────────────
    quant = "INT8" if args.int8 else "FP16"
    print(f"\n[3/5] Building RKNN ({quant})")
    ret = rknn.build(
        do_quantization=args.int8,
        dataset=args.dataset if args.int8 else None,
    )
    if ret != 0:
        sys.exit(f"build failed: {ret}")

    # ── Step 4: Export ───────────────────────────────────────
    print(f"\n[4/5] Exporting RKNN → {rknn_path}")
    ret = rknn.export_rknn(rknn_path)
    if ret != 0:
        sys.exit(f"export_rknn failed: {ret}")
    size_kb = os.path.getsize(rknn_path) / 1024
    print(f"  Size: {size_kb:.1f} KB")

    # ── Step 5: Sanity check (PC sim) ────────────────────────
    print("\n[5/5] Sanity check — PC simulation inference")
    ret = rknn.init_runtime()
    if ret != 0:
        sys.exit(f"init_runtime failed: {ret}")

    dummy = np.random.randint(0, 256, (1, 640, 640, 3), dtype=np.uint8)
    outputs = rknn.inference(inputs=[dummy])
    print(f"  Input:  {dummy.shape} {dummy.dtype}")
    for i, o in enumerate(outputs):
        print(f"  Output[{i}]: {o.shape} {o.dtype}")

    rknn.release()
    print(f"\nDone → {rknn_path}")
    print(f"Next: cp {rknn_path} {MODEL_DIR}/")
    print(f"      then deploy to RK3588 with v2.3.0 librknnrt.so")


if __name__ == "__main__":
    main()
