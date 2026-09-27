"""YOLO (Ultralytics export) on TensorRT without torch/ultralytics: TensorRT + cuda-python + numpy + OpenCV.

The system Python on the Jetson cannot import torch/ultralytics (numpy 2 in ~/.local), but TensorRT and
cuda-python work, and they are all inference needs. The .engine from `yolo export format=engine`
starts with a 4-byte length + JSON metadata, then the serialized engine.
"""
from __future__ import annotations

import json
import struct
from dataclasses import dataclass

import cv2
import numpy as np

from .detection import letterbox, postprocess_yolo


@dataclass(frozen=True)
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    score: float
    cls: int


def _cuda():
    try:
        from cuda.bindings import runtime as cudart      # cuda-python ≥ 12.6
    except ImportError:
        from cuda import cudart                          # older cuda-python
    return cudart


def _check(res):
    err = res[0]
    if int(err) != 0:
        raise RuntimeError(f"CUDA error {err}")
    return res[1] if len(res) > 1 else None


class YoloTrt:
    def __init__(self, engine_path: str, conf: float = 0.4, iou: float = 0.5, classes: tuple[int, ...] = (0,)) -> None:
        import tensorrt as trt

        self.cudart = _cuda()
        self.conf, self.iou, self.classes = conf, iou, classes
        with open(engine_path, "rb") as f:
            blob = f.read()
        meta_len = struct.unpack("<i", blob[:4])[0]
        self.meta: dict = {}
        if 0 < meta_len < 1_000_000:
            try:
                self.meta = json.loads(blob[4:4 + meta_len].decode())
                blob = blob[4 + meta_len:]
            except (UnicodeDecodeError, json.JSONDecodeError):
                self.meta = {}                               # plain TensorRT engine without metadata
        self.imgsz = int((self.meta.get("imgsz") or [640])[0])
        logger = trt.Logger(trt.Logger.WARNING)
        self.engine = trt.Runtime(logger).deserialize_cuda_engine(blob)
        if self.engine is None:
            raise RuntimeError(f"cannot deserialize {engine_path} (TensorRT version mismatch?)")
        self.context = self.engine.create_execution_context()
        self.stream = _check(self.cudart.cudaStreamCreate())
        self.bufs: dict[str, tuple[np.ndarray, int]] = {}
        self.input_name = self.output_name = ""
        for i in range(self.engine.num_io_tensors):
            name = self.engine.get_tensor_name(i)
            shape = tuple(self.engine.get_tensor_shape(name))
            dtype = np.dtype(trt.nptype(self.engine.get_tensor_dtype(name)))
            host = np.empty(shape, dtype=dtype)
            dev = _check(self.cudart.cudaMalloc(host.nbytes))
            self.bufs[name] = (host, dev)
            self.context.set_tensor_address(name, dev)
            if self.engine.get_tensor_mode(name) == trt.TensorIOMode.INPUT:
                self.input_name = name
            else:
                self.output_name = name

    def __call__(self, bgr: np.ndarray) -> list[Detection]:
        cudart = self.cudart
        inp_host, inp_dev = self.bufs[self.input_name]
        out_host, out_dev = self.bufs[self.output_name]
        img, scale, pad = letterbox(bgr, self.imgsz)
        blob = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)[None].astype(inp_host.dtype) / np.asarray(255, inp_host.dtype)
        np.copyto(inp_host, blob)
        kind_h2d, kind_d2h = cudart.cudaMemcpyKind.cudaMemcpyHostToDevice, cudart.cudaMemcpyKind.cudaMemcpyDeviceToHost
        _check(cudart.cudaMemcpyAsync(inp_dev, inp_host.ctypes.data, inp_host.nbytes, kind_h2d, self.stream))
        if not self.context.execute_async_v3(self.stream):
            raise RuntimeError("TensorRT inference failed")
        _check(cudart.cudaMemcpyAsync(out_host.ctypes.data, out_dev, out_host.nbytes, kind_d2h, self.stream))
        _check(cudart.cudaStreamSynchronize(self.stream))
        boxes = postprocess_yolo(out_host[0].astype(np.float32), scale, pad, bgr.shape[:2], self.conf, self.iou, self.classes)
        return [Detection(*b) for b in boxes]

    def close(self) -> None:
        for _, dev in self.bufs.values():
            self.cudart.cudaFree(dev)
        self.bufs.clear()
        self.cudart.cudaStreamDestroy(self.stream)
