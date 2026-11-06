#!/usr/bin/env python3
"""
YOLO detection microservice for the Arrowhead Recognizer system.

Run as a standalone process (independently of the Go binary):
    python3 yolo_service.py [--model yolov8n.pt] [--device auto] [--port 5000]

API
---
POST /detect
    Request : multipart/form-data
              field 'image' — JPEG image bytes (required)
              field 'model' — model filename to override the default (optional)
    Response: 200 OK, application/json
              {"labels": ["person", "chair"], "annotated": "<base64-encoded JPEG>"}
    Errors  : 400 if the image field is missing or the image cannot be decoded
              500 on internal detection failure

GET /health
    Response: 200 OK, application/json
              {"status": "ok", "model": "<active model name>",
               "device": "cuda:0", "backend": "cuda",
               "accelerator": "<device name>"}
"""

import argparse
import base64
import sys

import cv2
import numpy as np
import torch
from flask import Flask, jsonify, request
from ultralytics import YOLO

app = Flask(__name__)
_model = None        # loaded at startup
_model_name = "yolov8n.pt"
_device = torch.device("cpu")
_backend = "cpu"
_accelerator_name = "CPU"
MINIMUM_DETECTION_CONFIDENCE = 0.85


def resolve_device(requested):
    """Resolve an automatic, CPU, CUDA, or ROCm-backed CUDA device."""
    requested = requested.strip().lower()
    if requested == "auto":
        requested = "cuda:0" if torch.cuda.is_available() else "cpu"

    try:
        device = torch.device(requested)
    except (RuntimeError, ValueError) as exc:
        raise ValueError(f"invalid device {requested!r}") from exc

    if device.type == "cpu":
        return device, "cpu", "CPU"
    if device.type != "cuda":
        raise ValueError("device must be 'auto', 'cpu', 'cuda', or 'cuda:<index>'")
    if not torch.cuda.is_available():
        raise RuntimeError(
            f"device {requested!r} was requested, but no CUDA/ROCm GPU is available"
        )

    index = device.index if device.index is not None else 0
    device_count = torch.cuda.device_count()
    if index >= device_count:
        raise ValueError(
            f"GPU index {index} is unavailable; found {device_count} device(s)"
        )

    device = torch.device(f"cuda:{index}")
    backend = "rocm" if torch.version.hip is not None else "cuda"
    return device, backend, torch.cuda.get_device_name(device)


def detect_objects(model, image):
    """Return only detections whose confidence score is at least 85%."""
    return model.predict(
        source=image,
        conf=MINIMUM_DETECTION_CONFIDENCE,
        device=str(_device),
        verbose=False,
    )


@app.get("/health")
def health():
    return jsonify({
        "status": "ok",
        "model": _model_name,
        "device": str(_device),
        "backend": _backend,
        "accelerator": _accelerator_name,
    })


@app.post("/detect")
def detect():
    if "image" not in request.files:
        return jsonify({"error": "missing 'image' field in multipart body"}), 400

    image_bytes = request.files["image"].read()
    override = request.form.get("model")

    # Select model: use per-request override or fall back to the default.
    model = _model
    if override and override != _model_name:
        model = YOLO(override)   # Ultralytics caches weights in ~/.config/Ultralytics/
        model.to(_device)

    # Decode image bytes → OpenCV array.
    arr = np.frombuffer(image_bytes, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        return jsonify({"error": "could not decode image — expected a valid JPEG"}), 400

    # Run inference and reject detections below the minimum confidence score.
    results = detect_objects(model, img)

    # Collect unique class labels in detection order.
    labels = [results[0].names[int(cls)] for cls in results[0].boxes.cls.tolist()]

    # Render bounding boxes on a copy of the image.
    annotated_bgr = results[0].plot()
    ok, buf = cv2.imencode(".jpg", annotated_bgr)
    if not ok:
        return jsonify({"error": "failed to encode annotated image"}), 500

    annotated_b64 = base64.b64encode(buf.tobytes()).decode()
    return jsonify({"labels": labels, "annotated": annotated_b64})


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="YOLO detection microservice")
    parser.add_argument("--model", default="yolov8n.pt",
                        help="YOLOv8 model file (default: yolov8n.pt)")
    parser.add_argument("--device", default="auto",
                        help="inference device: auto, cpu, cuda, or cuda:<index> (default: auto)")
    parser.add_argument("--port",  type=int, default=5000,
                        help="port to listen on (default: 5000)")
    parser.add_argument("--host",  default="127.0.0.1",
                        help="interface to bind (default: 127.0.0.1)")
    args = parser.parse_args()

    try:
        _device, _backend, _accelerator_name = resolve_device(args.device)
    except (ValueError, RuntimeError) as exc:
        parser.error(str(exc))

    _model_name = args.model
    print(
        f"Loading {_model_name} on {_backend} device {_device} "
        f"({_accelerator_name}) …",
        file=sys.stderr,
        flush=True,
    )
    _model = YOLO(_model_name)
    _model.to(_device)
    print(f"Model ready. Listening on {args.host}:{args.port}", file=sys.stderr, flush=True)

    # threaded=True allows the Go binary to call /health while a detection is running.
    app.run(host=args.host, port=args.port, threaded=True)
