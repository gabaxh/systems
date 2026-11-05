#!/usr/bin/env python3
"""Stationary test: Filmer -> existing YOLO service -> browser.
Start Filmer and yolo_service.py, then run this in yolo-env.
Close direct Filmer viewers. Open http://127.0.0.1:5001 on Artitrax.
This experiment does not control the vehicle. Restart it after a camera failure.
"""

import base64
import threading
import time

import requests
from flask import Flask, Response

SOURCE = "http://127.0.0.1:20162/filmer/FrontCamera/stream"
YOLO = "http://127.0.0.1:5000/detect"
app = Flask(__name__)
frame_lock = threading.Lock()
detection_lock = threading.Lock()
latest = {"jpeg": None, "received": 0, "error": "Waiting for Filmer"}


def read_camera():
    # Keep reading while YOLO is busy. Replace the previous frame instead of
    # putting frames in a queue, which would make detection fall behind.
    buffer = b""
    try:
        with requests.get(SOURCE, stream=True, timeout=(3, 5)) as stream:
            stream.raise_for_status()
            if "multipart/x-mixed-replace" not in stream.headers.get("Content-Type", ""):
                raise ValueError("Filmer did not return an MJPEG stream")
            for chunk in stream.iter_content(chunk_size=4096):
                buffer += chunk
                if len(buffer) > 8 * 1024 * 1024:
                    raise ValueError("Camera frame is too large")
                while True:
                    start = buffer.find(b"\xff\xd8")  # JPEG start marker
                    if start < 0:
                        buffer = buffer[-1:]
                        break
                    buffer = buffer[start:]
                    end = buffer.find(b"\xff\xd9", 2)  # JPEG end marker
                    if end < 0:
                        break
                    jpeg, buffer = buffer[:end + 2], buffer[end + 2:]
                    with frame_lock:
                        latest.update(jpeg=jpeg, received=time.monotonic(), error="")
            raise ConnectionError("Filmer stream ended")
    except Exception as error:
        with frame_lock:
            latest.update(jpeg=None, error=str(error))
        print(f"Camera stopped: {error}. Restart the prototype after fixing it.", flush=True)


@app.get("/")
def index():
    # Request the next detection only after the previous request has finished.
    return '''<!doctype html><title>Artitrax detection test</title>
<h2>Stationary detection test</h2><p id="status">Waiting...</p><img id="image" hidden>
<script>
let previousImage;
async function update() {
  const image = document.getElementById('image');
  const status = document.getElementById('status');
  try {
    const response = await fetch('/detect.jpg', {cache: 'no-store'});
    if (!response.ok) throw new Error(await response.text());
    const nextImage = URL.createObjectURL(await response.blob());
    image.src = nextImage; image.hidden = false;
    if (previousImage) URL.revokeObjectURL(previousImage);
    previousImage = nextImage;
    status.textContent = 'YOLO request: ' + response.headers.get('X-YOLO-ms') + ' ms';
  } catch (error) { image.hidden = true; status.textContent = error.message; }
  setTimeout(update, 0);
}
update();
</script>'''


@app.get("/detect.jpg")
def detect():
    # Only one inference request at a time, even if another page is opened.
    with detection_lock:
        with frame_lock:
            jpeg, received, error = latest["jpeg"], latest["received"], latest["error"]
        if error or jpeg is None or time.monotonic() - received > 3:
            return error or "No recent camera frame", 503
        try:
            started = time.monotonic()
            response = requests.post(YOLO, files={"image": ("frame.jpg", jpeg, "image/jpeg")}, timeout=15)
            response.raise_for_status()
            result = response.json()
            annotated = base64.b64decode(result["annotated"], validate=True)
            elapsed = (time.monotonic() - started) * 1000
            if time.monotonic() - received > 3:
                return "Detection took too long; result is over 3 seconds old", 503
            print(f"YOLO request: {elapsed:.0f} ms; labels: {result['labels']}", flush=True)
            return Response(annotated, mimetype="image/jpeg", headers={
                "Cache-Control": "no-store", "X-YOLO-ms": f"{elapsed:.0f}"})
        except Exception as error:
            return f"Detection failed: {error}", 503


if __name__ == "__main__":
    threading.Thread(target=read_camera, daemon=True).start()
    app.run(host="127.0.0.1", port=5001, threaded=True, use_reloader=False)
