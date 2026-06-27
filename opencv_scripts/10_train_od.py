"""
Trains an object-detection model. The dataset should be in the od/ directory.

GPU acceleration is supported on MPS (on ARM Macs) as well as with NVIDIA GPUs with the device= parameter.
"""

import ultralytics

model = ultralytics.YOLO("yolo26n.pt")
model.train(data="od/data.yaml", imgsz=320, epochs=50)