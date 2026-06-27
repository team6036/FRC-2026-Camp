"""
Part (w) of pull-up shooting.
Requires a Roboflow classification dataset exported to a folder named "data"
in the same directory as this script. The dataset should contain two classes: "Shooting" and "Not Shooting".
Trains the classification YOLO model on the dataset for 20 epochs.
The trained model will be saved to the "runs" folder.

GPU acceleration is supported on MPS (on ARM Macs) as well as with NVIDIA GPUs with the device= parameter.
"""

import ultralytics

model = ultralytics.YOLO("yolo26n-cls.pt")
model.train(data="data/", epochs=20)
