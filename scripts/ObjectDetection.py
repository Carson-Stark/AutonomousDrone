

# Lint as: python3
# Copyright 2019 Google LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
r"""Example using PyCoral to detect objects in a given image.

To run this code, you must attach an Edge TPU attached to the host and
install the Edge TPU runtime (`libedgetpu.so`) and `tflite_runtime`. For
device setup instructions, see coral.ai/docs/setup.

Example usage:
```
bash examples/install_requirements.sh detect_image.py

python3 examples/detect_image.py \
  --model test_data/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite \
  --labels test_data/coco_labels.txt \
```
"""

import argparse
import time
import os

from PIL import Image
from PIL import ImageDraw

import cv2

import tflite_runtime.interpreter as tflite
from pycoral.adapters import common
from pycoral.adapters import detect
from pycoral.utils.dataset import read_label_file
from pycoral.utils.edgetpu import make_interpreter
from pycoral.adapters.common import input_size
from pycoral.utils.edgetpu import run_inference
import rospkg

import numpy as np

def append_objs_to_img(cv2_im, inference_size, objs, labels):
    height, width, channels = cv2_im.shape
    scale_x, scale_y = width / inference_size[1], height / inference_size[0]
    for obj in objs:

        bbox = obj.bbox.scale(scale_x, scale_y)
        x0, y0 = int(bbox.xmin), int(bbox.ymin)
        x1, y1 = int(bbox.xmax), int(bbox.ymax)

        percent = int(100 * obj.score)
        label = '{}% {}'.format(percent, labels.get(obj.id, obj.id))

        cv2_im = cv2.rectangle(cv2_im, (x0, y0), (x1, y1), (0, 255, 0), 2)
        cv2_im = cv2.putText(cv2_im, label, (x0, y0+30),
                             cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 0, 0), 2)
    return cv2_im

script_dir = os.path.dirname(__file__) #<-- absolute dir the script is in
labels_file = os.path.join(script_dir, "bag_labels.txt")
model = os.path.join(script_dir, "efficientdet-lite-plastic_bag2_edgetpu.tflite")


labels = read_label_file(labels_file) if labels_file else {}
interpreter = make_interpreter(model)

interpreter.allocate_tensors()
inference_size = input_size(interpreter)

last_obj = None

def find_bag(frame):
    start = time.perf_counter()
    _, scale = common.set_resized_input(interpreter, (frame.shape[1], frame.shape[0]), lambda size: cv2.resize(frame, size))
    interpreter.invoke()
    objs = detect.get_objects(interpreter, 0.3, scale)
    inference_time = time.perf_counter() - start
    best_obj = None
    if len(objs) > 0:
        for obj in objs:
            if best_obj == None or obj.score > best_obj.score:
                best_obj = obj

    """print(best_obj)

    result = append_objs_to_img(frame, frame.shape, objs, labels)
    cv2.imshow("detection", result)
    cv2.waitKey(1)"""

    if best_obj == None:
        return None

    return (best_obj.bbox.xmin, best_obj.bbox.xmax, best_obj.bbox.ymin, best_obj.bbox.ymax)

"""cap = cv2.VideoCapture("/home/greta/Desktop/FlightRecordings/recording2024-01-07 17:42:01.250807.mp4")

while(cap.isOpened()):
    # Capture frame-by-frame
    ret, frame = cap.read()
    if ret == True:
        find_bag(frame)"""