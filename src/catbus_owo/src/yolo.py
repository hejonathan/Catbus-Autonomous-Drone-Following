"""
pre-trained model retrieved from https://github.com/deepakcrk/yolov5-crowdhuman
"""

import rospy
from action_lib_msgs.msg import GoalID
from geometry_msgs.msg import Image

import math
import numpy as np
import time
import torch
import random
from pathlib import Path
import cv2

from yolo.models.experimental import attempt_load
from yolo.utils.datasets import LoadStreams, LoadImages
from yolo.utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from yolo.utils.plots import plot_one_box
from yolo.utils.torch_utils import select_device, load_classifier, time_synchronized

class Yolo():

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('driver', anonymous=True)

        # ROS Publishers
        self.displacement_pub = rospy.Publisher('displacement', GoalID, queue_size=1)
        self.goal_msg = GoalID() # TODO what kind of message hmmm

        # ROS Subscribers
        rospy.Subscriber('camera', Image, self.camera_callback)

        set_logging()

        # Load model
        # self.source = 'data/images'
        self.weights = 'yolov5s.pt'
        self.device = 'cpu'

        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check img_size

        # Second-stage classifier
        self.classify = False
        if self.classify:
            self.modelc = load_classifier(name='resnet101', n=2)  # initialize
            self.modelc.load_state_dict(torch.load('weights/resnet101.pt', map_location=self.device)['model']).to(self.device).eval()

         # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]
        self.t0 = time.time()

    def camera_callback(self, data):
        dataset = LoadImages(data, img_size=self.imgsz, stride=self.stride) # Loads images from camera subscriber
        for path, img, im0s, vid_cap in dataset: # TODO wut this mean cri
            img = torch.from_numpy(img).to(self.device)
            img = img.float()  # uint8 to fp16/32
            img /= 255.0  # 0 - 255 to 0.0 - 1.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)

            # Inference
            t1 = time_synchronized()
            pred = self.model(img, augment=True)[0]

            # Apply NMS
            pred = non_max_suppression(pred, 0.25, 0.45, classes=0, agnostic=True) # TODO weird params no comprendo
            t2 = time_synchronized()

            # Apply Classifier
            if self.classify:
                pred = apply_classifier(pred, self.modelc, img, im0s)

            for i, det in enumerate(pred):  # detections per image
                p, s, im0, frame = path, '', im0s, getattr(dataset, 'frame', 0)

                p = Path(p)  # to Path
                s += '%gx%g ' % img.shape[2:]  # print string

                if len(det):
                    # Rescale boxes from img_size to im0 size
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

                    # Print results
                    for c in det[:, -1].unique():
                        n = (det[:, -1] == c).sum()  # detections per class
                        s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

                    # Write results
                    for *xyxy, conf, cls in reversed(det):
                        # Find bbox
                        label = f'{self.names[int(cls)]} {conf:.2f}'
                        if 'head' in label:
                            c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                            wh = c2[0] - c1[0]
                            centroid = [c1[0] + wh / 2, c1[1] - wh / 2]
                            # TODO publish displacement nani nani
                            plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=3)
                        else:
                            pass

                # Print time (inference + NMS)
                print(f'{s}Done. ({t2 - t1:.3f}s)')

                # Stream video
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)


if __name__=='__main__':
    yolo = Yolo()
