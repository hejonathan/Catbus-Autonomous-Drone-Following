#!/usr/bin/env python3
"""
Implements YOLOv5 to detect the head of a person in the camera frame
Pre-trained model retrieved from https://github.com/deepakcrk/yolov5-crowdhuman
"""

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import Image

import math
import numpy as np
import time
import torch
import random
from pathlib import Path
import cv2
from cv_bridge import CvBridge

from models.experimental import attempt_load
from utils.datasets import LoadStreams, LoadImages, letterbox
from utils.general import check_img_size, check_requirements, check_imshow, non_max_suppression, apply_classifier, \
    scale_coords, xyxy2xywh, strip_optimizer, set_logging, increment_path
from utils.plots import plot_one_box
from utils.torch_utils import select_device, load_classifier, time_synchronized

camera_matrix = np.array([[921.170702, 0.000000, 459.904354], [0.000000, 919.018377, 351.238301], [0.000000, 0.000000, 1.000000]])
dist_coeffs= np.array([-0.033458, 0.105152, 0.001256, -0.006647, 0.000000])

RESCALE = .3
IMG_SZ = (int(960*RESCALE), int(720*RESCALE))

class Yolo():

    def __init__(self):
        # Initialize ROS node
        rospy.init_node('yolo', anonymous=True)

        # ROS Publishers
        self.displacement_pub = rospy.Publisher('displacement', Point, queue_size=5)
        self.goal_msg = Point() # TODO what kind of message hmmm

        # ROS Subscribers
        rospy.Subscriber('vid', Image, self.camera_callback)

        set_logging()

        # Load model
        # self.source = 'data/images'
        self.weights = '/media/newhd/catbus/crowdhuman_yolov5m.pt'
        self.device = 'cpu'

        self.model = attempt_load(self.weights, map_location=self.device)  # load FP32 model
        self.stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(IMG_SZ, s=self.stride)  # check img_size

        # self.model.warmup(imgsz=(1,3,*self.imgsz))

         # Get names and colors
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names
        self.colors = [[random.randint(0, 255) for _ in range(3)] for _ in self.names]

        rospy.loginfo(f'model loaded. Classes:{self.names}')

    def camera_callback(self, data):
        if not hasattr(self, 'imgsz'):
            return
        frame = CvBridge().imgmsg_to_cv2(data)
        frame = cv2.resize(frame, IMG_SZ, interpolation=cv2.INTER_AREA)
        im0 = frame.copy()
        img = letterbox(frame, self.imgsz, stride=self.stride)[0]
        print(img.shape)
        img = np.swapaxes(img, 0, 2)
        img = np.swapaxes(img, 1, 2)
        img = torch.from_numpy(img).to(self.device)
        img = img.float()  # uint8 to fp16/32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0
        if img.ndimension() == 3:
            img = img.unsqueeze(0)

        # Inference
        t1 = time_synchronized()
        pred = self.model(img, augment=True)[0]

        # Apply NMS
        pred = non_max_suppression(pred, 0.25, 0.45, classes=None, agnostic=True) # TODO weird params no comprendo
        t2 = time_synchronized()

        det = pred[0]
        s = '%gx%g ' % img.shape[2:]  # print string

        if len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

            # Print results
            for c in det[:, -1].unique():
                n = (det[:, -1] == c).sum()  # detections per class
                s += f"{n} {self.names[int(c)]}{'s' * (n > 1)}, "  # add to string

            # Write results
            for *xyxy, conf, cls in [det[0]]:
                print(conf)
                # Find bbox
                label = f'{self.names[int(cls)]} {conf:.2f}'
                if 'head' in label:
                    # Calculate displacement and draw bbox
                    # xyxy /= RESCALE
                    c1, c2 = (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3]))
                    wh = c2[0] - c1[0]
                    d1 = [c1[0], c1[1] - wh / 2]
                    d2 = [c2[0], c1[1] - wh / 2]
                    d3 = [c2[0], c2[1] + wh / 2]
                    d4 = [c1[0], c2[1] + wh /2]
                    # centroid = [c1[0] + 3 * wh / 4, c1[1] - wh / 2]
                    _ , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(np.array([[d1, d2, d3, d4]]), 12, camera_matrix, dist_coeffs)
                    tvec = tvec[0][0]
                    self.goal_msg.x = tvec[0]
                    self.goal_msg.z = tvec[1] # TODO I'm not sure if this is correct :/
                    self.goal_msg.y = tvec[2]
                    print(tvec)
                    self.displacement_pub.publish(self.goal_msg)
                    plot_one_box(xyxy, im0, label=label, color=self.colors[int(cls)], line_thickness=5)

            # Print time (inference + NMS)
            print(f'{s}Done. ({t2 - t1:.3f}s)')

            # Stream video
            cv2.imshow('vid', im0)
            cv2.waitKey(1)


if __name__=='__main__':
    yolo = Yolo()
    rospy.spin()