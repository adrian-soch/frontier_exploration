#! /usr/bin/env python3

import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn

from learned_frontier_detector.models.common import DetectMultiBackend
from learned_frontier_detector.utils.general import (non_max_suppression, scale_boxes)
from learned_frontier_detector.utils.torch_utils import time_sync, select_device

DEBUG = True

class FrontierDetector():
    """Create a frontier detector that loads a custom trained yolov5 DNN
        that predicts frontier regions from occupancy maps converted to images
    """
    def __init__(self, weights, imgsz=(640,640), conf_thresh=0.6, iou_thres=0.4, max_det=30, device='cpu'):

        yolo_weights= weights   # Path to the network weights
        self.imgsz=imgsz  # inference size (height, width)
        self.conf_thres=conf_thresh  # confidence threshold
        self.iou_thres=iou_thres  # NMS IOU threshold
        self.max_det=max_det  # maximum detections per image
        self.device=device  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.half=False  # use FP16 half-precision inference (GPU ONLY)
        self.classes = [0]  # only 1 class (frontier)

        # Load model
        self.device = select_device(self.device)
        self.model = DetectMultiBackend(yolo_weights, device=self.device, data=None, fp16=self.half)
        cudnn.benchmark = True  # set True to speed up constant image size inference
    
    @torch.no_grad()
    def update(self, im):

        im0 = im.copy()
        im0 = cv2.cvtColor(im0, cv2.COLOR_GRAY2RGB)

        # Scale image to (64,64)
        im = cv2.resize(im, (64, 64), interpolation=cv2.INTER_AREA)
        im = cv2.cvtColor(im, cv2.COLOR_GRAY2RGB) # Comvert to 3 channel for yolov5

        # Prepare for inference
        im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
        im = np.ascontiguousarray(im)
        im = torch.from_numpy(im).to(self.device)
        im = im.half() if self.half else im.float()  # uint8 to fp16/32
        im /= 255.0  # 0 - 255 to 0.0 - 1.0
        if len(im.shape) == 3:
            im = im[None]  # expand for batch dim

        # Inference
        pred = self.model(im, augment=False, visualize=False)

        # Apply NMS
        pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, agnostic=False, max_det=self.max_det)

        # Process detections
        det = pred[0]
        if det is not None and len(det):
            # Rescale boxes from img_size to im0 size
            det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()  # xyxy

        if DEBUG:
            # Draw bounding boxes on image and save to disk
            d = det.numpy()
            for i in range(d.shape[0]):
                data = d[i]
                start = (np.int32((data[0], data[1])))
                end = (np.int32((data[2], data[3])))
                im0 = cv2.rectangle(im0, start, end, (0,255,0), 2)
            cv2.imwrite("/workspace/src/result.png", im0)

        return det

def main():

    image = cv2.imread("/workspace/08_04_2023_15_58_00/0_504_779_0.030000.png")
    fd = FrontierDetector()
    fd.update(image)
    
if __name__ == '__main__':
    main()
