#! /usr/bin/env python3

import cv2
import numpy as np
import torch
import torch.backends.cudnn as cudnn

from learned_frontier_detector.models.common import DetectMultiBackend
from learned_frontier_detector.utils.augmentations import letterbox
from learned_frontier_detector.utils.general import (non_max_suppression, scale_boxes)
from learned_frontier_detector.utils.torch_utils import time_sync, select_device

from ament_index_python.packages import get_package_share_directory

class FrontierDetector():

    def __init__(self):

        share_path = get_package_share_directory('learned_frontier_detector')

        yolo_weights= str(share_path) + '/weights/yolov5n_frontier_64.pt'  # model.pt path(s)
        self.imgsz=(64, 64)  # inference size (height, width)
        self.conf_thres=0.6  # confidence threshold
        self.iou_thres=0.45  # NMS IOU threshold
        self.max_det=30  # maximum detections per image
        self.device='cpu'  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        self.half=False  # use FP16 half-precision inference (GPU ONLY)
        self.classes = [0]

        # Load model
        self.device = select_device(self.device)
        self.model = DetectMultiBackend(yolo_weights, device=self.device, data=None, fp16=self.half)
        cudnn.benchmark = True  # set True to speed up constant image size inference
    
    @torch.no_grad()
    def update(self, im):

        t1 = time_sync()

        im0 = im.copy()
        im = letterbox(im, self.imgsz, stride=32, auto=True)[0]
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
            print(det)
        t2 = time_sync()

        print(t2-t1)

def main():

    image = cv2.imread("/workspace/08_04_2023_15_58_00/0_504_779_0.030000.png")

    fd = FrontierDetector()
    fd.update(image)
    
if __name__ == '__main__':
    main()
