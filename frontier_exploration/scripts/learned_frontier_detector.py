#! /usr/bin/env python3

'''
This node is for detecting frontier regions in an accupancy map with a learned model.
It serves requests from another node asking for the best frontier to travel to in the current map.
'''
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frontier_interfaces.srv import FrontierGoal

import torch
import torch.backends.cudnn as cudnn

import cv2 as cv
import numpy as np
 
class FrontierDetector(Node):

    def __init__(self):
        super().__init__('frontier_detector')
        self.srv = self.create_service(FrontierGoal,'frontier_pose', self.detect_callback)

        self.subscription = self.create_subscription(
            OccupancyGrid, 'map',
            self.map_callback, 2)
        self.subscription  # prevent unused variable warning

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.declare_parameter('weights_path', 'yolov5n_frontier_64.pt')
        weights_path = self.get_parameter('weights_path').get_parameter_value().string_value

        self.map = OccupancyGrid()

        cudnn.benchmark = True  # set True to speed up constant image size inference
        # self.model = torch.hub.load('ultralytics/yolov5', 'custom', path=weights_path)
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
        
        self.get_logger().info('Starting detector.')

    def map_callback(self, msg):
        self.get_logger().info('Got message.')
        self.map = msg

    @torch.no_grad()
    def detect_callback(self, request, response):

        self.get_logger().info('Got Request.')

        map = self.map

        map.info

        """
        Inference the model
        scale detections back to original size
        pick best frontier
        publish frontier markers
        optional publish image with bboxes annotated
        """

        # convert occupancy grid to grayscale image
        # img = np.ones((map.info.width,map.info.height), dtype=np.uint8)


        img = np.array(255 * (map.data != -1)).astype('uint8')
        img[map.data == -1] = 128
        # img[map.data >= 1] = 0

        # arr[arr > 255] = x

        # Scale image to (64,64)
        # img = cv.resize(img, (64, 64), interpolation=cv.INTER_AREA)
        # img = np.ascontiguousarray(img)
        # im = torch.from_numpy(im).to(self.device)

        # Inference
        pred = self.model(img)

        pred.print()

        # Tranform reults to map coordinates
        pix2meters = 1.0/map.info.resolution
        scaleX = map.info.width/64.0 * pix2meters
        scaleY = map.info.height/64.0 * pix2meters

        # det = pred[0]
        # if det is not None and len(det):


        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0

        self.get_logger().info("Req: %d. Returned x: %f y: %f .".format(request.goal_rank,
            goal.pose.position.x, goal.pose.position.y))
        response.goal_pose = goal
        return response
    
    def get_current_pose(self) -> PoseStamped:
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform odom to base_link: {ex}')
            return None
            
        p = PoseStamped()
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.header.stamp = self.navigator.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        return p

def main(args=None):
    rclpy.init(args=args)
    frontier_detector = FrontierDetector() 
    rclpy.spin(frontier_detector)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()