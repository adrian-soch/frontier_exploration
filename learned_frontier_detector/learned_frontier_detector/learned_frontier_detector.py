#! /usr/bin/env python3

'''
This node is for detecting frontier regions in an accupancy map with a learned model.
It serves requests from another node asking for the best frontier to travel to in the current map.
'''
import numpy as np
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frontier_interfaces.srv import FrontierGoal
from ament_index_python.packages import get_package_share_directory

from dataclasses import dataclass

from learned_frontier_detector.detector import FrontierDetector

@dataclass
class Frontier:
    size: int
    x: float
    y: float
    score: float = 0.0
    
 
class FrontierDetectorNode(Node):

    def __init__(self):
        super().__init__('frontier_detector_node')
        self.srv = self.create_service(FrontierGoal,'frontier_pose', self.detect_callback)

        self.subscription = self.create_subscription(
            OccupancyGrid, 'map',
            self.map_callback, 2)
        self.subscription  # prevent unused variable warning

        self.marker_pub = self.create_publisher(Marker, 'f_marker', 2)

        # TF for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = OccupancyGrid()

        # Get path to model weights
        weights = get_package_share_directory('learned_frontier_detector') + \
            '/weights/yolov5n_frontier_64.pt'
        
        # Create detector
        self.detector = FrontierDetector(weights=weights, imgsz=(64,64),
                conf_thresh=0.6, iou_thres=0.4,max_det=30)

        self.get_logger().info('Starting detector.')

    def map_callback(self, msg):
        self.get_logger().info('Got message.')
        self.map = msg

    def detect_callback(self, request, response):

        self.get_logger().info('Got Request.')

        # Copy map to local variable
        map = self.map

        # convert occupancy grid to grayscale image
        img = np.array(255 * (map.data != -1)).astype('uint8')
        img[map.data == -1] = 128

        # Inference - [xmin, ymin, xmax, ymax, confidence, class]
        pred = self.detector.update(img)[0]
        print(pred)

        frontiers = self.pred2regions(pred)
        self.marker_pub(frontiers)

        # Select best frontier
        goal_px = self.select_frontier(frontiers)

        # Transform reults to map coordinates
        goal_m = self.px2meters(goal_px, map.info.resolution, map.info.origin)

        goal_m.header.stamp = self.get_clock().now().to_msg()
        goal_m.header.frame_id = 'odom'

        self.get_logger().info("Req: %d. Returned x: %f y: %f .".format(request.goal_rank,
            goal_m.pose.position.x, goal_m.pose.position.y))
        response.goal_pose = goal_m
        return response
    
    def select_frontier(self, dets):
        print(dets[0])
    
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
    
    def publishMarkers(self, regions):
        spheres = Marker()
        spheres.header.frame_id = 'map'
        spheres.header.stamp = self.get_clock().now().to_msg()
        spheres.type = Marker.SPHERE_LIST
        spheres.action = Marker.ADD
        spheres.scale.x = 0.1
        spheres.scale.y = 0.1
        spheres.scale.z = 0.1
        spheres.color.a = 1.0
        spheres.color.g = 1.0
        for r in regions:
            p = Point()
            p.x = r.x
            p.y = r.y
            p.z = 0.05
            spheres.points.append(p)
        
        self.marker_pub.publish(spheres)

    def pred2regions(self, pred):
        

    def px2meters(self, resolution, origin) -> PoseStamped:
        pix2meters = 1.0/map.info.resolution



def main(args=None):
    rclpy.init(args=args)
    frontier_detector_node = FrontierDetectorNode() 
    rclpy.spin(frontier_detector_node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()