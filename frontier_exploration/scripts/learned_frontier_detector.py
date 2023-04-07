#! /usr/bin/env python3

'''
This node is for detecting frontier regions in an accupancy map with a learned model.
It serves requests from another node asking for the best frontier to travel to in the current map.
'''
 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frontier_interfaces.srv import FrontierGoal
 
class FrontierDetector(Node):

    def __init__(self):
        super().__init__('frontier_detector')
        self.srv = self.create_service(FrontierGoal,'frontier_pose', self.detect_callback)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Starting detector.')


    def detect_callback(self, request, response):

        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = 'odom'
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0

        self.get_logger().info("Req: %d. Returned x: %f y: %f .".format(request.goal_rank, goal.pose.position.x, goal.pose.position.y))
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