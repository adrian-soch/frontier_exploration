#! /usr/bin/env python3

'''
This node is for autonomous exploration. It requests frontier regions from a service and
sends the goal points to the nav2 stack until the the entire environment has been explored.

Reference code: https://automaticaddison.com/how-to-send-goals-to-the-ros-2-navigation-stack-nav2/
'''
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frontier_interfaces.srv import FrontierGoal
from frontier_exploration.robot_navigator import BasicNavigator, NavigationResult
 
class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        self.cli = self.create_client(FrontierGoal, 'frontier_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = FrontierGoal.Request()
        self.navigator = BasicNavigator()

        self.EXPLORATION_TIME_OUT_SEC = Duration(seconds=1200)
        self.NAV_TO_GOAL_TIMEOUT_SEC = 75
        self.DIST_THRESH_FOR_HEADING_CALC = 0.25

        self.goal_pose = PoseStamped()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.start_time = self.get_clock().now()
        self.get_logger().info('Starting frontier exploration...')
        self.explore()

    def explore(self):

        while self.start_time - self.get_clock().now() < self.EXPLORATION_TIME_OUT_SEC:

            # Delay getting next goal so map updates
            prev_time = self.get_clock().now()
            while self.get_clock().now() - prev_time < Duration(seconds=0.3):
                pass

            # Get a frontier we can drive to
            self.goal_pose = self.get_reachable_goal()

            # If we cant find a path to any frontiers
            if self.goal_pose is None:
                self.get_logger().error('No reachable frontiers!')
                exit(-1)
            elif self.goal_pose == "Done":
                self.get_logger().info('Exploration complete! No frontiers detected.')
                exit(0)
            
            # Go to the goal pose
            self.navigator.goToPose(self.goal_pose)
            
            # Keep doing stuff as long as the robot is moving towards the goal
            i = 0
            while not self.navigator.isNavComplete():

                i = i + 1
                feedback = self.navigator.getFeedback()
                if feedback and i % 40 == 0:
                    self.get_logger().info('Distance remaining: ' + '{:.2f}'.format(
                        feedback.distance_remaining) + ' meters.')
                
                    # Set goal pose heading to robots current heading once it is close]
                    # This avoids unecessary rotation once reaching the goal position
                    if feedback.distance_remaining <= self.DIST_THRESH_FOR_HEADING_CALC:
                        self.get_logger().info('Setting new heading')
                        self.set_goal_heading()
                        self.navigator.goToPose(self.goal_pose)
            
                # Cancel the goal if robot takes too long
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=self.NAV_TO_GOAL_TIMEOUT_SEC):
                    self.navigator.cancelNav()
            
            # Print result when nav completes
            result = self.navigator.getResult()
            self.log_nav_status(result)

    def get_reachable_goal(self):
        rank = 0
        reachable = False
        while not reachable:
            goal = self.send_request(rank)
            if goal is None:
                return "Done"

            self.goal_pose = goal
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            # sanity check a valid path exists
            initial_pose = self.get_current_pose()
            if initial_pose is None:
                # Return goal is current pose is unavailble
                return goal
            path = self.navigator.getPath(initial_pose, self.goal_pose)

            # If top 4 frontiers are not reachable, abort
            if path is not None:
                return goal
            elif rank > 3:
                return None
            rank += 1

    def send_request(self, rank):
        self.req.goal_rank = rank
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        res = self.future.result()
        return res.goal_pose

    def get_current_pose(self) -> PoseStamped:
        try:
            t = self.tf_buffer.lookup_transform(
                "odom",
                "base_link",
                rclpy.time.Time(), Duration(seconds=0.5))
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform odom to base_link: {ex}')
            self.get_logger().warn('Current pose unavailable.')
            return None
            
        p = PoseStamped()
        p.pose.position.x = t.transform.translation.x
        p.pose.position.y = t.transform.translation.y
        p.header.stamp = self.navigator.get_clock().now().to_msg()
        p.header.frame_id = 'odom'
        return p
    
    def set_goal_heading(self):
        curr_pose = self.get_current_pose()

        if curr_pose is None:
            return
        
        # Set goal orientation to current heading
        self.goal_pose.pose.orientation.x = curr_pose.pose.orientation.x
        self.goal_pose.pose.orientation.y = curr_pose.pose.orientation.y
        self.goal_pose.pose.orientation.z = curr_pose.pose.orientation.z
        self.goal_pose.pose.orientation.w = curr_pose.pose.orientation.w
    
    def log_nav_status(self, result):
        if result == NavigationResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif result == NavigationResult.CANCELED:
            self.get_logger().info('Goal was canceled!')
        elif result == NavigationResult.FAILED:
            self.get_logger().info('Goal failed!')
        else:
            self.get_logger().error('Goal has an invalid return status!')

def main(args=None):
    rclpy.init(args=args)
    frontier_explorer = FrontierExplorer()   
    frontier_explorer.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()