#! /usr/bin/env python3

#Ref: https://automaticaddison.com/how-to-send-goals-to-the-ros-2-navigation-stack-nav2/
 
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import PoseStamped

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from frontier_interfaces.srv import FrontierGoal
from robot_navigator import BasicNavigator, NavigationResult
 
class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        self.cli = self.create_client(FrontierGoal, 'frontier_pose')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        self.req = FrontierGoal.Request()
        self.navigator = BasicNavigator()

        self.EXPLORE_TIMEOUT_SEC = 7200
        self.NAV_TO_GOAL_TIMEOUT_SEC = 300.0
        self.DIST_THRESH_FOR_HEADING_CALC = 0.2

        self.goal_pose = PoseStamped()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info('Starting frontier exploration...')
        self.explore()

    def explore(self):
        fully_explored = False

        while not fully_explored:

            # Get a frontier we can drive to
            self.goal_pose = self.get_reachable_goal()

            # If we cant find a path to any frontiers
            if self.goal_pose is None:
                exit(-1)
            
            # Go to the goal pose
            self.navigator.goToPose(self.goal_pose)
            
            # Keep doing stuff as long as the robot is moving towards the goal
            i = 0
            while not self.navigator.isNavComplete():
                
                feedback = self.navigator.getFeedback()

                if feedback:
                    # Set goal pose heading to robots current heading once it is close]
                    # This avoids unecessary rotation once reaching the goal position
                    # if feedback.distance_remaining < self.DIST_THRESH_FOR_HEADING_CALC:
                    #     self.get_logger().info('Setting new heading')
                    #     self.set_goal_heading()
                    #     self.navigator.goToPose(self.goal_pose)

                    # DEBUG info
                    if i % 15 == 0:
                        print('Distance remaining: ' + '{:.2f}'.format(
                            feedback.distance_remaining) + ' meters.')
                    i += 1
            
                # Some navigation timeout to demo cancellation
                if Duration.from_msg(feedback.navigation_time) > Duration(seconds=self.NAV_TO_GOAL_TIMEOUT_SEC):
                    self.navigator.cancelNav()
            
                # TODO something if the robot seems stuck?
                # Some navigation request change to demo preemption
                # if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
                #     self.goal_pose.pose.position.x = -3.0
                #     self.navigator.goToPose(self.goal_pose)
            
            result = self.navigator.getResult()
            self.log_nav_status(result)

    def get_reachable_goal(self):
        rank = 0
        reachable = False
        while not reachable:
            goal = self.send_request(rank)
            self.goal_pose.header.frame_id = 'map'
            self.goal_pose.header.stamp = self.navigator.get_clock().now().to_msg()

            # sanity check a valid path exists
            initial_pose = self.get_current_pose()
            path = self.navigator.getPath(initial_pose, self.goal_pose)

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
        # return self.future.result()

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
        return p
    
    def set_goal_heading(self):
        curr_pose = self.get_current_pose()
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
            self.get_logger().info('Goal has an invalid return status!')


def main(args=None):
    rclpy.init(args=args)
    frontier_explorer = FrontierExplorer()   
    frontier_explorer.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()