#! /usr/bin/env python3

#Ref: https://automaticaddison.com/how-to-send-goals-to-the-ros-2-navigation-stack-nav2/
 
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration
import rclpy
 
from robot_navigator import BasicNavigator, NavigationResult
 
'''
Navigates a robot from an initial pose to a goal pose.
'''
def main():
 
  rclpy.init()
  navigator = BasicNavigator()
 
  # You may use the navigator to clear or obtain costmaps
  # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
  # global_costmap = navigator.getGlobalCostmap()
  # local_costmap = navigator.getLocalCostmap()
 
  # Set the robot's goal pose
  goal_pose = PoseStamped()
  goal_pose.header.frame_id = 'map'
  goal_pose.header.stamp = navigator.get_clock().now().to_msg()
  goal_pose.pose.position.x = 5.0
  goal_pose.pose.position.y = 3.0
  goal_pose.pose.position.z = 0.0
  goal_pose.pose.orientation.x = 0.0
  goal_pose.pose.orientation.y = 0.0
  goal_pose.pose.orientation.z = 0.0
  goal_pose.pose.orientation.w = 1.0
 
  # sanity check a valid path exists
  # path = navigator.getPath(initial_pose, goal_pose)
 
  # Go to the goal pose
  navigator.goToPose(goal_pose)
 
  i = 0

  # Keep doing stuff as long as the robot is moving towards the goal
  while not navigator.isNavComplete():
    ################################################
    #
    # Implement some code here for your application!
    #
    ################################################
 
    # Do something with the feedback
    i = i + 1
    feedback = navigator.getFeedback()
    if feedback and i % 15 == 0:
      print('Distance remaining: ' + '{:.2f}'.format(
            feedback.distance_remaining) + ' meters.')
 
      # Some navigation timeout to demo cancellation
      if Duration.from_msg(feedback.navigation_time) > Duration(seconds=300.0):
        navigator.cancelNav()
 
      # Some navigation request change to demo preemption
    #   if Duration.from_msg(feedback.navigation_time) > Duration(seconds=120.0):
    #     goal_pose.pose.position.x = -3.0
    #     navigator.goToPose(goal_pose)
 
  # Do something depending on the return code
  result = navigator.getResult()
  if result == NavigationResult.SUCCEEDED:
      print('Goal succeeded!')
  elif result == NavigationResult.CANCELED:
      print('Goal was canceled!')
  elif result == NavigationResult.FAILED:
      print('Goal failed!')
  else:
      print('Goal has an invalid return status!')
 
  exit(0)
 
if __name__ == '__main__':
  main()