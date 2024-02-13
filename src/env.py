#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty
from training_zone.srv import Step
from geometry_msgs.msg import Pose

import numpy as np
import math

class Env(Node):
  def __init__(self):
    super().__init__('env')
    
    self.min_angle = 10.0
    self.min_range = 10.0

    self.goal_init_dis = 10.0
    self.goal_dis = 10
    self.goal_angle = 0.0
    
    self.goal_pose = (10, 10)
    
    self.done = False
    self.success = False
    self.fail = False
    
    qos = QoSProfile(depth=10)
    
    self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)
    
    self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, qos)
    self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos)
    
    self.goal_pose_sub = self.create_subscription(Pose, 'goal_pose', self.goal_pose_callback, qos)
    
    self.step_server = self.create_service(Step, 'step', self.step_callback)

    self.task_success_client = self.create_client(Empty, 'task_success')
    while not self.task_success_client.wait_for_service(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')
          
    self.task_fail_client = self.create_client(Empty, 'task_fail')
    while not self.task_fail_client.wait_for_service(timeout_sec=1.0):
          self.get_logger().info('service not available, waiting again...')

  def step_callback(self, req, res):
  
    act_msg = Twist()
    act_msg.linear.x = 0.5
    act_msg.angular.z = ((3 - 1)/2 - req.action) * 1.5
        
    self.cmd_vel_pub.publish(act_msg)
    
    res.state = self.get_state()
    res.done = self.is_done()
    res.reward = self.get_reward()
    
    if self.done:
      if self.success:
        self.task_success_client.call_async(Empty.Request())
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('success')
        self.success = False
      else:
        self.task_fail_client.call_async(Empty.Request())
        self.cmd_vel_pub.publish(Twist())
        self.get_logger().info('fail')
        self.fail = False
      self.done = False
      
    if req.init:
      self.goal_init_dis = self.goal_dis
      
    return res
    
  def odom_callback(self, odom):
    x = odom.pose.pose.position.x
    y = odom.pose.pose.position.y
    _, _, self.last_pose_theta = self.euler_from_quaternion(odom.pose.pose.orientation)
    
    goal_pose = np.array(self.goal_pose)
    pose = np.array([x, y])
    
    self.goal_dis = np.linalg.norm(goal_pose - pose)
    
    path_theta = math.atan2(
            self.goal_pose[1]-y,
            self.goal_pose[0]-x)
            
    goal_angle = path_theta - self.last_pose_theta
    if goal_angle > math.pi:
      goal_angle -= 2 * math.pi
    elif goal_angle < -math.pi:
      goal_angle += 2 * math.pi
      
    self.goal_angle = goal_angle
    
  def scan_callback(self, scan):
    self.min_range = min(scan.ranges)
    self.min_angle = np.argmin(scan.ranges)
    
  def goal_pose_callback(self, pose):
    self.goal_pose = (pose.position.x, pose.position.y)
    
  def get_state(self):
    state = list()
    state.append(float(self.min_range) if self.min_range < 3.5 else 3.5)
    state.append(float(self.min_angle)/360)
    state.append(float(self.goal_init_dis) / 2 - float(self.goal_dis))
    state.append(float(self.goal_angle))
    
    return state
    
  def get_reward(self):
    dis_reward = (2 * self.goal_init_dis) / \
            (self.goal_init_dis + self.goal_dis) - 1
    yaw_reward = 1 - 2*math.sqrt(math.fabs(self.goal_angle / math.pi))
    ob_reward = 0

    if self.min_range < 0.25:
      ob_reward = -2
    
    reward = yaw_reward + ob_reward + dis_reward
      
    if self.done:
      if self.success:
        reward += 5
      elif self.fail:
        reward += -10
        
    return reward
  
  def is_done(self):
  
    if self.goal_dis < 0.7:
      self.done = True
      self.success = True
    
    if self.min_range < 0.15:
      self.done = True
      self.fail = True
      
    return self.done
        
  def euler_from_quaternion(self, quat):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quat = [x, y, z, w]
    """
    x = quat.x
    y = quat.y
    z = quat.z
    w = quat.w

    sinr_cosp = 2 * (w*x + y*z)
    cosr_cosp = 1 - 2*(x*x + y*y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w*y - z*x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w*z + x*y)
    cosy_cosp = 1 - 2 * (y*y + z*z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
  
def main(args=None):
    rclpy.init(args=args)

    env = Env()
    
    rclpy.spin(env)
    
    env.destroy_node()
    
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
		
