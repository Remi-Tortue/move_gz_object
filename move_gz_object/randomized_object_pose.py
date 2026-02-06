#!/usr/bin/env python3
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
import std_msgs.msg as std_msgs
from move_gz_object.srv import ModifyObjectPose

#


class Randomized_Object_Pose(Node):
   
  def __init__(self) -> None:
    super().__init__('randomized_object_pose_srv')


    # Srv
    srv_callback_group = MutuallyExclusiveCallbackGroup()
    self.srv = self.create_service(ModifyObjectPose, 'modify_object_pose', self.__callback_srv, callback_group=srv_callback_group)


#


  def __callback_srv(self, request, response):
    self.get_logger().info("Modify object pose")
    


  # 


  def __enter__(self):
    while True:
      try:
        return self
      except RuntimeError as e:
          self.get_logger().warn(f"Error when initializing: {e}. Retrying...")

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.get_logger().debug("Stop randomized object pose srv")


###########################################################################################


def run_node(node: Node, args):
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(node)
    # try:
    #     executor.spin()
    # except KeyboardInterrupt:
    #     node.get_logger().info(f"User stopped {node.get_name()}")
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(f"User stopped {node.get_name()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    with Randomized_Object_Pose() as node:
        run_node(node, args)

if __name__ == '__main__':

  main()