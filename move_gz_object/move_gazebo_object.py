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

#


class Move_Gazebo_Object(Node):
   
  def __init__(self) -> None:
    super().__init__('move_gazebo_object')
    self.gz_object_name  = self.declare_parameter("gz_object_name", "roi_sphere", 
                                                  ParameterDescriptor(type=ParameterType.PARAMETER_STRING)).get_parameter_value().string_value
    self.pose_topic  = self.declare_parameter("pose_topic", "/object_pose", 
                                                  ParameterDescriptor(type=ParameterType.PARAMETER_STRING)).get_parameter_value().string_value
    
    # sub
    sub_callback_group = MutuallyExclusiveCallbackGroup()
    self.sub_object = self.create_subscription(PoseStamped, self.pose_topic, self.__callback_object, 10, callback_group=sub_callback_group)

    # Cli
    cli_callback_group = MutuallyExclusiveCallbackGroup()
    self.cli_entity_state = self.create_client(SetEntityState, '/gazebo/set_entity_state', callback_group=cli_callback_group)

    # Timer
    timer_callback_group = MutuallyExclusiveCallbackGroup()
    self.__timer_period = 0.01  # seconds
    self.__timer_main = self.create_timer(self.__timer_period, self.__main, callback_group=timer_callback_group)
    self.get_logger().info('Start move object')

    # Init msg
    self.object_msg = PoseStamped()

    self.object_state_gazebo_msg = EntityState()
    self.object_state_gazebo_msg.name = self.gz_object_name
    self.object_state_gazebo_msg.pose = self.object_msg.pose


#


  def __callback_object(self, msg):
    self.object_msg = msg


  def __main(self):
    self.object_state_gazebo_msg.pose = self.object_msg.pose
    req = SetEntityState.Request()
    req.state = self.object_state_gazebo_msg

    future = self.cli_entity_state.call_async(req)
    future.add_done_callback(self.gazebo_client_callback_response)


  def gazebo_client_callback_response(self, future):
      try:
          response = future.result()
          if response.success:
              pass
              # self.get_logger().info("Entity moved successfully!")
          else:
              self.get_logger().warn("Failed to move the gazebo object entity!")
              self.get_logger().warn("Deduce that there is no gazebo object to move.")
              self.destroy_node()
      except Exception as e:
          self.get_logger().error(f"Service call failed: {e}")


  # 


  def __enter__(self):
    while True:
      try:
        return self
      except RuntimeError as e:
          self.get_logger().warn(f"Error when initializing: {e}. Retrying...")

  def __exit__(self, exc_type, exc_val, exc_tb):
    self.get_logger().debug("Stop move entity")


###########################################################################################


def run_node(node: Node, args):
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(f"User stopped {node.get_name()}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    # try:
    #     rclpy.spin(node)
    # except KeyboardInterrupt:
    #     node.get_logger().info(f"User stopped {node.get_name()}")
    # finally:
    #     node.destroy_node()
    #     rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    with Move_Gazebo_Object() as node:
        run_node(node, args)

if __name__ == '__main__':

  main()