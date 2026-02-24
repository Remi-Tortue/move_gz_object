#!/usr/bin/env python3
import numpy as np
import time

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rcl_interfaces.msg import ParameterDescriptor, ParameterType

from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState, GetEntityState

from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Quaternion
from move_gz_object.srv import ModifyObjectPose

#

def mult_orientation(a,b):
    # x,y,z,w
    return np.array([        
        a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2],  # 1
        a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1],  # i
        a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],  # j
        a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3]   # k
        ])

    


class Randomized_Object_Pose(Node):
   
    def __init__(self) -> None:
        super().__init__('randomized_object_pose_srv')

        seed_value  = self.declare_parameter("seed_value", 0, 
                                                    ParameterDescriptor(type=ParameterType.PARAMETER_STRING)).get_parameter_value().integer_value
        np.random.seed(seed_value)

        # pub
        self.__pub_offset_pose = self.create_publisher(PoseStamped, 'offset_pose', 10)

        # Services
        srv_callback_group = MutuallyExclusiveCallbackGroup()
        self.srv = self.create_service(ModifyObjectPose, 'modify_object_pose', self.__callback_srv, callback_group=srv_callback_group)

        # Client for setting entity state in Gazebo
        cli_callback_group = MutuallyExclusiveCallbackGroup()
        self.set_entity_state_client = self.create_client(SetEntityState, '/gazebo/set_entity_state', callback_group=cli_callback_group)
        self.get_entity_state_client = self.create_client(GetEntityState, '/gazebo/get_entity_state', callback_group=cli_callback_group)

        # Wait for services to be available
        while not self.set_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('set_entity_state service not available, waiting again...')

        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('get_entity_state service not available, waiting again...')


    ##################################################################################


    def __callback_srv(self, request, response):
        self.get_logger().info(f"Received request to modify pose of object: {request.object_name}")

        # Get current pose of the object
        current_pose = self.get_current_pose(request.object_name)
        self.get_logger().info(f"current_pose: {current_pose}")

        if current_pose is None:
            response.success = False
            response.message = "Failed to get current pose"
            return response

        # Generate a new random pose within the provided range
        new_pose = self.generate_random_pose(current_pose, request.range)
        self.get_logger().info(f"new_pose: {new_pose}")

        # Set the new pose
        success = self.set_new_pose(request.object_name, new_pose)
        if success:
            response.success = True
            response.message = f"Successfully updated pose for {request.object_name}"
            response.new_pose = PoseStamped(pose=new_pose)
        else:
            response.success = False
            response.message = "Failed to set new pose"

        return response


    #


    def get_current_pose(self, object_name: str) -> Pose:
        """Get the current pose of the object using Gazebo's GetEntityState service."""
        req = GetEntityState.Request()
        req.name = object_name
        future = self.get_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            result = future.result()
            return result.state.pose
        except Exception as e:
            self.get_logger().error(f"Error getting entity state: {e}")
            return None


    def generate_random_pose(self, current_pose: Pose, pose_range: Pose) -> Pose:
        """Generate a random pose within the given range around the current pose."""
        # Convert Pose to numpy arrays for easier manipulation
        current_pos = np.array([current_pose.position.x, current_pose.position.y, current_pose.position.z])
        current_orient = np.array([current_pose.orientation.x, current_pose.orientation.y,
                                    current_pose.orientation.z, current_pose.orientation.w])

        # Random translation within the specified range
        translation_range = np.array([
            pose_range.position.x,
            pose_range.position.y,
            pose_range.position.z
        ])
        rand_translation = np.random.uniform(-translation_range, translation_range)

        # Random rotation (we limit to Euler angles to simplify)
        rotation_range = np.array([
            pose_range.orientation.x,
            pose_range.orientation.y,
            pose_range.orientation.z,
            pose_range.orientation.w
        ])
        rand_rotation = np.random.uniform(np.array([0.,0.,0.,1.]), rotation_range)

        self.__pub_offset_pose.publish(PoseStamped(pose=Pose(position=Point(x=rand_translation[0], y=rand_translation[1], z=rand_translation[2]),
                                                             orientation=Quaternion(x=rand_rotation[0], y=rand_rotation[1], z=rand_rotation[2], w=rand_rotation[3]))))

        new_position = current_pos + rand_translation
        # new_orientation = current_orient + rand_rotation
        new_orientation = mult_orientation(current_orient, rand_rotation)

        # Normalize quaternion
        new_orientation = new_orientation / np.linalg.norm(new_orientation)

        new_pose = Pose()
        new_pose.position.x = float(new_position[0])
        new_pose.position.y = float(new_position[1])
        new_pose.position.z = float(new_position[2])
        new_pose.orientation.x = float(new_orientation[0])
        new_pose.orientation.y = float(new_orientation[1])
        new_pose.orientation.z = float(new_orientation[2])
        new_pose.orientation.w = float(new_orientation[3])

        return new_pose


    def set_new_pose(self, object_name: str, new_pose: Pose) -> bool:
        """Set the new pose of the object using Gazebo's SetEntityState service."""
        req = SetEntityState.Request()
        req.state.name = object_name
        req.state.pose = new_pose
        future = self.set_entity_state_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        try:
            result = future.result()
            return result.success
        except Exception as e:
            self.get_logger().error(f"Error setting entity state: {e}")
            return False


    ##################################################################################


    def __enter__(self):
        while True:
            try:
                return self
            except RuntimeError as e:
                self.get_logger().warn(f"Error when initializing: {e}. Retrying...")

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.get_logger().debug("Stop randomized object pose srv")


##################################################################################


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
    with Randomized_Object_Pose() as node:
        run_node(node, args)

if __name__ == '__main__':
    main()
