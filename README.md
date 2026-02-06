# move_gz_object
Tools to move a gazebo object on simulation.


## How to use
#### move_gazebo_object
Move a gazebo object `gz_object_name` following given position via `pose_topic` topic.

#### randomized_object_pose
To randomize the position of a gazebo object within a certain range regarding its current position, you van use `randomized_object_pose.py`.

[NOTE] Add it into your launch file to try different init positons :

```python
# Launch randomize target pose node
nodes_to_start.append(Node(
    package='move_gz_object',
    executable='randomized_object_pose.py',
    output='log',
))

# Call the service
my_service_name = "/modify_object_pose"
my_service_type = "move_gz_object/srv/ModifyObjectPose"
my_service_arguments = '{object_name: "aruco_marker", range: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

command = f"ros2 service call {my_service_name} {my_service_type} '{my_service_arguments.strip()}'"

nodes_to_start.append(ExecuteProcess(
    cmd=['bash', '-c', command],
))
```

## Overview
- `move_gazebo_object.py` Move a gazebo object based on an input topic.
- `randomized_object_pose.py` Service that modify the current position of a gazebo object.

## TODO

## Contributors
- **Rémi Porée**