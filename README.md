# mobipick_api

[robot_api](https://github.com/DFKI-NI/robot_api) customized for the mobipick robot.

A pre- and concise Python API to control [mobipick robot](https://github.com/DFKI-NI/mobipick) with simple commands.

# Usage snippets

Get a Mobipick Robot object using the robot's namespace:
```
import mobipick_api
mobipick = mobipick_api.Robot('mobipick')
```

Navigation:
```
# Get the robot's 2D pose using localization.
mobipick.base.get_2d_pose()
# Move the robot's base using move_base.
mobipick.base.move(21.0, 7.0, 3.141592)
```

Perception:
```
# go to observe100cm_right arm pose and activate pose selector
mobipick.arm_cam.perceive()
# alternatively, define a list of observation poses to visit
mobipick.arm_cam.perceive(observation_list=['observe100cm_right', 'observe100cm_front'])
# alternatively, perceive without moving the robot arm
mobipick.arm_cam.perceive_without_moving_arm()
# query 6D pose estimate of a specific object
print(mobipick.arm_cam.get_object_pose('multimeter_1'))
# query if a specific object was perceived or not
print(mobipick.arm_cam.is_object_inside_pose_selector('multimeter_1')) # expected return value is a boolean
```

Manipulation (with MoveIt):
```
# move the robot's arm in configuration space to predefined semantic poses
mobipick.arm.move('transport')
# to see predefined semnatic poses do the following command in a terminal:
roscat mobipick_moveit_config mobipick.srdf.xacro | grep arm | grep state
# pick an object that was previously perceived
mobipick.arm.pick_object('multimeter_1', 'table_3', planning_scene_ignore_list=[], timeout=50.0)
```

# Credit

mobipick_api depends and is inspired by robot_api which was developed by Alexander Sung alexander.sung@dfki.de

mobipick_api was developed and is maintained by Oscar Lima oscar.lima@dfki.de
