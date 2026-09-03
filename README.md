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
# activate pose selector, collect updates for 5 seconds, deactivate pose selector
# as DOPE is implemented as a lazy subscriber, activating pose selector means DOPE is activated as well
# DOPE : Deep Object Pose Estimation
mobipick.arm_cam.perceive()
# or
mobipick.arm_cam.perceive(observation_list=[])

# alternatively, define a list of observation poses to visit
mobipick.arm_cam.perceive(observation_list=['observe100cm_right', 'observe100cm_front'])
# query 6D pose estimate of a specific object
mobipick.arm_cam.get_object_pose('multimeter_1')
# query if a specific object was perceived or not
mobipick.arm_cam.is_object_inside_pose_selector('multimeter_1') # expected return value is a boolean
# remove pose-selector entries for objects whose semantic facts place them on a table
mobipick.arm_cam.clear_poses_for_table('table_1')
```

The robot object can be created before the pose selector node is running. Perception
operations wait for the particular service they need and can be retried after a
startup-order or temporary service outage. If the service does not become ready,
the operation raises `rospy.ServiceException` with the unavailable service name and
retry guidance. The readiness timeout defaults to 2 seconds and can be configured
with the private ROS parameter `~pose_selector_service_timeout`.

Manipulation (with MoveIt):
```
# move the robot's arm in configuration space to predefined semantic poses
mobipick.arm.move('transport')
# to see predefined semnatic poses do the following command in a terminal:
roscat mobipick_moveit_config mobipick.srdf.xacro | grep arm | grep state
# pick an object that was previously perceived
mobipick.arm.pick_object('multimeter_1', 'table_3', planning_scene_ignore_list=[], timeout=50.0)
# insert an object that was previously picked into a container, e.g. a box
mobipick.arm.insert_object('klt_3', observe_before_insert=False, timeout=50.0)
# assuming that mobipick has an object in its gripper, you can place it on a surface by doing:
mobipick.arm.place_object('table_3', observe_before_place=False, timeout=50.0)
```

# Credit

mobipick_api depends and is inspired by robot_api which was developed by Alexander Sung alexander.sung@dfki.de

mobipick_api was developed and is maintained by Oscar Lima oscar.lima@dfki.de
