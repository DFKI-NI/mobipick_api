# mobipick_api

[robot_api](https://github.com/DFKI-NI/robot_api) customized for the mobipick robot.

A pre- and concise Python API to control [mobipick robot](https://github.com/DFKI-NI/mobipick) with simple commands.

# Usage snippets

Navigation and simple manipulation:
```
import mobipick_api
# Get a Robot object using the robot's namespace.
mobipick = mobipick_api.Robot("mobipick")
# Get the robot's 2D pose using localization.
mobipick.base.get_2d_pose()
# Move the robot's arm using MoveIt.
mobipick.arm.move("transport")
# Move the robot's base using move_base.
mobipick.base.move(21.0, 7.0, 3.141592)
```

Perception:
```
#!/usr/bin/env python3

import mobipick_api

mobipick = mobipick_api.Robot('mobipick')
#mobipick.arm_cam.perceive()
mobipick.arm_cam.perceive(observation_list=['observe100cm_right', 'observe100cm_front'])

#mobipick.arm.move('observe100cm_right')
#mobipick.arm_cam.perceive_without_moving_arm()

print(mobipick.arm_cam.get_object_pose('multimeter_1'))
print(mobipick.arm_cam.is_object_inside_pose_selector('multimeter_1'))
```

# Credit

mobipick_api depends and is inspired by robot_api which was developed by Alexander Sung alexander.sung@dfki.de

mobipick_api was developed and is maintained by Oscar Lima oscar.lima@dfki.de
