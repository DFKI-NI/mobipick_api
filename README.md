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
# open-set detection with AnyGrasp's Grounding DINO + SAM2 (no grasping): accepted
# detections land in the pose selector as <class_id>_<n>; returns grasplan/DetectObjectsResult
# with 2D boxes/masks, map-frame oriented 3D boxes and the accepted objects, or None if unavailable
mobipick.arm_cam.detect_open_set('coke can', use_vlm_verifier=True, observation_pose='observe100cm_right')
# two-stage variant: proposals from several views (nothing accepted yet) ...
first = mobipick.arm_cam.detect_proposals('coke can', max_proposals=3, observation_pose='observe100cm_right')
second = mobipick.arm_cam.detect_proposals('coke can', max_proposals=3, observation_pose='inspect100cm_right_low')
# ... grouped into candidate objects by 3D position (mobipick_api.open_set) and verified once each
# with a multi-image VLM request; verdict.status tells MATCH / NO_MATCH from verifier failures
from mobipick_api.open_set import Proposal, group_proposals
candidates = group_proposals([Proposal(d.detection_id, d.view_id, d.score, d.label, tuple(d.bbox_xyxy),
                                       (d.position.x, d.position.y, d.position.z) if d.has_position else None)
                              for r in (first, second) if r for d in r.detections], radius=0.08)
mobipick.arm_cam.verify_candidates('coke can', [c.detection_ids for c in candidates], commit=True)
# query 6D pose estimate of a specific object
mobipick.arm_cam.get_object_pose('multimeter_1')
# query if a specific object was perceived or not
mobipick.arm_cam.is_object_inside_pose_selector('multimeter_1') # expected return value is a boolean
# remove pose-selector entries for objects whose semantic facts place them on a table
mobipick.arm_cam.clear_poses_for_table('table_1')
# match a map-frame pose to the nearest configured table center
mobipick.table_matcher.closest_table({'position': {'x': 12.1, 'y': 3.2, 'z': 0.8}})
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
