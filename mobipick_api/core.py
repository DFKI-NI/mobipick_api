#!/usr/bin/env python3

import rospy
from robot_api.lib import _init_node
from robot_api import Base
from mobipick_api.perception import Perception
from mobipick_api.manipulation import Manipulation
from mobipick_api.semantic_environment_rep import SemEnvRep
from mobipick_api.hri import HRI

from ur_dashboard_msgs.srv import GetSafetyMode, GetRobotMode
from ur_dashboard_msgs.msg import SafetyMode, RobotMode

class Robot:
    def __init__(self, namespace: str=rospy.get_namespace(), connect_navigation_on_init: bool=False,
            connect_manipulation_on_init: bool=False) -> None:
        _init_node()
        # Make sure namespace naming is correct.
        if not namespace.startswith('/'):
            namespace = '/' + namespace
        if not namespace.endswith('/'):
            namespace += '/'
        self.namespace = namespace
        self.base = Base(namespace, connect_navigation_on_init)
        self.arm = Manipulation(namespace, connect_manipulation_on_init)
        self.semantic_env_rep = SemEnvRep(namespace)
        self.arm_cam = Perception(namespace, self.arm, self.semantic_env_rep)
        self.hri = HRI(namespace)

        self._emergency_stop_status = rospy.ServiceProxy(
            f'{self.namespace}ur_hardware_interface/dashboard/get_safety_mode',
            GetSafetyMode)

        self._robot_mode_status = rospy.ServiceProxy(
            f'{self.namespace}ur_hardware_interface/dashboard/get_robot_mode',
            GetRobotMode)

    def emergency_stop_triggered(self) -> bool:
        """Check if the emergency stop has been triggered.

        Returns:
            bool: True if emergency stop is triggered, False otherwise.
        """
        try:
            self._emergency_stop_status.wait_for_service(timeout=5.0)
            self._robot_mode_status.wait_for_service(timeout=5.0)
            status_response = self._emergency_stop_status()
            robot_mode_response = self._robot_mode_status()
            return status_response.safety_mode.mode == SafetyMode.ROBOT_EMERGENCY_STOP or robot_mode_response.robot_mode.mode == RobotMode.POWER_OFF
        except rospy.ServiceException as e:
            rospy.logerr(f'UR5 Dashboard Safety Mode Service not available: {e}')
            return False
