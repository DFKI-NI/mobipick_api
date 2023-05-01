#!/usr/bin/env python3

import rospy
from robot_api.lib import _init_node
from robot_api import Base
from mobipick_api.perception import Perception
from mobipick_api.manipulation import Manipulation
from mobipick_api.semantic_environment_rep import SemEnvRep

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
        self.arm_cam = Perception(namespace, self.arm)
        self.semantic_env_rep = SemEnvRep(namespace)
