#!/usr/bin/env python3

import rospy
from symbolic_fact_generation.msg import ChangedFacts
from symbolic_fact_generation.msg import Facts

class SemEnvRep:
    def __init__(self, namespace):
        rospy.Subscriber("/fact_publisher/changed_facts", ChangedFacts, self._ChangedFactsCB)
        rospy.Subscriber("/fact_publisher/facts", Facts, self._FactsCB)
        self.changed_facts = None
        self.facts = None

    def _ChangedFactsCB(self, msg):
        self.changed_facts = msg

    def get_changed_facts(self):
        return self.changed_facts

    def _FactsCB(self, msg):
        self.facts = msg

    def get_facts(self):
        if self.facts is None:
            return []
        return self.facts.facts
