#!/usr/bin/env python3

from typing import List, Optional
import rospy
from symbolic_fact_generation.msg import ChangedFacts
from symbolic_fact_generation.msg import Facts

class SemEnvRep:
    def __init__(self, namespace: str):
        rospy.Subscriber("/fact_publisher/changed_facts", ChangedFacts, self._ChangedFactsCB)
        rospy.Subscriber("/fact_publisher/facts", Facts, self._FactsCB)
        self.changed_facts: Optional[ChangedFacts] = None
        self.facts: Optional[Facts] = None

    def _ChangedFactsCB(self, msg: ChangedFacts) -> None:
        self.changed_facts = msg

    def get_changed_facts(self) -> Optional[ChangedFacts]:
        return self.changed_facts

    def _FactsCB(self, msg) -> None:
        self.facts = msg

    def get_facts(self) -> List[object]:
        if self.facts is None:
            return []
        return self.facts.facts
