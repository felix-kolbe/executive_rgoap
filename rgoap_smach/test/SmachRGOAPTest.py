# Copyright (c) 2013, Felix Kolbe
# All rights reserved. BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# * Redistributions of source code must retain the above copyright
#   notice, this list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright
#   notice, this list of conditions and the following disclaimer in the
#   documentation and/or other materials provided with the distribution.
#
# * Neither the name of the {organization} nor the names of its
#   contributors may be used to endorse or promote products derived
#   from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import unittest
from unittest.case import SkipTest

import time

from smach import Sequence

from rgoap import Condition, Precondition, VariableEffect, Goal
from rgoap import MemoryCondition
from rgoap import Runner

from rgoap_smach import SMACHStateWrapperAction



def get_lookaround_smach_mock():
    sq = Sequence(outcomes=['succeeded', 'aborted', 'preempted'],
                  connector_outcome='succeeded')
    return sq


class LookAroundAction(SMACHStateWrapperAction):

    def __init__(self):
        SMACHStateWrapperAction.__init__(self, get_lookaround_smach_mock(),
                                  [Precondition(Condition.get('arm_can_move'), True)],
                                  [VariableEffect(Condition.get('awareness'))])

    def _generate_variable_preconditions(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect is self._effects[0]
        # increase awareness by one
        precond_value = worldstate.get_condition_value(effect._condition) - 1
        return [Precondition(effect._condition, precond_value, None)]



class Test(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        # rospy.init_node('smach_rgoap_test')
        pass

    def setUp(self):
        self.runner = Runner()

        memory = self.runner.memory
        memory.declare_state('awareness', 0)
        Condition.add(MemoryCondition(memory, 'awareness'))
        memory.declare_state('arm_can_move', True)
        Condition.add(MemoryCondition(memory, 'arm_can_move'))

        self.runner.actionbag.add(LookAroundAction())

        print self.runner.actionbag

    def tearDown(self):
        pass

    @classmethod
    def tearDownClass(cls):
        pass

    @SkipTest
    def testName(self):
        goal = Goal([Precondition(Condition.get('awareness'), 2)])

        self.runner.update_and_plan_and_execute(goal, introspection=True)

        time.sleep(15) # to latch introspection # TODO: check why spinner does not work [while in unittest]


    def testStateAction(self):
        Condition.add(MemoryCondition(self.runner.memory, 'robot.pose'))
        Condition.add(MemoryCondition(self.runner.memory, 'robot.bumpered'))



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
