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

from goap.common import *
from goap.inheriting import *
from goap.planning import Planner, Node, PlanExecutor
from goap.runner import Runner


#@unittest.skip
class TestSimple(unittest.TestCase):

    def setUp(self):
        self.runner = Runner()

        self.memory = self.runner.memory
        self.worldstate = self.runner.worldstate

        self.memory.set_value('memory.counter', 0)

        print self.memory

        Condition._conditions_dict.clear() # start every test without previous conditions

        Condition.add(MemoryCondition(self.memory, 'memory.counter'))

        print Condition.print_dict()

        self.actionbag = self.runner.actionbag
        self.actionbag.add(MemoryChangeVarAction(self.memory, 'memory.counter', 2, 3))
        self.actionbag.add(MemoryChangeVarAction(self.memory, 'memory.counter', 0, 1))
        self.actionbag.add(MemoryChangeVarAction(self.memory, 'memory.counter', 1, 2))
        self.actionbag.add(MemoryChangeVarAction(self.memory, 'memory.counter', -2, 3))

        print self.actionbag

        self.goal = Goal([Precondition(Condition.get('memory.counter'), 3)])

        self.goal_inaccessible = Goal([Precondition(Condition.get('memory.counter'), 4)])

        print self.worldstate

        print self.runner

    def testGoals(self):
        print '==', self.testGoals.__name__
        Condition.initialize_worldstate(self.worldstate) # needed because worldstate was never initialized before
        self.assertFalse(self.goal.is_valid(self.worldstate), 'Goal should not be valid yet')

    def testPlannerPos(self):
        print '==', self.testPlannerPos.__name__
        start_node = self.runner.update_and_plan(self.goal, introspection=True)
        print 'start_node found: ', start_node
        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')
        self.assertEqual(len(start_node.parent_actions_path_list), 3, 'Start Node should have three actions')
        self.assertEqual(len(start_node.parent_nodes_path_list), 3, 'Start Node should have three parent nodes')

        PlanExecutor().execute(start_node)

        import rospy
        rospy.sleep(5) # to latch introspection # TODO: check why spinner does not work [while in unittest]


    def testPlannerNeg(self):
        print '==', self.testPlannerNeg.__name__
        start_node = self.runner.update_and_plan(self.goal_inaccessible)
        print 'start_node found: ', start_node
        self.assertIsNone(start_node, 'There should be no plan')


    def tearDown(self):
        print 'memory was:', self.memory


#@unittest.skip
class TestIncrementer(unittest.TestCase):

    def setUp(self):
        self.runner = Runner()

        self.memory = self.runner.memory
        self.worldstate = self.runner.worldstate

        self.memory.set_value('memory.counter', 0)

        print self.memory

        Condition._conditions_dict.clear() # start every test without previous conditions

        Condition.add(MemoryCondition(self.memory, 'memory.counter'))

        Condition.initialize_worldstate(self.worldstate)

        self.actionbag = self.runner.actionbag
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter'))

        print Condition.print_dict()

        print self.actionbag

        self.goal = Goal([Precondition(Condition.get('memory.counter'), 3)])

        self.goal_inaccessible = Goal([Precondition(Condition.get('memory.counter'), -2)])

        print self.worldstate

        print self.runner

    def testGoals(self):
        print '==', self.testGoals.__name__
        Condition.initialize_worldstate(self.worldstate) # needed because worldstate was never initialized before
        self.assertFalse(self.goal.is_valid(self.worldstate), 'Goal should not be valid yet')

    def testPlannerPos(self):
        print '==', self.testPlannerPos.__name__
        start_node = self.runner.update_and_plan(self.goal)
        print 'start_node found: ', start_node
        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')
        self.assertEqual(len(start_node.parent_actions_path_list), 3, 'Plan should have three actions')


    def testPlannerPosUnneededCondition(self):
        Condition.add(MemoryCondition(self.memory, 'memory.unneeded'))
        Condition.initialize_worldstate(self.worldstate)
        print 'reinitialized worldstate with unneeded condition: ', self.worldstate
        self.testPlannerPos()


    def testPlannerNeg(self):
        print '==', self.testPlannerNeg.__name__
        start_node = self.runner.update_and_plan(self.goal_inaccessible)
        print 'start_node found: ', start_node
        self.assertIsNone(start_node, 'There should be no plan')

    def testPlannerNegPos(self):
        print '==', self.testPlannerNegPos.__name__
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter', -4))
        start_node = self.runner.update_and_plan(self.goal_inaccessible, introspection=True)
        print 'start_node found: ', start_node
        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')
        self.assertEqual(len(start_node.parent_actions_path_list), 3, 'Plan should have three actions')

        import rospy
        rospy.sleep(5) # to latch introspection # TODO: check why spinner does not work [while in unittest]


    @unittest.skip("deviation does not work yet") # FIXME: deviation does not work yet
    def testPlannerDeviation(self):
        print '==', self.testPlannerDeviation.__name__
        goal_dev = Goal([Precondition(Condition.get('memory.counter'), 2.05, 0.1)])
        start_node = self.runner.update_and_plan(goal_dev)
        print 'start_node found: ', start_node
        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')
        self.assertEqual(len(start_node.parent_actions_path_list), 2, 'Plan should have two actions')


    def testPlannerBig(self):
        print '==', self.testPlannerBig.__name__
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter', -4))
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter', 11))
        self.actionbag.add(MemoryIncrementerAction(self.memory, 'memory.counter', 3))
        goal_big = Goal([Precondition(Condition.get('memory.counter'), 23)])
        start_node = self.runner.update_and_plan(goal_big, introspection=True)
        print 'start_node found: ', start_node

        import rospy
        rospy.sleep(5) # to latch introspection # TODO: check why spinner does not work [while in unittest]

        self.assertIsNotNone(start_node, 'There should be a plan')
        self.assertIsInstance(start_node, Node, 'Plan should be a Node')


    def tearDown(self):
        print 'memory was:', self.memory



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
