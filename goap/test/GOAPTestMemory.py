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

from goap.goap import *
from goap.inheriting import *
from goap.planning import Planner


#@unittest.skip
class TestSimple(unittest.TestCase):

    def setUp(self):
        self.memory = Memory()
        self.worldstate = WorldState(self.memory)

        self.memory.set_value('memory.counter', 0)

        print self.memory

        Condition._conditions_dict.clear()

        Condition.add('memory.counter', MemoryCondition('counter', self.worldstate))

        print Condition.print_dict()

        self.actionbag = ActionBag()
        self.actionbag.add(MemoryChangeVarAction('counter', 2, 3))
        self.actionbag.add(MemoryChangeVarAction('counter', 0, 1))
        self.actionbag.add(MemoryChangeVarAction('counter', 1, 2))
        self.actionbag.add(MemoryChangeVarAction('counter', -2, 3))

        print self.actionbag

        self.goal = Goal([Precondition(Condition.get('memory.counter'), 3)])

        self.goal_inaccessible = Goal([Precondition(Condition.get('memory.counter'), 4)])

        print self.worldstate


    def testGoals(self):
        print '==', self.testGoals.__name__
        self.assertFalse(self.goal.is_valid(self.worldstate), 'Goal should not be valid yet')

    def testPlannerPos(self):
        print '==', self.testPlannerPos.__name__
        planner = Planner(self.actionbag, self.worldstate, self.goal)
        plan = planner.plan()
        print 'plan found: ', plan
        self.assertIsNotNone(plan, 'There should be a plan')
        self.assertEqual(len(plan), 3, 'Plan should have three actions')

    def testPlannerNeg(self):
        print '==', self.testPlannerNeg.__name__
        planner = Planner(self.actionbag, self.worldstate, self.goal_inaccessible)
        plan = planner.plan()
        print 'plan found: ', plan
        self.assertIsNone(plan, 'There should be no plan')


    def tearDown(self):
        print 'memory was:', self.memory


#@unittest.skip
class TestIncrementer(unittest.TestCase):

    def setUp(self):
        self.memory = Memory()
        self.worldstate = WorldState(self.memory)

        self.memory.set_value('memory.counter', 0)

        print self.memory

        Condition._conditions_dict.clear()

        Condition.add('memory.counter', MemoryCondition('counter', self.worldstate))

        self.actionbag = ActionBag()
        self.actionbag.add(MemoryIncrementerAction('counter'))

        print Condition.print_dict()

        print self.actionbag

        self.goal = Goal([Precondition(Condition.get('memory.counter'), 3)])

        self.goal_inaccessible = Goal([Precondition(Condition.get('memory.counter'), -2)])

        print self.worldstate


    def testGoals(self):
        print '==', self.testGoals.__name__
        self.assertFalse(self.goal.is_valid(self.worldstate), 'Goal should not be valid yet')

    def testPlannerPos(self):
        print '==', self.testPlannerPos.__name__
        planner = Planner(self.actionbag, self.worldstate, self.goal)
        plan = planner.plan()
        print 'plan found: ', plan
        self.assertIsNotNone(plan, 'There should be a plan')
        self.assertEqual(len(plan), 3, 'Plan should have three actions')

    def testPlannerNeg(self):
        print '==', self.testPlannerNeg.__name__
        planner = Planner(self.actionbag, self.worldstate, self.goal_inaccessible)
        plan = planner.plan()
        print 'plan found: ', plan
        self.assertIsNone(plan, 'There should be no plan')

    def testPlannerNegPos(self):
        print '==', self.testPlannerPos.__name__
        self.actionbag.add(MemoryIncrementerAction('counter', -4))
        planner = Planner(self.actionbag, self.worldstate, self.goal_inaccessible)
        plan = planner.plan()
        print 'plan found: ', plan
        self.assertIsNotNone(plan, 'There should be a plan')
        self.assertEqual(len(plan), 3, 'Plan should have three actions')


    def tearDown(self):
        print 'memory was:', self.memory



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
