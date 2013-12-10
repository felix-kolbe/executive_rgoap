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

from smach import Sequence, State

from rgoap import Condition, Precondition, Effect, VariableEffect, Goal
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
        Condition._conditions_dict.clear() # start every test without previous conditions
        self.runner = Runner()

    def tearDown(self):
        pass

    @classmethod
    def tearDownClass(cls):
        pass


    def testRunner(self):
        memory = self.runner.memory
        Condition.add(MemoryCondition(memory, 'awareness', 0))
        Condition.add(MemoryCondition(memory, 'arm_can_move', True))

        self.runner.actionbag.add(LookAroundAction())
        print self.runner.actionbag

        goal = Goal([Precondition(Condition.get('awareness'), 2)])
        self.runner.update_and_plan_and_execute(goal)


    def testStateWrapperAction(self):
        # The in-value is translated into the SMACHStateWrapperAction
        # and checked inside the wrapped state to check data translation
        # to the wrapped state.
        NUMBER_IN = 123
        # The out-value is translated into the SMACHStateWrapperAction
        # and out of it again to check data translation from the wrapped
        # state and also the next_worldstate parameter.
        NUMBER_OUT = 456

        Condition.add(MemoryCondition(self.runner.memory, 'memory.in', NUMBER_IN))
        Condition.add(MemoryCondition(self.runner.memory, 'memory.out', -1))

        class ValueLooperState(State):
            def __init__(self):
                State.__init__(self, ['succeeded', 'aborted'],
                               input_keys=['i', 'to_loop'], output_keys=['o'])

            def execute(self, userdata):
                print "%s found 'i' in userdata to be %s.." % (self.__class__.__name__, userdata.i)
                if userdata.i == NUMBER_IN:
                    print "..which is correct"
                    userdata.o = userdata.to_loop
                    #return 'succeeded' # will only work when using rgoap_ros.SMACHRunner
                else:
                    print "..which is not correct!"
                    #return 'aborted' # see above

        class TranslateAction(SMACHStateWrapperAction):
            def __init__(self):
                SMACHStateWrapperAction.__init__(self, ValueLooperState(),
                                                 [Precondition(Condition.get('memory.in'), NUMBER_IN),
                                                  Precondition(Condition.get('memory.out'), -1)],
                                                 [Effect(Condition.get('memory.out'), NUMBER_OUT)])

            def translate_worldstate_to_userdata(self, next_worldstate, userdata):
                userdata.i = next_worldstate.get_condition_value(Condition.get('memory.in'))
                userdata.to_loop = next_worldstate.get_condition_value(Condition.get('memory.out'))


            def translate_userdata_to_worldstate(self, userdata, next_worldstate):
                print "FIXME: translation from userdata does not work"
                # FIXME: translation from userdata does not work
#                # memory.set_value('memory.out', userdata.o)
#                next_worldstate.set_condition_value(Condition.get('memory.out'), userdata.o)


        self.runner.actionbag.add(TranslateAction())

        goal = Goal([Precondition(Condition.get('memory.out'), NUMBER_OUT),
                     # memory.in is added to goal to be available in goal/next_worldstate
                     Precondition(Condition.get('memory.in'), NUMBER_IN)])

        plan = self.runner.update_and_plan(goal)
        print "plan:", plan
        self.assertIsNotNone(plan, "plan should not be None")

        success = self.runner.execute(plan)
        print "success: ", success
        self.assertTrue(success, "executing plan was not successful")
        # self.assertEqual(outcome, 'succeeded', "executing plan did not return succeeded")

        mem_out = self.runner.memory.get_value('memory.out')
        print "memory.out = %s" % mem_out

        self.assertEqual(mem_out, NUMBER_OUT,
                         "out variable in memory is not changed successfully")






if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
