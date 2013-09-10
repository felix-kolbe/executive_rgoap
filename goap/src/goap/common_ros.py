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


import roslib; roslib.load_manifest('goap')
import rospy

import rostopic
import actionlib
import tf


from std_msgs.msg import Empty

from move_base_msgs.msg import MoveBaseGoal
import move_base_msgs.msg # for MoveBaseAction, preventing name duplicates
from geometry_msgs.msg import Pose, Point, Quaternion

from common import Action, Condition, Precondition, Effect, VariableEffect


## ROS specific class specializations

class ROSTopicCondition(Condition):

    def __init__(self, state_name, topic, topic_class, field):
        Condition.__init__(self, state_name)
        self._topic = topic
        self._field = field
        self._subscriber = rospy.Subscriber(topic, topic_class, self._callback)
        self._msgeval = rostopic.msgevalgen(field)
        self._value = None

    def __repr__(self):
        return '<%s topic=%s field=%s>' % (self.__class__.__name__, self._topic, self._field)

    def _callback(self, msg):
        self._value = self._msgeval(msg)
#        print 'callback with: ', self._value

    def get_value(self):
        return self._value


## concrete usable classes (no constructor parameters anymore)

class ResetBumperAction(Action):

    def __init__(self):
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), True)],  # TODO: Precondition fails if message wasn't received yet
                            [Effect(Condition.get('robot.bumpered'), False)])
        self._publisher = rospy.Publisher('/bumper_reset', Empty)

    def check_freeform_context(self):
        return self._publisher.get_num_connections() > 0  # unsafe

    def run(self, next_worldstate):
        print 'num of subscribers: ', self._publisher.get_num_connections()
        print 'sending bumper_reset message..'
        self._publisher.publish(Empty())
        rospy.sleep(1)  # TODO: find solution without sleep

# TODO: implement denial of trivial actions (not changing conditions)

class MoveBaseAction(Action):

    class PositionVarEffect(VariableEffect):
        def __init__(self, condition):
            VariableEffect.__init__(self, condition)
        def _is_reachable(self, value):
            return True

    def __init__(self):
        self._condition = Condition.get('robot.pose')
        super(MoveBaseAction, self).__init__(
                        [Precondition(Condition.get('robot.bumpered'), False)],
                        [MoveBaseAction.PositionVarEffect(self._condition)])

        self._client = actionlib.SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)

    def check_freeform_context(self):
        # TODO: cache freeform context?
        return self._client.wait_for_server(rospy.Duration(0.1))

    def apply_adhoc_preconditions_for_vareffects(self, var_effects, worldstate, start_worldstate):
        effect = var_effects.pop()  # this action has one variable effect
        assert effect.__class__ == MoveBaseAction.PositionVarEffect
        precond_value = start_worldstate.get_condition_value(Condition.get('robot.pose'))
        Precondition(effect._condition, precond_value, None).apply(worldstate)

    def run(self, next_worldstate):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "/map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = next_worldstate.get_condition_value(Condition.get('robot.pose'))

        print 'Waiting for base to reach goal...'
        goalstatus = self._client.send_goal_and_wait(goal)
        print 'Goal status: ', goalstatus
