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

from std_msgs.msg import Empty

from goap import Action, Condition, Precondition, Effect, VariableEffect


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
        return '<ROSTopicCondition topic=%s field=%s>' % (self._topic, self._field)

    def _callback(self, msg):
        self._value = self._msgeval(msg)
        print 'callback with: ', self._value

    def get_value(self):
        return self._value


## concrete usable classes (no constructor parameters anymore)

class ResetBumperAction(Action):

    def __init__(self):
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), True)],
                            [Effect(Condition.get('robot.bumpered'), False)])

    def run(self):
        rospy.Publisher('/bumper_reset', Empty).publish()


class MoveBaseAction(Action):

    class PositionEffect(VariableEffect):
        def __init__(self):
            VariableEffect.__init__(Condition.get('robot.position'))
        def _is_reachable(self, value):
            return True # TODO: change reachability from boolean to float

    def __init__(self):
        Action.__init__(self, [Precondition(Condition.get('robot.bumpered'), False)],
                        [MoveBaseAction.PositionEffect()])

    def run(self):
#        MoveBaseState() TODO
        pass
