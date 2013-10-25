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

import tf

from geometry_msgs.msg import Pose, Point, Quaternion

from goap.common import *
from goap.inheriting import *
from goap.common_ros import *
from goap.runner import Runner

import goap.config_scitos as config_scitos

from uashh_smach.platform.move_base import position_tuple_to_pose



if __name__ == "__main__":

    rospy.init_node('goap_bumper_test', log_level=rospy.INFO)

    runner = Runner(config_scitos)

    Condition.add(MemoryCondition(runner.memory, 'memory.reminded_myself'))

    runner.memory.set_value('awareness', 0)
    runner.memory.set_value('arm_can_move', True)
    runner.memory.set_value('memory.reminded_myself', 333)

    print 'Waiting to let conditions represent reality...'
    print 'Remember to start topic publishers so conditions make sense instead of None!'
    rospy.sleep(2)
    Condition.initialize_worldstate(runner.worldstate)
    print 'worldstate now is: ', runner.worldstate

    runner.actionbag.add(MemoryChangeVarAction(runner.memory, 'memory.reminded_myself', 333, 555))


    goal = Goal([Precondition(Condition.get('robot.pose'), position_tuple_to_pose(1, 0, 0)),
                 Precondition(Condition.get('memory.reminded_myself'), 555)])

    start_node = runner.update_and_plan(goal, introspection=True)

    print 'start_node: ', start_node


    if start_node is None:
        print 'No plan found! Check your ROS graph!'
    else:
        runner.execute_as_smach(start_node, introspection=True)


    rospy.sleep(20)

