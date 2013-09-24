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


from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from metralabs_msgs.msg import ScitosG5Bumper


from common_ros import MoveBaseAction, ResetBumperAction, ROSTopicCondition
from inheriting import MemoryCondition
from smach_bridge import LookAroundAction, FoldArmAction, ARM_FOLDED_POSE_NAMED


def get_all_conditions(memory):
    return [
        # memory
        MemoryCondition(memory, 'arm_can_move', True),
        MemoryCondition(memory, 'awareness'),
        # ROS
        ROSTopicCondition('robot.pose', '/odom', Odometry, '/pose/pose'),
        ROSTopicCondition('robot.bumpered', '/bumper', ScitosG5Bumper, '/motor_stop'),
        ROSTopicCondition('robot.arm_folded', '/joint_states', JointState,
                          msgeval=lambda msg: all([abs(ARM_FOLDED_POSE_NAMED[name] -
                                                       position) < 0.01
                                                   for (name, position)
                                                   in zip(msg.name, msg.position)
                                                   if name in ARM_FOLDED_POSE_NAMED]
                                                  ))
        ]


def get_all_actions(memory):
    return [
        # memory
        # ROS - pure actions
        ResetBumperAction(),
        # ROS - wrapped SMACH states
        MoveBaseAction(),
        LookAroundAction(),
        FoldArmAction()
        ]

