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


import roslib; roslib.load_manifest('rgoap_ros')
import rospy

import thread

from smach import UserData
from smach_ros import IntrospectionServer

from rgoap import Runner

from rgoap_ros import Introspector
from rgoap_smach import SMACHStateWrapperAction
from rgoap_smach import rgoap_path_to_smach_container


import logging
_logger = logging.getLogger('rgoap.ros')



## from uashh_smach.util import execute_smach_container
def execute_smach_container(smach_container, enable_introspection=False,
                            name='/SM_ROOT', userdata=UserData()):
    if not rospy.core.is_initialized():
        rospy.init_node('smach')

    if enable_introspection:
        # Create and start the introspection server
        sis = IntrospectionServer('smach_executor', smach_container, name)
        sis.start()

    outcome = smach_container.execute(userdata)
    _logger.info("smach outcome: %s", outcome)

    if enable_introspection:
        sis.stop()

    return outcome


class SMACHRunner(Runner):
    """
    This Runner subclass uses SMACH instead of the rgoap.PlanExecutor to
    execute an RGOAP plan.

    If enabled the smach viewer can be used for introspection.
    """

    def __init__(self, *args, **kwargs):
        Runner.__init__(self, *args, **kwargs)

        self._introspector = None
        self._current_smach = None # used to propagate preemption into generated smach


    def _setup_introspection(self):
        # init what could have been initialized externally
        if not rospy.core.is_initialized():
            rospy.init_node('rgoap_runner_introspector')
        # init everything else but only once
        if self._introspector is None:
            self._introspector = Introspector()
            thread.start_new_thread(rospy.spin, ())
            _logger.info("introspection spinner started")
        # TODO: check why spinner does not work [when runner called from unittest?]


    def request_preempt(self):
        Runner.request_preempt(self)
        if self._current_smach is not None:
            self._current_smach.request_preempt()

    def preempt_requested(self):
        return Runner.preempt_requested(self) or (self._current_smach.preempt_requested()
                                                   if self._current_smach is not None
                                                   else False)

    def service_preempt(self):
        Runner.service_preempt(self)
        if self._current_smach is not None:
            self._current_smach.service_preempt()


    def plan(self, goal, introspection=False):
        """plan for given goal and return start_node of plan or None

        introspection: introspect RGOAP planning via smach.introspection
        """
        if introspection:
            self._setup_introspection()

        start_node = Runner.plan(self, goal, introspection)

        if introspection:
            if start_node is not None:
                self._introspector.publish(start_node)
            self._introspector.publish_net(self.planner.last_goal_node, start_node)

        return start_node

    def plan_and_execute_goals(self, goals):
        """Sort goals by usability and try to plan and execute one by one until
        one goal is achieved"""
        self._setup_introspection()
        return Runner.plan_and_execute_goals(self, goals)


    def execute(self, start_node, introspection=False):
        outcome = self.execute_as_smach(start_node, introspection)
        return outcome

    def execute_as_smach(self, start_node, introspection=False):
        sm = rgoap_path_to_smach_container(start_node)
        # TODO: create proxies / userdata info for inner-sm introspection
        self._current_smach = sm
        outcome = execute_smach_container(sm, introspection, name='/RGOAP_GENERATED_SMACH')
        self._current_smach = None
        return outcome
