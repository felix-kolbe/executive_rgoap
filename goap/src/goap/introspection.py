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


import rospy
import pickle

from smach_msgs.msg import SmachContainerStatus, SmachContainerStructure
from smach_ros.introspection import STATUS_TOPIC, STRUCTURE_TOPIC


class Introspector(object):
    """Gives insight to a planner's plan and GOAP net by publishing prepared
    information to a smach_viewer.
    """
    def __init__(self):
        self._pathprefix = '/GOAP_PLAN'
        self._pathprefix_net = '/GOAP_NET'
        self._publisher_structure = rospy.Publisher(self._pathprefix + STRUCTURE_TOPIC, SmachContainerStructure, latch=True)
        self._publisher_status = rospy.Publisher(self._pathprefix + STATUS_TOPIC, SmachContainerStatus, latch=True)
        self._publisher_structure_net = rospy.Publisher(self._pathprefix_net + STRUCTURE_TOPIC, SmachContainerStructure, latch=True)
        self._publisher_status_net = rospy.Publisher(self._pathprefix_net + STATUS_TOPIC, SmachContainerStatus, latch=True)


    def publish_net(self, goal_node, start_node=None):
        """Publishes a GOAP planning net, reconstructing it from with the goal node.

        The goal node will be set as the active state, as its userdata is displayed.
        """
        def _add_nodes_recursively(node, structure):
            """node: beginning with the goal node"""
            structure.children.append(self._nodeid(node))

            if len(node.possible_prev_nodes) > 0: # goal or inner node
                for prev_node in node.possible_prev_nodes:
                    structure.internal_outcomes.append('\n'.join(str(e) for e in prev_node.action._effects))
                    structure.outcomes_from.append(self._nodeid(prev_node))
                    structure.outcomes_to.append(self._nodeid(node))
                    _add_nodes_recursively(prev_node, structure)
            else: # start or dead-end node
                pass

#            if len(node.parent_nodes_path_list) == 0: # goal node
#                structure.internal_outcomes.append('succeeded')
#                structure.outcomes_from.append(self._nodeid(node))
#                structure.outcomes_to.append('None')
#            else:
#                structure.internal_outcomes.append('aborted')
#                structure.outcomes_from.append(self._nodeid(node))
#                structure.outcomes_to.append('None')

        structure = SmachContainerStructure()
        structure.header.stamp = rospy.Time.now()
        structure.path = self._pathprefix_net
#        structure.container_outcomes = ['succeeded', 'aborted']
        _add_nodes_recursively(goal_node, structure)
        self._publisher_structure_net.publish(structure)
        print 'published net has ~%s nodes' % len(structure.children)

        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix_net
        status.initial_states = [self._nodeid(start_node) if start_node is not None else 'No plan found']
        status.active_states = [self._nodeid(goal_node)]
        goal_states_dict = goal_node.worldstate.get_state_name_dict()
        goal_states_dict['WORLDSTATE'] = 'GOAL'
        status.local_data = pickle.dumps(goal_states_dict, 2)
        status.info = 'goal state'
        self._publisher_status_net.publish(status)

        rospy.sleep(5)

    def publish(self, start_node):
        """Publishes a planned GOAP plan

        The start node will be set as the active state, as its userdata is displayed.
        """
        def _add_nodes_recursively(node, structure):
            """node: beginning with the start node"""
            structure.children.append(self._nodeid(node))

            if not node.is_goal():
                next_node = node.parent_node()

                structure.internal_outcomes.append('\n'.join(str(e) for e in node.action._effects))
                structure.outcomes_from.append(self._nodeid(node))
                structure.outcomes_to.append(self._nodeid(next_node))
#
#                structure.internal_outcomes.append('aborted')
#                structure.outcomes_from.append(self._nodeid(node))
#                structure.outcomes_to.append('None')

                _add_nodes_recursively(next_node, structure)
#            else: # goal node
#                structure.internal_outcomes.append('succeeded')
#                structure.outcomes_from.append(self._nodeid(node))
#                structure.outcomes_to.append('None')

        structure = SmachContainerStructure()
        structure.header.stamp = rospy.Time.now()
        structure.path = self._pathprefix
#        structure.container_outcomes = ['succeeded', 'aborted']
        _add_nodes_recursively(start_node, structure)
        self._publisher_structure.publish(structure)
        print 'published plan has ~%s nodes' % len(structure.children)

        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix
        status.initial_states = [self._nodeid(start_node)]
        status.active_states = [self._nodeid(start_node)]
        start_states_dict = start_node.worldstate.get_state_name_dict()
        start_states_dict['WORLDSTATE'] = 'START'
        status.local_data = pickle.dumps(start_states_dict, 2)
        status.info = 'initial state'
        self._publisher_status.publish(status)

#        for node in start_node.parent_nodes_path_list:
#            print "publishing update for node: ", node
#            self.publish_update(node)

    def publish_update(self, node):
        nodeid = self._nodeid(node)
        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix + '/' + nodeid
        status.initial_states = []
        status.active_states = [self._nodeid(node)]
        start_states_dict = node.worldstate.get_state_name_dict()
        start_states_dict['WORLDSTATE'] = str(node) # id?
        status.local_data = pickle.dumps(start_states_dict, 2)
        status.info = 'node state'
        self._publisher_status.publish(status)


    def _nodeid(self, node):
        if node.is_goal():
            node_action_name = 'GOAL'
        else:
            node_action_name = str(node.action) #.__class__.__name__
        return (node_action_name + ' %X' % id(node) +
                ' n%s' % node.cost() +
                ' p%s' % node.path_cost() +
                ' h%s' % node.heuristic_distance +
                ' t%s' % node.total_cost())
