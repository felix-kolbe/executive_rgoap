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

from planning import Node

class GOAPIntrospection(object):
    '''
    classdocs
    '''

    def __init__(self):
        self._pathprefix = '/GOAP_PLAN'
        self._pathprefix_net = '/GOAP_NET'
        self._publisher_structure = rospy.Publisher(self._pathprefix + STRUCTURE_TOPIC, SmachContainerStructure, latch=True)
        self._publisher_status = rospy.Publisher(self._pathprefix + STATUS_TOPIC, SmachContainerStatus, latch=True)
        self._publisher_structure_net = rospy.Publisher(self._pathprefix_net + STRUCTURE_TOPIC, SmachContainerStructure, latch=True)
        self._publisher_status_net = rospy.Publisher(self._pathprefix_net + STATUS_TOPIC, SmachContainerStatus, latch=True)


    def _add_nodes_recursively(self, node, structure):
        """node: beginning with the start node"""
        structure.children.append(self._nodeid(node))

        if len(node.parent_nodes_path_list) > 0:
            next_node = node.parent_nodes_path_list[-1]

            structure.internal_outcomes.append(str(node.action._effects))
            structure.outcomes_from.append(self._nodeid(node))
            structure.outcomes_to.append(self._nodeid(next_node))
#
#            structure.internal_outcomes.append('aborted')
#            structure.outcomes_from.append(self._nodeid(node))
#            structure.outcomes_to.append('None')

            self._add_nodes_recursively(next_node, structure)
#        else: # goal node
#            structure.internal_outcomes.append('succeeded')
#            structure.outcomes_from.append(self._nodeid(node))
#            structure.outcomes_to.append('None')


    def _add_nodes_recursively_net(self, node, structure):
        """node: beginning with the goal node"""
        structure.children.append(self._nodeid(node))

        if len(node.possible_prev_nodes) > 0: # goal or inner node
            for prev_node in node.possible_prev_nodes:
                structure.internal_outcomes.append(str(prev_node.action._effects))
                structure.outcomes_from.append(self._nodeid(prev_node))
                structure.outcomes_to.append(self._nodeid(node))
                self._add_nodes_recursively_net(prev_node, structure)
        else: # start or dead-end node
            pass

#        if len(node.parent_nodes_path_list) == 0: # goal node
#            structure.internal_outcomes.append('succeeded')
#            structure.outcomes_from.append(self._nodeid(node))
#            structure.outcomes_to.append('None')
#        else:
#            structure.internal_outcomes.append('aborted')
#            structure.outcomes_from.append(self._nodeid(node))
#            structure.outcomes_to.append('None')


    def publish_net(self, start_node, goal_node):
        structure = SmachContainerStructure()
        structure.header.stamp = rospy.Time.now()
        structure.path = self._pathprefix_net
#        structure.container_outcomes = ['succeeded', 'aborted']
        self._add_nodes_recursively_net(goal_node, structure)
        self._publisher_structure_net.publish(structure)

        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix_net
        status.initial_states = [self._nodeid(start_node)]
        #status.active_states = ['None']
        status.active_states = [self._nodeid(goal_node)]
        status.local_data = pickle.dumps(goal_node.worldstate.get_state_name_dict(), 2)
        status.info = 'initial state'
        self._publisher_status_net.publish(status)

    def publish(self, start_node):
        structure = SmachContainerStructure()
        structure.header.stamp = rospy.Time.now()
        structure.path = self._pathprefix
#        structure.container_outcomes = ['succeeded', 'aborted']
        self._add_nodes_recursively(start_node, structure)
        self._publisher_structure.publish(structure)

        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix
        status.initial_states = [self._nodeid(start_node)]
        status.active_states = ['None']
        status.local_data = pickle.dumps(start_node.worldstate.get_state_name_dict(), 2)
        status.info = 'initial state'
        self._publisher_status.publish(status)

#        for node in start_node.parent_nodes_path_list:
#            print node
#            self.publish_update(node)

    def publish_update(self, node):
        nodeid = self._nodeid(node)
        status = SmachContainerStatus()
        status.header.stamp = rospy.Time.now()
        status.path = self._pathprefix
        status.initial_states = []
        status.active_states = [nodeid]
#        status.local_data = pickle.dumps(self._worldstate_to_userdata(start_node.worldstate)._data, 2)
        status.local_data = pickle.dumps(node.worldstate.get_state_name_dict(), 2)
        status.info = 'node state'
        self._publisher_status.publish(status)


    def _nodeid(self, node):
        if node.action is None:
            node_action_name = 'GOAL'
            node_action_cost = '-'
        else:
            node_action_name = str(node.action) #.__class__.__name__
            node_action_cost = node.action.cost()
        return node_action_name + ' %x' % id(node) + ' ' + str(node_action_cost) + ' ' + str(node.cost())
