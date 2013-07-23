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


from goap import ActionBag, WorldState

from copy import deepcopy


class Node(object):

    def __init__(self, worldstate, parent_nodes_path_list, parent_actions_path_list):
        self.worldstate = worldstate
        self.parent_nodes_path_list = parent_nodes_path_list
        self.parent_actions_path_list = parent_actions_path_list

        self.valid_actionbag = ActionBag()

    def __repr__(self):
        return '<Node worldstate=%s valid_actions=%s>' % (self.worldstate, self.valid_actionbag)

#     def _check_and_add_actions(self, actionbag):
#         for action in actionbag.generate_matching_actions():
#              # TODO use difference subset
# #         for action in actionbag.get():
# #             if action.has_matching_effects(self.worldstate):
# #                 print 'helping action: ', action
#             self.valid_actionbag.add(action)
# #             else:
# #                 print 'helpless action: ', action

    def get_child_nodes_for_valid_actions(self, actionbag):
#         self._check_and_add_actions(actionbag)
        self.valid_actionbag = actionbag
        nodes = []
        for action in self.valid_actionbag:
            nodes_path_list = self.parent_nodes_path_list[:]
            nodes_path_list.append(self)
            actions_path_list = self.parent_actions_path_list[:]
            actions_path_list.append(action)
            worldstatecopy = deepcopy(self.worldstate)
            worldstatecopy.apply_effects(action)
            action.apply_preconditions(worldstatecopy)
            node = Node(worldstatecopy, nodes_path_list, actions_path_list) # copy worldstate
            nodes.append(node)
        return nodes



class Planner(object):

    def __init__(self, actionbag, worldstate, goal):
        self._actionbag = actionbag
        self._start_worldstate = worldstate
        self._goal = goal

    def plan(self):
        print 'start_worldstate: ', self._start_worldstate

        print 'goal: ', self._goal

        goal_worldstate = WorldState()

        self._goal.apply_preconditions(goal_worldstate)
        print 'goal_worldstate: ', goal_worldstate

        goal_node = Node(goal_worldstate, [], [])
        print 'goal_node: ', goal_node

#         worldstate = self._start_worldstate

        child_nodes = {goal_node}

#         while not self._goal.is_valid(worldstate):
        count = 0
        while len(child_nodes) != 0:
            count += 1
            if count >= 10:
                break

            print "=Doing another planning loop="
            print 'nodes: ', child_nodes

            current_node = child_nodes.pop()
            print 'popping this: ', current_node
#             print 'nodes: ', child_nodes, len(child_nodes)

            print 'current_node.worldstate: ', current_node.worldstate

            if self._start_worldstate.matches(current_node.worldstate):
                print "Found plan!"
                print 'plan nodes: ', current_node.parent_nodes_path_list
                print 'plan actions: ', current_node.parent_actions_path_list
                return current_node.parent_actions_path_list

            #current_node.check_and_add_actions(self._actionbag)
            print "Current node: ", current_node

            new_child_nodes = current_node.get_child_nodes_for_valid_actions(
                    self._actionbag.generate_matching_actions(self._start_worldstate, current_node.worldstate))
            print 'new child nodes: ', new_child_nodes

            child_nodes.update(new_child_nodes)


        return None

