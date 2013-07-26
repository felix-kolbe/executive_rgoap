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


from numpy.core.numeric import NaN



class Memory(object):

    def __init__(self):
        self._memory = {}

    def __repr__(self):
        return '<Memory: %s>' % self._memory

    def declare_variable(self, name, value=None):
        if name not in self._memory:
            self._memory[name] = value

    def get_value(self, name):
        return self._memory[name]

    def set_value(self, name, value):
        self._memory[name] = value

    def matches(self, memory):
        for (k, v) in self._memory.iteritems():
            if k in memory._memory and memory._memory[k] != v:
                return False
        return True


#     def __call__(self):
#         return self
# NOTE: cleanup Memory singleton
# Memory = Memory()


#class WorldState(object):
#
#    def __init__(self, memory=Memory()):
#        self._memory = memory
#
#    def __repr__(self):
#        return '<WorldState memory=%s>' % self._memory
#
#
#    def declare_variable(self, name, value=None):
#        if name not in self._memory:
#            self._memory[name] = value
#
#    def get_value(self, name):
#        return self._memory[name]
#
#    def set_value(self, name, value):
#        self._memory[name] = value
#
#
#    def matches(self, worldstate):
#        return self.memory.matches(worldstate.memory)
#
#        for (k, v) in self._memory.iteritems():
#            if k in worldstate._memory and worldstate._memory[k] != v:
#                return False
#        return True
#
#    def apply_effects(self, action):
#        action.apply_effects(self)


class WorldState(object):

    def __init__(self, memory=Memory()):
        self.memory = memory

    def __repr__(self):
        return '<WorldState memory=%s>' % self.memory

    def matches(self, worldstate):
        return self.memory.matches(worldstate.memory)

#    def apply_effects(self, action): # TODO: replace by direct calls to action.apply_effects()
# delete me       action.apply_effects(self)




## known as state
class Condition(object):

    # TODO: maybe convert to singleton
    def __init__(self, state_name):
        self._state_name = state_name

    def get_value(self, worldstate):
        raise NotImplementedError

    def set_value(self, worldstate, value):
        raise NotImplementedError


    _conditions_dict = {}

    @classmethod
    def add(cls, name, condition):
        assert name not in cls._conditions_dict, "Condition '" + name + "' had already been added previously!"
        cls._conditions_dict[name] = condition

    @classmethod
    def get(cls, name):
        assert name in cls._conditions_dict, "Condition '" + name + "' has not yet been added!"
        return cls._conditions_dict[name]

    @classmethod
    def print_dict(cls):
        return '<Conditions: %s>' % cls._conditions_dict



class Precondition(object):

    def __init__(self, condition, value, deviation=None):
        self._condition = condition
        self._value = value
        self._deviation = deviation

    def __repr__(self):
        return '<Precondition cond=%s value=%s dev=%s>' % (self._condition, self._value, self._deviation)

    def is_valid(self, worldstate):
        if self._deviation is None:
            return self._condition.get_value(worldstate) == self._value
        else:
            return abs(self._condition.get_value(worldstate) - self._value) <= self._deviation

    def apply(self, worldstate):
        # TODO: deviation gets lost in backwards planner
        self._condition.set_value(worldstate, self._value)



class Effect(object):
    # TODO: integrate conditions beside memory
    # TODO: think about optional deviation

    def __init__(self, condition, new_value):
        self._condition = condition
        self._new_value = new_value

    def apply_to(self, worldstate):
        self._condition.set_value(worldstate, self._new_value)

    def matches_condition(self, worldstate):
        return self._condition.get_value(worldstate) == self._new_value



class VariableEffect(object):

    def __init__(self, condition):
#        Effect.__init__(self, condition, None)
        self._condition = condition

#     def apply_to(self, worldstate):
#         worldstate.memory.set_value(self._condition, self._new_value)

    def matches_condition(self, worldstate):
        return self._is_reachable(self._condition.get_value(worldstate))

    def _is_reachable(self, value):
        raise NotImplementedError


class Goal(object):

    def __init__(self, preconditions):
        self._preconditions = preconditions

    def __repr__(self):
        return '<Goal preconditions=%s>' % self._preconditions

    def is_valid(self, worldstate):
        for precondition in self._preconditions:
            if not precondition.is_valid(worldstate):
                return False
        return True

    def apply_preconditions(self, worldstate):
        for precondition in self._preconditions:
            precondition.apply(worldstate)



class Action(object):

    def __init__(self, preconditions, effects):
        self._preconditions = preconditions
        self._effects = effects

    def __repr__(self):
        return '<Action type=%s>' % self.__class__.__name__

    def run(self):
        raise NotImplementedError

    ## following two for forward planner

    def is_valid(self, worldstate):
        for precondition in self._preconditions:
            if not precondition.is_valid(worldstate):
                return False
        return True

    def apply_effects(self, worldstate):
        for effect in self._effects:
            effect.apply_to(worldstate)

    ## following two for backward planner

    def check_freeform_context(self):
        """Override to add context checks required to run this action but cannot be satisfied by the planner."""
        return True

    def has_matching_effects(self, worldstate, unsatisfied_states_key_set): # TODO: add difference subset
        # TODO
        for effect in self._effects:
            if effect._condition._state_name in unsatisfied_states_key_set:
                if effect.matches_condition(worldstate):
                    return True
        return False

    def apply_preconditions(self, worldstate):
        # TODO: make required derivation of variable actions more obvious and fail-safe
        for precondition in self._preconditions:
            precondition.apply(worldstate)



class ActionBag(object):

    def __init__(self):
        self._actions = set()

    def __repr__(self):
        return '<ActionBag %s>' % self._actions

    def add(self, action):
        self._actions.add(action)

    def get(self):
        return self._actions

    def get_sorted(self): # TODO: sort by costs
#         self._actions.sort(cmp=None, key=None, reverse=False)
        return self.get()

    def generate_matching_actions(self, start_worldstate, node_worldstate):
        # TODO

        common_states_key_set = start_worldstate.memory._memory.viewkeys() & node_worldstate.memory._memory.viewkeys()
        unsatisfied_states_key_set = set()
        for key in common_states_key_set:
            if start_worldstate.memory._memory[key] != node_worldstate.memory._memory[key]:
                print 'state different: ', key
                unsatisfied_states_key_set.add(key)
            else:
                print 'state equal: ', key
        # TODO: move that to worldstate

        for action in self._actions:
            if action.has_matching_effects(node_worldstate, unsatisfied_states_key_set):
                print 'helping action: ', action
                yield action
            else:
                print 'helpless action: ', action



