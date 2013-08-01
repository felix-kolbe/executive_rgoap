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


from goap import *


class Memory(object):
    """ATM a class to store condition data not representing real world."""

    def __init__(self):
        self._memory = {}

    def __repr__(self):
        return '<Memory %s>' % self._memory

    def declare_variable(self, name, value=None):
        if name not in self._memory:
            self._memory[name] = value

    def get_value(self, name):
        return self._memory[name]

    def set_value(self, name, value):
        self._memory[name] = value

#    def matches(self, memory):
#        for (k, v) in self._memory.iteritems():
#            if k in memory._memory and memory._memory[k] != v:
#                return False
#        return True


#     def __call__(self):
#         return self
# NOTE: cleanup Memory singleton
# Memory = Memory()


#class MemoryAction(Action):
#
#    def __init(self, memory, preconditions, effects):
#        Action.__init__(self, preconditions, effects)
#        self._memory = memory

class MemorySetVarAction(Action):

    def __init__(self, memory, variable, new_value, preconditions, effects):
        Action.__init__(self, preconditions, effects)
        self._memory = memory
        self._variable = variable
        self._new_value = new_value
        self._state_name = 'memory.' + self._variable
        self._memory.declare_variable(self._variable)

    def run(self, next_worldstate):
        self._memory.set_value(self._variable, self._new_value)


class MemoryChangeVarAction(MemorySetVarAction):

    def __init__(self, memory, variable, old_value, new_value):
        MemorySetVarAction.__init__(self, memory, variable, new_value,
                [Precondition(Condition.get('memory.' + variable), old_value)],
                [Effect(Condition.get('memory.' + variable), new_value)]
            )
        self._old_value = old_value

    def __repr__(self):
        return '<MemoryChangeVarAction var=%s old=%s new=%s>' % (self._variable, self._old_value, self._new_value)


class MemoryIncrementerAction(Action):

    class IncEffect(VariableEffect):
        def __init__(self, condition):
            VariableEffect.__init__(self, condition)
        def _is_reachable(self, value):
            return True # TODO: change reachability from boolean to float


    def __init__(self, memory, variable, increment=1):
        self._condition = Condition.get('memory.' + variable)
        Action.__init__(self, [], [MemoryIncrementerAction.IncEffect(self._condition)])
        self._memory = memory
        self._variable = variable
        self._increment = increment
        self._memory.declare_variable(self._variable)

    def __repr__(self):
        return '<MemoryIncrementerAction var=%s incr=%s>' % (self._variable, self._increment)

    def cost(self):
        return abs(self._increment)

    def run(self, next_worldstate):
        self._memory.set_value(self._variable, self._memory.get_value(self._variable) + self._increment)

    def apply_preconditions(self, worldstate, start_worldstate):
        # calculate an ad hoc precondition for our variable effect and apply it
        effect_value = worldstate.get_condition_value(self._condition)
        precond_value = self._calc_preconditional_value(worldstate, start_worldstate, effect_value)
        Precondition(self._condition, precond_value, None).apply(worldstate)

    def _calc_preconditional_value(self, worldstate, start_worldstate, effect_value):
        return effect_value - self._increment



class MemoryCondition(Condition):
    """The state_name of a memory condition is memory.<variable>"""

    def __init__(self, memory, variable):
        Condition.__init__(self, 'memory.' + variable)
        self._memory = memory
        self._variable = variable
        memory.declare_variable(self._state_name)

    def __repr__(self):
        return '<MemoryCondition var=%s>' % self._variable

    def get_value(self):
        return self._memory.get_value(self._state_name)
