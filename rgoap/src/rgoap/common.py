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


import logging
_logger = logging.getLogger('rgoap')



def no_multilines(string):
    """anti-multiline workaround for ROS message types"""
    return string.replace('\n  ', ' ').replace(' \n', ' ').replace('\n', ' ')

def stringify_dict(dict_, delim=', '):
    return delim.join(['%s: %s' % (c, v) for (c, v) in dict_.iteritems()])

def stringify(iterable, delim=', '):
    return delim.join([str(e) for e in iterable])



class WorldState(object):
    """Storage for values of conditions.

    self._condition_values: Map<Condition, Object>
    """

    def __init__(self, worldstate=None):
        self._condition_values = {}
        if worldstate is not None:
            self._condition_values.update(worldstate._condition_values)

    def __str__(self):
        return '%s {%s}' % (self.__class__.__name__,
                            no_multilines(stringify_dict(self._condition_values)))

    def __repr__(self):
        return '<WorldState %X values=%s>' % (id(self), self._condition_values)
#        return '<WorldState>'

    def get_condition_value(self, condition):
        return self._condition_values[condition]

    def set_condition_value(self, condition, value):
        self._condition_values[condition] = value

    def matches(self, start_worldstate):
        """Return whether self is an 'equal subset' of start_worldstate."""
        start_ws_dict = start_worldstate._condition_values
        matches = True
        for (cond, value) in self._condition_values.viewitems():
            if cond in start_ws_dict:
                if not start_ws_dict[cond] == value:
                    matches = False
                    break
        _logger.debug('comparing worldstates: %s', matches)
#        _logger.debug('mine:  %s', self._condition_values)
#        _logger.debug('start: %s', start_ws_dict)
        return matches

    def get_state_name_dict(self):
        """Returns a dictionary with not the conditions themselves but
        their state_names as keys."""
        return {cond._state_name: val
                for cond, val in self._condition_values.viewitems()}

    def get_unsatisfied_conditions(self, worldstate):
        """Return a set of conditions that are in both the given and this
        worldstate but have unequal values. By now this is symmetric."""
        common_conditions_set = (self._condition_values.viewkeys() &
                                 worldstate._condition_values.viewkeys())
        unsatisfied_conditions = {condition
                                  for condition in common_conditions_set
                                  if (self.get_condition_value(condition) !=
                                      worldstate.get_condition_value(condition))}

        if _logger.isEnabledFor(logging.DEBUG):
            _logger.debug("unsatisfied conditions between world states: %d:\n%s",
                          len(unsatisfied_conditions),
                          '\n'.join([str(c) for c in unsatisfied_conditions]))

        return unsatisfied_conditions


## known as state
class Condition(object):
    """The object that makes any kind of robot or system state available.

    This class, at least its static part, is a multiton:
    * For each state_name only one instance is allowed to be in the
      _conditions_dict mapping.
    * If there is no mapping for a get(state_name) call an assertion is
      triggered, as creating a new instance makes no sense here.

    self._state_name: id name of condition, must not be changed
    """

    def __init__(self, state_name):
        assert state_name not in Condition._conditions_dict, \
            "Condition '" + state_name + "' had already been created previously!"
        self._state_name = state_name

    def __str__(self):
        return '%s:%s' % (self.__class__.__name__, self._state_name)

    def __repr__(self):
        return '<%s state_name=%s>' % (self.__class__.__name__, self._state_name)

    def get_value(self):
        """Returns the current value, hopefully not blocking."""
        raise NotImplementedError

    def _update_value(self, worldstate):
        """Update the condition's current value to the given worldstate."""
        worldstate.set_condition_value(self, self.get_value())



    _conditions_dict = {}

    @classmethod
    def add(cls, condition):
        assert condition._state_name not in cls._conditions_dict, \
            "Condition '" + condition._state_name + "' had already been added previously!"
        cls._conditions_dict[condition._state_name] = condition

    @classmethod
    def get(cls, state_name):
        assert state_name in cls._conditions_dict, "Condition '" + state_name + "' has not yet been added!"
        return cls._conditions_dict[state_name]

    @classmethod
    def print_dict(cls):
        return '<Conditions %s>' % cls._conditions_dict

    @classmethod
    def initialize_worldstate(cls, worldstate):
        """Initialize the given worldstate with all known conditions and their current values."""
        for condition in cls._conditions_dict.values():
            condition._update_value(worldstate)



class Precondition(object):

    def __init__(self, condition, value, deviation=None):
        self._condition = condition
        self._value = value
        self._deviation = deviation
        # TODO: make deviation relative/percental, not absolute

    def __str__(self):
        return '%s:%s=%s~%s' % (self.__class__.__name__, self._condition._state_name, self._value, self._deviation)

    def __repr__(self):
        return '<%s cond=%s value=%r dev=%s>' % (self.__class__.__name__, self._condition, self._value, self._deviation)

    def is_valid(self, worldstate):
        cond_value = worldstate.get_condition_value(self._condition)
        if self._deviation is None:
            return cond_value == self._value
        else:
            return abs(cond_value - self._value) <= self._deviation

    def apply(self, worldstate):
        # TODO: deviation gets lost in backwards planner
        worldstate.set_condition_value(self._condition, self._value)



class Effect(object):
    # TODO: think about optional deviation (coordinate with multiple action results)

    def __init__(self, condition, new_value):
        self._condition = condition
        self._new_value = new_value

    def __str__(self):
        return '%s:%s=%s' % (self.__class__.__name__, self._condition._state_name, self._new_value)

    def __repr__(self):
        return '<%s cond=%s new_val=%s>' % (self.__class__.__name__, self._condition._state_name, self._new_value)

    def apply_to(self, worldstate):
        # TODO: remove me as I'm only for forward planning?
        worldstate.set_condition_value(self._condition, self._new_value)

    def matches_condition(self, worldstate, start_worldstate):
        """Return whether this effect can reach worldstate from start_worldstate"""
        return worldstate.get_condition_value(self._condition) == self._new_value



class VariableEffect(object):
    """This variable effect can by default reach every value and therefore
    matches every worldstate.

    To make an effect reach not every possible value of its condition,
    subclass and override _is_reachable.
    """
    def __init__(self, condition):
#        Effect.__init__(self, condition, None)
        self._condition = condition

    def __str__(self):
        return '%s:%s' % (self.__class__.__name__, self._condition._state_name)

    def __repr__(self):
        return '<%s cond=%s>' % (self.__class__.__name__, self._condition._state_name)

#     def apply_to(self, worldstate):
        # TODO: remove me as I'm only for forward planning?
#         worldstate.memory.set_value(self._condition, self._new_value)

    def matches_condition(self, worldstate, start_worldstate):
        """Return whether this effect can reach worldstate from start_worldstate"""
        return self._is_reachable(worldstate.get_condition_value(self._condition),
                                  start_worldstate.get_condition_value(self._condition))

    def _is_reachable(self, value, start_value):
        """Return a Boolean whether this variable effect can reach the given
        value from the given start_value. If this effect can reach certain
        values from any value, the start_value just might be ignored.

        Defaults to True, subclass to limit variability.
        """
        # TODO: change reachability from boolean to float
        return True


class Goal(object):
    """
    usability range: from 0 to 1, defaults to 1
    """
    def __init__(self, preconditions, usability=1):
        self._preconditions = preconditions
        self.usability = usability

    def __str__(self):
        return '%s (usability=%s)' % (self.__class__.__name__, self.usability)

    def __repr__(self):
        return '<%s usability=%f preconditions=%s>' % (
                   self.__class__.__name__, self.usability, self._preconditions)

    def is_valid(self, worldstate):
        return all(precondition.is_valid(worldstate)
                   for precondition in self._preconditions)

    def apply_preconditions(self, worldstate):
        for precondition in self._preconditions:
            precondition.apply(worldstate)


# TODO: implement denial of trivial actions (not changing conditions), if they're actually concerned?

class Action(object):

    def __init__(self, preconditions, effects):
        self._preconditions = preconditions
        self._effects = effects

    def __str__(self):
        return self.__class__.__name__

    def __repr__(self):
        return '<%s preconditions=%s effects=%s>' % (self.__class__.__name__, self._preconditions, self._effects)

    def cost(self):
        """Return this action's cost value

        Override to apply own cost calculation. The returned cost must not be
        less than the number of declared effects!

        To check this when overriding use validate_cost:
            def cost(self):
                return self.validate_cost(..custom cost calculation..)
        """
        return len(self._effects)

    def validate_cost(self, cost):
        """Can be used to validate custom cost calculations, see cost()"""
        minimum_cost = len(self._effects)
        if cost < minimum_cost:
            _logger.error("Warning: action %s proposed too small cost (%s), "
                          "overriding with minimum cost (%s)",
                          self.__class__.__name__, cost, minimum_cost)
            cost = minimum_cost
        return cost


    def run(self, next_worldstate):
        """
        next_worldstate: the worldstate that this action should lead to when run
        """
        raise NotImplementedError


    ## following for executor

    def is_valid(self, worldstate):
        """Return whether this action is applicable from the given worldstate
        on, i.e. all preconditions are valid."""
        return all(precondition.is_valid(worldstate)
                   for precondition in self._preconditions)


    ## following was for forward planner

    def apply_effects(self, worldstate):
        # TODO: remove me as I'm only for forward planning?
        for effect in self._effects:
            effect.apply_to(worldstate)


    ## following for backward planner

    def check_freeform_context(self):
        """Override to add context checks required to run this action that cannot be satisfied by the planner."""
        return True

    def has_satisfying_effects(self, worldstate, start_worldstate, unsatisfied_conditions):
        """Return True if at least one of own effects matches unsatisfied_conditions."""
        for effect in self._effects:
            if effect._condition in unsatisfied_conditions: # TODO: maybe put this check into called method // no, would make the return value trilateral
                if effect.matches_condition(worldstate, start_worldstate):
                    return True
        return False

    def apply_preconditions(self, worldstate, start_worldstate):
        """
        worldstate: the worldstate to apply this action's preconditions to
        start_worldstate: needed to let actions optimize their variable precondition parameters
        """
        # TODO: make required derivation of variable actions more obvious and fail-safe
        for precondition in self._preconditions:
            precondition.apply(worldstate)

        # let the action generate preconditions for its variable effects
        var_effects = [effect for effect in self._effects if isinstance(effect, VariableEffect)]
        if len(var_effects) > 0:
            g = self._generate_variable_preconditions(var_effects, worldstate, start_worldstate)
            for precondition in g:
                precondition.apply(worldstate)

    def _generate_variable_preconditions(self, var_effects, worldstate, start_worldstate):
        """
        Let the action itself generate variable preconditions for its variable effects.

        Must be implemented if the action contains variable effects.
        """
        # TODO: maybe implement a default behaviour, at least for variable effects that can reach any value
        for effect in self._effects:
            if isinstance(effect, VariableEffect):
                raise NotImplementedError


