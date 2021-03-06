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


"""
ROS specific specializations of rgoap classes
"""


import roslib; roslib.load_manifest('rgoap_ros')
import rospy
import rostopic

from rgoap import Condition, Action


import logging
_logger = logging.getLogger('rgoap.ros')



class ROSTopicCondition(Condition):
    """Mirrors a ROS message field of a topic as its value.
    Note that this Condition's value remains None until a message is received.
    """
    def __init__(self, state_name, topic, topic_class, field=None, msgeval=None):
        Condition.__init__(self, state_name)
        self._topic = topic
        self._field = field
        self._subscriber = rospy.Subscriber(topic, topic_class, self._callback)
        if msgeval is None:
            assert field is not None
            msgeval = rostopic.msgevalgen(field)
        self._msgeval = msgeval

        self._value = None

    def __repr__(self):
        return '<%s topic=%s field=%s>' % (self.__class__.__name__, self._topic, self._field)

    def _callback(self, msg):
        self._value = self._msgeval(msg)
#        _logger.debug("callback with: %s", self._value)

    def get_value(self):
        return self._value


class ROSTopicAction(Action):

    def __init__(self, topic, topic_class,
                 preconditions, effects,
                 msg_args=None, msg_cb=None, **kwargs):
        """
        :param topic_class: Message type, ``Class``
        :param msg_args: Arguments to initialize message that is published, ``[val]``
        :param msg_cb: callback to replace the default _msg_cb, useful with lambda expressions
        """
        assert msg_args is None or msg_cb is None, \
                    "Only either msg_args or msg_cb may be given, not both"
        Action.__init__(self, preconditions, effects, **kwargs)
        self._topic = topic
        self._topic_class = topic_class
        self._msg_args = msg_args
        if msg_cb is not None:
            self._msg_cb = msg_cb
        self._publisher = rospy.Publisher(topic, topic_class, **kwargs)

    def __str__(self):
        s = '%s:%s' % (self.__class__.__name__, self._topic)
        if self._msg_args is not None:
            s += '=%s' % self._msg_args
        return s

    def _msg_cb(self, message, next_worldstate):
        """Fill and return the given (or a created) message as desired

        Must be implemented and is concerned only if neither msg_args nor msg_cb is given.
        """
        raise NotImplementedError

    def check_freeform_context(self):
        return self._publisher.get_num_connections() > 0  # unsafe

    def run(self, next_worldstate):
        _logger.debug("num of subscribers to %s: %d",
                      self.__class__.__name__,
                      self._publisher.get_num_connections())
        if self._msg_args is None:
            _logger.info("%s publishes message..", self)
            self._publisher.publish(self._msg_cb(self._topic_class(), next_worldstate))
            rospy.sleep(1)  # TODO: find solution without sleep
        else:
            _logger.info("%s publishes message via rostopic..", self)
            rostopic.publish_message(self._publisher, self._topic_class, self._msg_args, once=True)
            # rostopic sleeps itself

        # TODO: integrate check for asynchronous action bodys
