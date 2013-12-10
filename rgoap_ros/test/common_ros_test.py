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

from rgoap import Condition, Precondition, Effect, Memory, MemoryCondition
from rgoap_ros import common_ros

from std_msgs.msg import Bool
import rospy



class Test(unittest.TestCase):

    topic = '/testbool'

    def setUp(self):
        rospy.init_node(self.__class__.__name__)

        Condition._conditions_dict.clear() # start every test without previous conditions
        memory = Memory()

        self.rtc = common_ros.ROSTopicCondition('topic.testbool', Test.topic, Bool, '/data')
        Condition.add(self.rtc)
        Condition.add(MemoryCondition(memory, 'memory.anyvar'))
        print self.rtc



        self.rta_true = common_ros.ROSTopicAction(Test.topic, Bool,
                                             [Precondition(Condition.get('memory.anyvar'), 42)],
                                             [Effect('topic.testbool', True)],
                                             msg_args=[True])
        self.rta_false = common_ros.ROSTopicAction(Test.topic, Bool,
                                             [Precondition(Condition.get('memory.anyvar'), 42)],
                                             [Effect('topic.testbool', False)],
                                             msg_args=[False])

        def msg_cb(message, next_worldstate, value):
            message.data = value
            return message
        self.rta_cb_true = common_ros.ROSTopicAction(Test.topic, Bool,
                                             [Precondition(Condition.get('memory.anyvar'), 42)],
                                             [Effect('topic.testbool', True)],
                                             msg_cb=lambda msg, ws: msg_cb(msg, ws, True))
        self.rta_cb_false = common_ros.ROSTopicAction(Test.topic, Bool,
                                             [Precondition(Condition.get('memory.anyvar'), 42)],
                                             [Effect('topic.testbool', False)],
                                             msg_cb=lambda msg, ws: msg_cb(msg, ws, False))

    def tearDown(self):
        pass


    def testRTC(self):
        publisher = rospy.Publisher(Test.topic, Bool, latch=True)
        rospy.sleep(2) # let subscribers find publishers
        self.assertIsNone(self.rtc.get_value(), 'New topic cond should have value None.')

        print 'publishing..'

        publisher.publish(True)
        rospy.sleep(2)
        self.assertTrue(self.rtc.get_value(), 'Now topic cond should have value True.')

        publisher.publish(False)
        rospy.sleep(2)
        self.assertFalse(self.rtc.get_value(), 'Now topic cond should have value False.')


    def testRTA_msg_args(self):
        print self.rta_true
        print self.rta_false

        rospy.sleep(2) # let subscribers find publishers
        self.assertIsNone(self.rtc.get_value(), 'New topic cond should have value None.')

        print 'publishing via ROSTopicAction..'

        self.rta_true.run(None)
        rospy.sleep(2)
        self.assertTrue(self.rtc.get_value(), 'Now topic cond should have value True.')

        self.rta_false.run(None)
        rospy.sleep(2)
        self.assertFalse(self.rtc.get_value(), 'Now topic cond should have value False.')

    def testRTA_msg_cb(self):
        print self.rta_cb_true
        print self.rta_cb_false

        rospy.sleep(2) # let subscribers find publishers
        self.assertIsNone(self.rtc.get_value(), 'New topic cond should have value None.')

        print 'publishing via ROSTopicAction..'

        self.rta_cb_true.run(None)
        rospy.sleep(2)
        self.assertTrue(self.rtc.get_value(), 'Now topic cond should have value True.')

        self.rta_cb_false.run(None)
        rospy.sleep(2)
        self.assertFalse(self.rtc.get_value(), 'Now topic cond should have value False.')



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
