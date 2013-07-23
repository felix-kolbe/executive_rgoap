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

from goap.goap import Condition


class Test(unittest.TestCase):


    def setUp(self):
        self.condition1 = Condition()
        self.condition2 = Condition()
        Condition._conditions_dict = {}


    def tearDown(self):
        pass


    def testAdd(self):
        self.assertIs(Condition.add('name1', self.condition1), None, 'Could not add new condition')
        self.assertIs(Condition.add('name2', self.condition2), None, 'Could not add another new condition')

    def testAddSame(self):
        self.assertIs(Condition.add('name1', self.condition1), None, 'Could not add new condition')
#         self.assertRaises(AssertionError, Condition.add, 'name2', self.condition1) #'Could not add another new condition')

    def testGet(self):
        self.assertRaises(AssertionError, Condition.get, 'name_inexistent') # 'Does not fail on getting inexistent condition')

    def testGetSame(self):
        self.assertIs(Condition.add('name', self.condition1), None, 'Could not add new condition')
        self.assertIs(Condition.get('name'), self.condition1, 'Could not get that same condition')



if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testName']
    unittest.main()
