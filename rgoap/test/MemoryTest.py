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

from rgoap.inheriting import Memory


class MemoryTest(unittest.TestCase):

    def setUp(self):
        self.mem = Memory()

    def test_fresh(self):
        self.assertRaises(KeyError, self.mem.get_value, 'undef_var')

    def test_unset_var(self):
        name = 'declared_only'
        self.mem.declare_state(name)
        self.assertIsNone(self.mem.get_value(name), 'Undeclared state should give None?')

    def test_consistency(self):
        name = "var_GHJKKL"
        value = "val_ASDFGHJ"
        self.mem.declare_state(name, value)
        self.assertEqual(self.mem.get_value(name), value, 'Memory state not consistent')

    def test_declare_none_after_value(self):
        name = "var_GHJKKL"
        value = "val_ASDFGHJ"
        self.mem.declare_state(name, value)
        self.mem.declare_state(name)
        self.assertEqual(self.mem.get_value(name), value, 'Memory state not consistent')


@unittest.skip('removed feature')
class MemoryTestSingletons(unittest.TestCase):

    def setUp(self):
        self.mem1 = Memory()
        self.mem2 = Memory()

    def test_Memory_singleton_inst(self):
        self.assertIs(self.mem1, self.mem2, 'Memory objects not same object')

    def test_Memory_singleton_mem_inst(self):
        self.assertIs(self.mem1._memory, self.mem2._memory, 'Memory objects dicts not same object')

    def test_Memory_singleton_values(self):
        name = "var_GHJKKL"
        value = "val_ASDFGHJ"
        self.mem1.declare_state(name, value)
        self.assertEqual(self.mem2.get_value(name), value, 'Nonempty Memory objects not equal')

    def test_Memory_singleton_itm_equ(self):
        self.mem1.declare_state('a', '1')
        self.mem1.declare_state('b', '2')
        self.mem2.declare_state('c', '3')
        self.mem2.declare_state('d', '4')
        self.assertDictEqual(self.mem1._memory, self.mem2._memory, 'Nonempty Memory objects not equal')


if __name__ == "__main__":
    #import sys;sys.argv = ['', 'Test.testMemory']
    unittest.main()
