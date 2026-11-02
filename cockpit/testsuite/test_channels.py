#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 University of Oxford
##
## This file is part of Cockpit.
##
## Cockpit is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## Cockpit is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with Cockpit.  If not, see <http://www.gnu.org/licenses/>.

import os
import tempfile
import unittest

from cockpit.interfaces import channels


class TestChannelsApi(unittest.TestCase):
    def setUp(self):
        self.channels = channels.Channels()
        self.channels.add("first", {"camera": {"exposure": 10}})
        self.channels.add("second", {"laser": {"power": 5}})

    def test_names_keep_insertion_order(self):
        self.assertEqual(self.channels.names, ["first", "second"])

    def test_get_returns_channel(self):
        self.assertEqual(self.channels.get("second"), {"laser": {"power": 5}})

    def test_add_existing_name_raises(self):
        with self.assertRaises(ValueError):
            self.channels.add("first", {})

    def test_change_replaces_channel_keeps_order(self):
        self.channels.change("first", {"camera": {"exposure": 20}})
        self.assertEqual(self.channels.names, ["first", "second"])
        self.assertEqual(self.channels.get("first"), {"camera": {"exposure": 20}})

    def test_remove_drops_channel(self):
        self.channels.remove("first")
        self.assertEqual(self.channels.names, ["second"])

    def test_update_adds_and_changes(self):
        other = channels.Channels()
        other.add("second", {"laser": {"power": 7}})
        other.add("third", {"camera": {"exposure": 1}})

        self.channels.update(other)

        self.assertEqual(self.channels.names, ["first", "second", "third"])
        self.assertEqual(self.channels.get("second"), {"laser": {"power": 7}})
        self.assertEqual(self.channels.get("third"), {"camera": {"exposure": 1}})


class TestChannelsFileRoundTrip(unittest.TestCase):
    def test_save_and_load_round_trip(self):
        original = channels.Channels()
        original.add("first", {"camera": {"exposure": 10}})
        original.add("second", {"laser": {"power": 5}})

        fd, path = tempfile.mkstemp(suffix=".json")
        os.close(fd)
        self.addCleanup(os.remove, path)

        channels.save_to_file(path, original)
        loaded = channels.load_from_file(path)

        self.assertEqual(loaded.names, original.names)
        for name in original.names:
            self.assertEqual(loaded.get(name), original.get(name))


if __name__ == "__main__":
    unittest.main()
