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

## Copyright 2013, The Regents of University of California
##
## Redistribution and use in source and binary forms, with or without
## modification, are permitted provided that the following conditions
## are met:
##
## 1. Redistributions of source code must retain the above copyright
##   notice, this list of conditions and the following disclaimer.
##
## 2. Redistributions in binary form must reproduce the above copyright
##   notice, this list of conditions and the following disclaimer in
##   the documentation and/or other materials provided with the
##   distribution.
##
## 3. Neither the name of the copyright holder nor the names of its
##   contributors may be used to endorse or promote products derived
##   from this software without specific prior written permission.
##
## THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
## "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
## LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
## FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
## COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
## INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
## BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
## LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
## CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
## LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
## ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
## POSSIBILITY OF SUCH DAMAGE.


import threading
import time

import wx

import cockpit.util.user_config
from cockpit import depot
from cockpit.experiment import z_stack
from cockpit.gui import gui_utils


## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "Example opto script"


class OptoExperiment(z_stack.ZStackExperiment):
    """This experiment is an example of a custom opto script.

    It is identical to a standard Z-stack experiment except that it turns on
    and off lights in a configurable timed sequence before running the stack
    Note that this class inherits from the ZStackExperiment class in
    experiment/z_stack.py; the generate_actions() function in that class is what
    collects the actual data. We just modify the execute() function (inherited
    from the base Experiment class in experiment/experiment.py) to engage the
    lights as specified before each rep. See below this class for the
    ExperimentUI and OptoPanel classes that set up the UI for configuring the
    experiment settings.
    """

    def __init__(
        self, light_to_sequence, light_to_is_on_during_acquisition, **kwargs
    ):
        """Set up the opto experiment with its light sequences.

        Args:
            light_to_sequence (dict): Maps light handler names to lists of
                [start, stop] times for illuminating them prior to running a
                standard Z-stack.
            light_to_is_on_during_acquisition (dict): Maps light handler names to
                whether or not those lights should be left on while taking
                images in the Z-stack section of the experiment.
        """
        # Convert from light source names to light handlers.
        self.light_to_sequence = {}
        for name, sequence in light_to_sequence.items():
            handler = depot.get_handler_with_name(name)
            self.light_to_sequence[handler] = sequence
        self.light_to_is_on_during_acquisition = {}
        for name, is_on in light_to_is_on_during_acquisition.items():
            handler = depot.get_handler_with_name(name)
            self.light_to_is_on_during_acquisition[handler] = is_on
        # Call the ZStackExperiment constructor with all of our remaining
        # parameters, which are in the "kwargs" dictionary object.
        super().__init__(**kwargs)

    def execute(self):
        """This function overrides the base execute() function in the
        Experiment class."""
        # HACK: replace self.num_reps since it's referred to in
        # Experiment.execute(), and we want to handle repeats at a higher
        # level.
        num_reps = self.num_reps
        self.num_reps = 1
        for i in range(num_reps):
            if self.should_abort:
                # User aborted experiment.
                break
            # Create a new thread for activating each light in parallel.
            # We could do this single-threaded, but it would require an
            # in-depth examination of self.light_to_sequence to determine when
            # to turn on each light.
            threads = []
            for light, sequence in self.light_to_sequence.items():
                new_thread = threading.Thread(
                    target=self.shine_light, args=[light, sequence]
                )
                new_thread.start()
                threads.append(new_thread)
            # Wait for all of our threads to finish.
            for thread in threads:
                thread.join()

            # Pre-acquisition illumination is complete; move on to taking
            # images.
            # Turn on all lights that want to be left on during acquisition.
            activated_lights = []
            for light, should_be_on in self.light_to_is_on_during_acquisition.items():
                if should_be_on:
                    light.set_exposing(True)
                    activated_lights.append(light)
            # Invoke our base class' execute() function to collect data.
            result = super().execute()
            # Turn off the triggering lights.
            for light in activated_lights:
                light.set_exposing(False)

        return result

    def shine_light(self, light, sequence):
        """Given a light source and a list of [start, stop] pairs when the
        light should be enabled/disabled, turn the light on and off according
        to the sequence."""
        cur_time = 0
        for start, stop in sequence:
            # Sleep until we should start illumination.
            time.sleep(start - cur_time)
            # Turn the light on.
            light.set_exposing(True)
            # Sleep until we should turn the light off.
            time.sleep(stop - start)
            # Turn the light off.
            light.set_exposing(False)
            cur_time = stop


## A consistent name to use to refer to the experiment class itself.
EXPERIMENT_CLASS = OptoExperiment


class ExperimentUI(wx.Panel):
    """Generate the UI for special parameters used by this experiment."""

    def __init__(self, parent, config_key):
        super().__init__(parent=parent)

        self.config_key = config_key

        ## List of all light sources available for use.
        self.all_lights = depot.get_handlers_of_type(depot.LIGHT_TOGGLE)

        ## User config settings.
        self.settings = self.load_settings()

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(
            wx.StaticText(self, -1, "Opto light exposure sequences:"),
            0,
            wx.ALL,
            5,
        )

        # Generate a header to label the controls in each OptoPanel. Sizes
        # for each element are derived from the OptoPanel's layout.
        header_sizer = wx.BoxSizer(wx.HORIZONTAL)
        for text, size in [
            ("Light", 60),
            ("", 80),
            ("", 80),
            ("Sequence", 200),
            ("Start", 40),
            ("End", 40),
            ("", 80),
            ("On during acquisition", 200),
        ]:
            header_sizer.Add(wx.StaticText(self, -1, text, size=(size, -1)))

        sizer.Add(header_sizer)

        ## Maps light handlers to OptoPanel instances.
        self.light_to_panel = {}
        default_vals = []
        for light in self.all_lights:
            sequence = self.settings["light_to_sequence"].get(light.name, [])
            does_start_on = self.settings["light_to_is_on_during_acquisition"].get(
                light.name, False
            )
            panel = OptoPanel(self, light, sequence, does_start_on)
            self.light_to_panel[light] = panel
            sizer.Add(panel)

        self.SetSizerAndFit(sizer)

    def get_light_to_sequence(self):
        """Return a mapping of light source names to lists of illumination
        sequences. e.g. {"650 LED": [[0, 20], [30, 50]]} to indicate that the
        650 LED should be on from 0 to 20 seconds, and from 30 to 50 seconds.

        We use handler names instead of handlers here so that they can be
        serialized (when saving experiment settings) cleanly.
        """
        return dict(
            [
                (l.name, self.light_to_panel[l].get_sequence())
                for l in self.all_lights
            ]
        )

    def get_light_to_is_on_during_acquisition(self):
        """Returns a mapping of light source names to whether or not those
        lights should be left on during image acquisition (independently of
        whether those lights are being used as traditional imaging lights).

        We use handler names instead of handlers here so that they can be
        serialized (when saving experiment settings) cleanly.
        """
        return dict(
            [
                (l.name, self.light_to_panel[l].get_is_on_during_acquisition())
                for l in self.all_lights
            ]
        )

    def augment_params(self, params, shouldSave=True):
        """Given a parameters dict (parameter name to value) to hand to the
        experiment instance,
        augment them with our special parameters."""
        if shouldSave:
            self.save_settings()
        params["light_to_sequence"] = self.get_light_to_sequence()
        params["light_to_is_on_during_acquisition"] = (
            self.get_light_to_is_on_during_acquisition()
        )
        return params

    def load_settings(self):
        """Load the saved experiment settings, if any."""
        return cockpit.util.user_config.get_value(
            self.config_key + "optoExampleSettings",
            default={
                "light_to_sequence": {},
                "light_to_is_on_during_acquisition": {},
            },
        )

    def get_settings_dict(self):
        """Generate a dict of our settings."""
        return self.augment_params({}, shouldSave=False)

    def save_settings(self, settings=None):
        """Save the current experiment settings to config."""
        if settings is None:
            settings = self.get_settings_dict()
        cockpit.util.user_config.set_value(
            self.config_key + "optoExampleSettings", settings
        )


## This panel lets the user define a sequence of illumination sequences (on and
# off) for a specific light.

from cockpit.gui.gui_utils import FLOATVALIDATOR


class OptoPanel(wx.Panel):
    def __init__(self, parent, light, sequence, doesStartOn):
        super().__init__(parent)
        self.light = light

        ## List of [start, stop] lists, indicating times when the light
        # should be turned on or off.
        self.sequence = sequence

        sizer = wx.BoxSizer(wx.HORIZONTAL)
        sizer.Add(wx.StaticText(self, -1, light.name + ": ", size=(60, -1)))

        button = wx.Button(self, -1, "Clear", size=(80, -1))
        button.Bind(wx.EVT_BUTTON, self.on_clear)
        button.SetToolTip(
            wx.ToolTip("Remove all entries for this light source")
        )
        sizer.Add(button)

        button = wx.Button(self, -1, "Delete last", size=(80, -1))
        button.Bind(wx.EVT_BUTTON, self.on_delete_last)
        button.SetToolTip(
            wx.ToolTip("Remove the last entry for this light source")
        )
        sizer.Add(button)

        self.sequence_text = wx.TextCtrl(
            self, -1, size=(200, -1), style=wx.TE_READONLY
        )
        self.sequence_text.SetToolTip(
            wx.ToolTip(
                "Displays the sequence of on/off actions this light source will perform prior to starting the acquisition."
            )
        )
        sizer.Add(self.sequence_text)

        self.start_text = wx.TextCtrl(self, -1, size=(40, -1))
        self.start_text.SetToolTip(
            wx.ToolTip("Time, in seconds, at which to start illumination")
        )
        sizer.Add(self.start_text)
        self.start_text.SetValidator(FLOATVALIDATOR)
        self.start_text.allow_empty = True

        self.stop_text = wx.TextCtrl(self, -1, size=(40, -1))
        self.start_text.SetToolTip(
            wx.ToolTip("Time, in seconds, at which to stop illumination")
        )
        sizer.Add(self.stop_text)
        self.stop_text.SetValidator(FLOATVALIDATOR)
        self.stop_text.allow_empty = True

        button = wx.Button(self, -1, "Add", size=(80, -1))
        button.Bind(wx.EVT_BUTTON, self.on_add)
        button.SetToolTip(
            wx.ToolTip("Add this start/stop pair to the sequence of actions.")
        )
        sizer.Add(button)

        self.is_on_during_acquisition = wx.CheckBox(self)
        self.is_on_during_acquisition.SetToolTip(
            wx.ToolTip(
                "If checked, this light source will continuously shine on the sample during acquisition."
            )
        )
        self.is_on_during_acquisition.SetValue(doesStartOn)
        sizer.Add(self.is_on_during_acquisition)

        self.SetSizerAndFit(sizer)
        self.update_display()

    def on_clear(self, event=None):
        """Clear all illumination settings."""
        self.sequence = []
        self.update_display()

    def on_delete_last(self, event=None):
        """Delete the last entry in the illumination settings."""
        self.sequence = self.sequence[:-1]
        self.update_display()

    def on_add(self, event=None):
        """Add a new entry to the illumination settings, merging entries as
        necessary."""
        start = gui_utils.try_parse_num(self.start_text, float)
        stop = gui_utils.try_parse_num(self.stop_text, float)
        if start >= stop:
            # No-op; easiest way for this to happen is for the stop text to
            # have no value.
            return
        # Go through our list of sequences and see if we need to merge them
        # together.
        self.sequence.append([start, stop])
        index_to_delete = len(self.sequence) - 1
        did_change = True
        while did_change:
            did_change = False
            for i, (alt_start, alt_stop) in enumerate(self.sequence):
                # Check for overlap.
                if (start < alt_start and stop > alt_start) or (
                    stop > alt_stop and start < alt_stop
                ):
                    # Merge the two entries.
                    start = min(start, alt_start)
                    stop = max(stop, alt_stop)
                    self.sequence[i] = [start, stop]
                    did_change = True
                    # Destroy the old sequence as it's been merged with
                    # this one.
                    del self.sequence[index_to_delete]
                    index_to_delete = i
                    break

        # Sort the sequence by start time.
        self.sequence.sort(key=lambda s: s[0])
        self.start_text.SetValue("")
        self.stop_text.SetValue("")
        self.update_display()

    def update_display(self):
        """Update our textual display of the illumination settings."""
        text = ", ".join(
            ["(%.2f, %.2f)" % (start, stop) for start, stop in self.sequence]
        )
        self.sequence_text.SetValue(text)

    def get_sequence(self):
        """Simple accessor."""
        return self.sequence

    def get_is_on_during_acquisition(self):
        """Simple accessor."""
        return self.is_on_during_acquisition.GetValue()
