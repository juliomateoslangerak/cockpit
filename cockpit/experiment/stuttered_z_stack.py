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

import wx

import cockpit.util.user_config
from cockpit import events
from cockpit.experiment import z_stack


## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "Stuttered Z-stack"


class StutteredZStackExperiment(z_stack.ZStackExperiment):
    """This class handles experiments where we alter the rate at which we take
    stacks as a function of time.

    E.g. (each | represents a single volume): ||||   |   |   |   ||||   |   |
    |   |||| ...
    """

    def __init__(self, sample_rates, *args, **kwargs):
        """Set up the stuttered Z-stack experiment.

        If we hit the end of the list before running out of reps, then we
        recommence from the beginning.

        Args:
            sample_rates (list): A list of (interval, num_reps) tuples
                indicating the amount of time in seconds between each rep, and
                the number of reps to perform at that rate.
        """
        super().__init__(*args, **kwargs)
        self.sample_rates = sample_rates
        self.should_abort = False

        events.subscribe(events.USER_ABORT, self.on_abort)

    def on_abort(self):
        """User aborted."""
        self.should_abort = True

    def execute(self):
        """Call Experiment.execute() repeatedly, while varying self.num_reps and
        self.rep_duration so that we do the right sequence with the right timings.
        """
        # HACK: replace self.num_reps since it's used in Experiment.execute(),
        # which we will be calling from within this function. We'll handle
        # reps more directly.
        true_reps = self.num_reps
        num_reps_performed = 0
        sequence_index = 0
        was_successful = True
        while num_reps_performed < true_reps:
            interval, num_reps = self.sample_rates[
                sequence_index % len(self.sample_rates)
            ]
            num_reps = min(num_reps, true_reps - num_reps_performed)
            # Set these values so that Experiment.execute() can use them
            # safely.
            self.num_reps = num_reps
            self.rep_duration = interval
            # Run a normal experiment.
            was_successful = super().execute()
            sequence_index += 1
            num_reps_performed += num_reps
            if self.should_abort:
                return False
        return was_successful


## A consistent name to use to refer to the class itself.
EXPERIMENT_CLASS = StutteredZStackExperiment
from cockpit.gui.gui_utils import FLOATVALIDATOR, INTVALIDATOR


class ExperimentUI(wx.Panel):
    def __init__(self, parent, config_key):
        super().__init__(parent=parent)

        self.config_key = config_key
        ## List of (interval in seconds, number of reps) tuples
        # representing the rate at which we should image the
        # sample.
        self.sample_rates = []

        sizer = wx.FlexGridSizer(2, 6, 2, 2)
        sizer.Add((0, 0))
        sizer.Add((0, 0))
        sizer.Add(wx.StaticText(self, -1, "Sampling sequence"))
        sizer.Add(wx.StaticText(self, -1, "Interval (s)"))
        sizer.Add(wx.StaticText(self, -1, "Reps"))
        sizer.Add((0, 0))
        # Commence second row.
        button = wx.Button(self, -1, "Clear")
        button.Bind(wx.EVT_BUTTON, self.on_clear)
        button.SetToolTip(wx.ToolTip("Remove all entries"))
        sizer.Add(button)

        button = wx.Button(self, -1, "Delete last")
        button.Bind(wx.EVT_BUTTON, self.on_delete_last)
        button.SetToolTip(wx.ToolTip("Remove the most recently-added entry"))
        sizer.Add(button)

        self.sequence_text = wx.TextCtrl(
            self, -1, size=(200, -1), style=wx.TE_READONLY
        )
        self.sequence_text.SetToolTip(
            wx.ToolTip(
                "Displays the sequence of "
                + "sampling intervals and reps we will perform for this "
                + "experiment."
            )
        )
        sizer.Add(self.sequence_text)

        self.interval = wx.TextCtrl(self, -1, size=(60, -1))
        self.interval.SetToolTip(
            wx.ToolTip(
                "Amount of time, in seconds, that "
                + "passes between each rep for this portion of the experiment."
            )
        )
        self.interval.SetValidator(FLOATVALIDATOR)
        self.interval.allow_empty = True
        sizer.Add(self.interval)

        self.num_reps = wx.TextCtrl(self, -1, size=(60, -1))
        self.num_reps.SetToolTip(
            wx.ToolTip(
                "Number of reps to perform at this " + "sampling interval."
            )
        )
        self.num_reps.SetValidator(INTVALIDATOR)
        self.num_reps.allow_empty = True
        sizer.Add(self.num_reps)

        button = wx.Button(self, -1, "Add")
        button.Bind(wx.EVT_BUTTON, self.on_add)
        button.SetToolTip(
            wx.ToolTip("Add this (interval, reps) pair to the sequence.")
        )
        sizer.Add(button)

        self.SetSizerAndFit(sizer)

    def on_clear(self, event=None):
        """User clicked the "Clear" button; wipe out our current settings."""
        self.sample_rates = []
        self.set_text()

    def on_delete_last(self, event=None):
        """User clicked the "Delete last" button; remove the most recent setting."""
        self.sample_rates = self.sample_rates[:-1]
        self.set_text()

    def on_add(self, event=None):
        """User clicked the "Add" button; add the new pair to our settings."""
        interval = float(self.interval.GetValue())
        num_reps = int(self.num_reps.GetValue())
        self.sample_rates.append((interval, num_reps))
        self.interval.SetValue("")
        self.num_reps.SetValue("")
        self.set_text()

    def set_text(self):
        """Update our text display of the settings."""
        text = ", ".join(
            ["(%.2f, %d)" % (i, n) for (i, n) in self.sample_rates]
        )
        self.sequence_text.SetValue(text)

    def augment_params(self, params, should_save=True):
        if should_save:
            self.save_settings()
        params["sample_rates"] = self.get_sample_rates()
        return params

    def load_settings(self):
        return cockpit.util.user_config.get_value(
            self.config_key + "StutteredZStackSettings", default=[]
        )

    def get_sample_rates(self):
        return self.sample_rates

    def get_settings_dict(self):
        return self.augment_params({}, should_save=False)

    def save_settings(self, settings=None):
        if settings is None:
            settings = self.get_settings_dict()
        cockpit.util.user_config.set_value(
            self.config_key + "StutteredZStackSettings", settings
        )
