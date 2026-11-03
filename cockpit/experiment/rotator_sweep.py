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

import decimal

import wx

import cockpit.util
from cockpit import depot
from cockpit.experiment import action_table, experiment
from cockpit.gui import gui_utils


## Provided so the UI knows what to call this experiment.
EXPERIMENT_NAME = "RotatorSweep"


class RotatorSweepExperiment(experiment.Experiment):
    """This class handles classic Z-stack experiments."""

    def __init__(
        self,
        polarizer_handler=None,
        settling_time=0.1,
        start_v=0.0,
        max_v=10.0,
        v_steps=100,
        *args,
        **kwargs,
    ):
        super().__init__(*args, **kwargs)
        self.polarizer_handler = polarizer_handler
        self.settling_time = settling_time
        # Look up the rotator analogue line handler.
        self.line_handler = polarizer_handler
        self.v_range = (start_v, max_v, v_steps)
        v_delta = float(max_v - start_v) / v_steps
        # Add voltage parameters to the metadata.
        self.metadata = "Rotator start and delta: [%f, %f]" % (
            start_v,
            v_delta,
        )

    def generate_actions(self):
        """Create the ActionTable needed to run the experiment."""
        table = action_table.ActionTable()
        cur_time = 0
        v_start, v_less_than, v_steps = self.v_range
        dv = float(v_less_than - v_start) / float(v_steps)
        dt = decimal.Decimal(self.settling_time)

        for step in range(v_steps):
            # Move to next polarization rotator voltage.
            v_target = v_start + step * dv
            table.add_action(cur_time, self.line_handler, v_target)
            cur_time += dt
            # Image the sample.
            for cameras, light_time_pairs in self.exposure_settings:
                cur_time = self.expose(
                    cur_time, cameras, light_time_pairs, table
                )
                # Advance the time very slightly so that all exposures
                # are strictly ordered.
                cur_time += decimal.Decimal(".001")
            # Hold the rotator angle constant during the exposure.
            table.add_action(cur_time, self.line_handler, v_target)
            # Advance time slightly so all actions are sorted (e.g. we
            # don't try to change angle and phase in the same timestep).
            cur_time += dt

        return table


## A consistent name to use to refer to the class itself.
EXPERIMENT_CLASS = RotatorSweepExperiment
from cockpit.gui.gui_utils import FLOATVALIDATOR, INTVALIDATOR


class ExperimentUI(wx.Panel):
    """Generate the UI for special parameters used by this experiment."""

    def __init__(self, parent, configKey):
        super().__init__(parent=parent)
        self.configkey = configKey
        sizer = wx.GridSizer(2, 4, 1)
        ## Maps strings to TextCtrls describing how to configure
        # response curve experiments.
        self.settings = self.load_settings()
        self.settling_time_control = gui_utils.add_labeled_input(
            self,
            sizer,
            label="settling time",
            default_value=self.settings["settling_time"],
        )
        self.settling_time_control.SetValidator(FLOATVALIDATOR)

        self.v_steps_control = gui_utils.add_labeled_input(
            self,
            sizer,
            label="V steps",
            default_value=self.settings["v_steps"],
        )
        self.v_steps_control.SetValidator(INTVALIDATOR)

        self.start_v_control = gui_utils.add_labeled_input(
            self,
            sizer,
            label="V start",
            default_value=self.settings["start_v"],
        )
        self.start_v_control.SetValidator(FLOATVALIDATOR)

        self.max_v_control = gui_utils.add_labeled_input(
            self,
            sizer,
            label="V max",
            default_value=self.settings["max_v"],
        )
        self.max_v_control.SetValidator(FLOATVALIDATOR)

        self.SetSizerAndFit(sizer)

    def augment_params(self, params):
        """Given a parameters dict (parameter name to value) to hand to the
        experiment instance, augment them with our special parameters."""
        self.save_settings()
        params["settling_time"] = gui_utils.try_parse_num(
            self.settling_time_control, float
        )
        params["start_v"] = gui_utils.try_parse_num(
            self.start_v_control, float
        )
        params["max_v"] = gui_utils.try_parse_num(self.max_v_control, float)
        params["v_steps"] = gui_utils.try_parse_num(self.v_steps_control)
        params["polarizer_handler"] = depot.get_handler_with_name(
            "SI polarizer"
        )
        return params

    def load_settings(self):
        """Load the saved experiment settings, if any."""
        return cockpit.util.user_config.get_value(
            self.configkey + "RotatorSweepExperimentSettings",
            default={
                "settling_time": "0.1",
                "start_v": "0.0",
                "max_v": "10.0",
                "v_steps": "100",
            },
        )

    def get_settings_dict(self):
        """Generate a dict of our settings."""
        return {
            "settling_time": self.settling_time_control.GetValue(),
            "start_v": self.start_v_control.GetValue(),
            "max_v": self.max_v_control.GetValue(),
            "v_steps": self.v_steps_control.GetValue(),
        }

    def save_settings(self, settings=None):
        """Save the current experiment settings to config."""
        if settings is None:
            settings = self.get_settings_dict()
        cockpit.util.user_config.set_value(
            self.configkey + "RotatorSweepExperimentSettings", settings
        )
