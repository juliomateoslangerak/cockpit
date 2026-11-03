#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2021 Nicholas Hall
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

import wx

from cockpit.experiment import structured_illumination
from cockpit.gui import gui_utils


EXPERIMENT_NAME = "SIM Flux"

EXPERIMENT_CLASS = structured_illumination.SIExperiment


class ExperimentUI(structured_illumination.BaseSIMExperimentUI):
    _CONFIG_KEY_SUFFIX = "SIMFluxExperimentSettings"

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)

        row_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.num_phases = gui_utils.add_labeled_input(
            self,
            row_sizer,
            label="Number of phases",
            helper_string="How many phases do you want?",
        )
        self.num_phases.SetValue(str(self.settings["num_phases"]))
        self.Sizer.Insert(0, row_sizer)

        row_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.num_angles = gui_utils.add_labeled_input(
            self,
            row_sizer,
            label="Number of angles",
            helper_string="How many angles do you want?",
        )
        self.num_angles.SetValue(str(self.settings["num_angles"]))
        self.Sizer.Insert(1, row_sizer)

        self.Sizer.SetSizeHints(self)

    def augment_params(self, params):
        params = super().augment_params(params)
        params["num_angles"] = int(self.num_angles.GetValue())
        params["num_phases"] = int(self.num_phases.GetValue())
        return params

    def _get_default_settings(self):
        default = super()._get_default_settings()
        default.update(
            {
                "num_angles": 2,
                "num_phases": 50,
            }
        )
        return default

    def get_settings_dict(self):
        all_settings = super().get_settings_dict()
        all_settings.update(
            {
                "num_angles": self.num_angles.GetValue(),
                "num_phases": self.num_phases.GetValue(),
            }
        )
        return all_settings
