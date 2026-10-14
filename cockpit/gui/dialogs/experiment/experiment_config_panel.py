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


import collections
import decimal
import json
import logging
import os.path
import time
import traceback
import typing

import wx

import cockpit.experiment.experiment_registry
import cockpit.interfaces.stage_mover
import cockpit.util.user_config
from cockpit import depot
from cockpit.gui import gui_utils


_logger = logging.getLogger(__name__)


## @package dialogs.experimentConfigPanel
# This module holds the ExperimentConfigPanel class and associated constants.

## List of Z positioning modes.
Z_POSITION_MODES = [
    "Current is center",
    "Current is bottom",
    "Use saved top/bottom",
]


class ExperimentConfigPanel(wx.Panel):
    """This class provides a GUI for setting up and running experiments, in the
    form of an embeddable wx.Panel and a selection of functions.

    To use the panel, create an instance of ExperimentConfigPanel and insert it
    into your GUI. Call its runExperiment function when you are ready to start
    the experiment.

    The parent is required to implement on_experiment_panel_resize so that changes
    in the panel size will be handled properly.
    """

    def __init__(
        self,
        parent,
        resize_callback,
        reset_callback,
        config_key="single_site_experiment",
    ):
        """Instantiate the class.

        Pull default values out of the config file, and create the UI and
        layout.

        Args:
            parent (wx.Window): Parent window for the panel.
            resize_callback (callable): Function to call when we have changed
                size.
            reset_callback (callable): Function to call to force a reset of the
                panel.
            config_key (str): String used to look up settings in the user
                config. This allows different experiment panels to have
                different defaults.
        """
        super().__init__(
            parent,
            style=wx.DEFAULT_DIALOG_STYLE
            | wx.RESIZE_BORDER
            | wx.TAB_TRAVERSAL,
        )
        self.parent = parent

        self.config_key = config_key
        self.resize_callback = resize_callback
        self.reset_callback = reset_callback

        ## cockpit.experiment.Experiment subclass instance -- mostly preserved for
        # debugging, so we can examine the state of the experiment.
        self.runner = None

        self.all_lights = wx.GetApp().depot.get_handlers_of_type(
            depot.LIGHT_TOGGLE
        )
        self.all_lights.sort(key=lambda l: l.wavelength)
        self.all_cameras = wx.GetApp().depot.get_handlers_of_type(depot.CAMERA)
        self.all_cameras.sort(key=lambda c: c.name)

        self.channels = wx.GetApp().channels

        ## Map of default settings as loaded from config.
        self.settings = self.load_config()

        self.SetSizer(wx.BoxSizer(wx.VERTICAL))
        self.sizer = self.GetSizer()

        # Section for settings that are universal to all experiment types.
        universal_sizer = wx.FlexGridSizer(2, 3, 5, 5)

        ## Maps experiment description strings to experiment modules.
        self.experiment_string_to_module = collections.OrderedDict()
        for (
            module
        ) in cockpit.experiment.experiment_registry.get_experiment_modules():
            self.experiment_string_to_module[module.EXPERIMENT_NAME] = module

        self.experiment_type = wx.Choice(
            self, choices=list(self.experiment_string_to_module.keys())
        )
        self.experiment_type.SetSelection(0)
        gui_utils.add_labeled_input(
            self,
            universal_sizer,
            label="Experiment type:",
            control=self.experiment_type,
        )

        self.num_reps = gui_utils.add_labeled_input(
            self,
            universal_sizer,
            label="Number of reps:",
            default_value=self.settings["num_reps"],
        )
        self.num_reps.SetValidator(gui_utils.INTVALIDATOR)

        self.rep_duration = gui_utils.add_labeled_input(
            self,
            universal_sizer,
            label="Rep duration (s):",
            default_value=self.settings["rep_duration"],
            helper_string="Amount of time that must pass between the start "
            + "of each rep. Use 0 if you don't want any wait time.",
        )
        self.rep_duration.SetValidator(gui_utils.FLOATVALIDATOR)

        self.z_position_mode = wx.Choice(self, choices=Z_POSITION_MODES)
        self.z_position_mode.SetSelection(0)
        gui_utils.add_labeled_input(
            self,
            universal_sizer,
            label="Z position mode:",
            control=self.z_position_mode,
        )

        self.stack_height = gui_utils.add_labeled_input(
            self,
            universal_sizer,
            label="Stack height (\u03bcm):",
            default_value=self.settings["stack_height"],
        )
        self.stack_height.SetValidator(gui_utils.FLOATVALIDATOR)

        self.slice_height = gui_utils.add_labeled_input(
            self,
            universal_sizer,
            label="Slice height (\u03bcm):",
            default_value=self.settings["slice_height"],
        )
        self.slice_height.SetValidator(gui_utils.FLOATVALIDATOR)

        self.sizer.Add(universal_sizer, 0, wx.ALL, border=5)

        ## Maps experiment modules to ExperimentUI instances holding the
        # UI for that experiment, if any.
        self.experiment_module_to_panel = {}
        for module in self.experiment_string_to_module.values():
            if not hasattr(module, "ExperimentUI"):
                # This experiment type has no special UI to set up.
                continue
            panel = module.ExperimentUI(self, self.config_key)
            panel.Hide()
            self.sizer.Add(panel)
            self.experiment_module_to_panel[module] = panel
        self.experiment_type.Bind(
            wx.EVT_CHOICE, self.on_experiment_type_choice
        )
        self.on_experiment_type_choice()

        # Section for exposure settings. We allow either setting per-laser
        # exposure times and activating all cameras as a group, or setting
        # them per-laser and per-camera (and activating each camera-laser
        # grouping in sequence).
        exposure_sizer = wx.BoxSizer(wx.VERTICAL)

        ## Controls which set of exposure settings we enable.
        self.exposure_mode = wx.RadioBox(
            self,
            label="Exposure mode",
            choices=[
                "Expose simultaneously",
                "Expose sequentially",
                "Expose channels",
            ],
            majorDimension=1,
        )
        exposure_sizer.Add(self.exposure_mode, 0, wx.ALL, border=5)

        ## Panel for holding controls for when we expose every camera
        # simultaneously.
        self.simultaneous_exposure_panel = wx.Panel(
            self, name="simultaneous exposures"
        )
        simultaneous_sizer = wx.BoxSizer(wx.VERTICAL)
        simultaneous_sizer.Add(
            wx.StaticText(
                self.simultaneous_exposure_panel,
                -1,
                "Exposure times for light sources:",
            ),
            0,
            wx.ALL,
            5,
        )

        ## Ordered list of exposure times for simultaneous exposure mode.
        self.light_exposure_times, timeSizer = gui_utils.make_lights_controls(
            self.simultaneous_exposure_panel,
            [str(l.name) for l in self.all_lights],
            self.settings["simultaneous_exposure_times"],
        )
        simultaneous_sizer.Add(timeSizer)
        use_current_button = wx.Button(
            self.simultaneous_exposure_panel, -1, "Use current settings"
        )
        use_current_button.SetToolTip(
            wx.ToolTip(
                "Use the same settings as are currently used to take images with the '+' button"
            )
        )
        use_current_button.Bind(
            wx.EVT_BUTTON, self.on_use_current_exposure_settings
        )
        simultaneous_sizer.Add(use_current_button)

        self.simultaneous_exposure_panel.SetSizerAndFit(simultaneous_sizer)
        exposure_sizer.Add(
            self.simultaneous_exposure_panel, 0, wx.ALL, border=5
        )

        ## Panel for when we expose each camera in sequence.
        self.sequenced_exposure_panel = wx.Panel(
            self, name="sequenced exposures"
        )
        ## Maps a camera handler to an ordered list of exposure times.
        self.camera_to_exposure_times = {}
        sequence_sizer = wx.FlexGridSizer(
            len(self.settings["sequenced_exposure_settings"]) + 1,
            len(self.settings["sequenced_exposure_settings"][0]) + 1,
            1,
            1,
        )
        for label in [""] + [str(l.name) for l in self.all_lights]:
            sequence_sizer.Add(
                wx.StaticText(self.sequenced_exposure_panel, -1, label),
                0,
                wx.ALIGN_RIGHT | wx.ALL,
                5,
            )
        for i, camera in enumerate(self.all_cameras):
            sequence_sizer.Add(
                wx.StaticText(
                    self.sequenced_exposure_panel, -1, str(camera.name)
                ),
                0,
                wx.TOP | wx.ALIGN_RIGHT,
                8,
            )
            times = []
            for label, default_val in zip(
                [str(l.name) for l in self.all_lights],
                self.settings["sequenced_exposure_settings"][i],
            ):
                exposure_time = wx.TextCtrl(
                    self.sequenced_exposure_panel,
                    size=(40, -1),
                    name="exposure: %s for %s" % (label, camera.name),
                )
                exposure_time.SetValue(default_val)
                # allow_empty=True lets validator know this control may be empty.
                exposure_time.SetValidator(gui_utils.FLOATVALIDATOR)
                exposure_time.allow_empty = True
                sequence_sizer.Add(exposure_time, 0, wx.ALL, border=5)
                times.append(exposure_time)
            self.camera_to_exposure_times[camera] = times
        self.sequenced_exposure_panel.SetSizerAndFit(sequence_sizer)
        exposure_sizer.Add(self.sequenced_exposure_panel, 0, wx.ALL, border=5)

        # Panel for when we expose channels.
        self.channel_exposure_panel = wx.Panel(self, name="channel sequence")
        channel_sizer = wx.FlexGridSizer(
            rows=len(self.channels.names), cols=3, vgap=5, hgap=10
        )
        channel_sizer.AddGrowableCol(1)

        # A dictionary containing the order in which each channel is acquired
        self.channels_checked = {}
        self.channels_order = {}
        self.channels_order_possible = []

        for channel in self.channels.names:
            # Checkboxes to activate channels
            channel_check_box = wx.CheckBox(
                self.channel_exposure_panel, label=""
            )
            self.channels_checked[channel] = channel_check_box
            channel_sizer.Add(
                channel_check_box, flag=wx.ALL | wx.EXPAND, border=2
            )
            channel_check_box.Bind(
                wx.EVT_CHECKBOX, self.on_channel_check_box_toggle
            )

            # Channel labels
            channel_label = wx.StaticText(
                self.channel_exposure_panel, label=channel
            )
            channel_sizer.Add(
                channel_label, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL, border=2
            )

            # Choice of acquisition order
            choice = wx.Choice(self.channel_exposure_panel, choices=[])
            choice.SetSelection(-1)
            choice.Disable()
            self.channels_order[channel] = choice
            channel_sizer.Add(choice, flag=wx.ALL | wx.EXPAND, border=2)
            choice.Bind(wx.EVT_CHOICE, self.on_channel_order_choice)

        self.channel_exposure_panel.SetSizerAndFit(channel_sizer)
        exposure_sizer.Add(self.channel_exposure_panel, 0, wx.ALL, border=5)

        # Toggle which panel is displayed based on the radio box.
        self.exposure_mode.Bind(wx.EVT_RADIOBOX, self.on_exposure_mode)
        self.exposure_mode.SetSelection(self.settings["exposure_mode"])
        self.on_exposure_mode()

        self.sizer.Add(exposure_sizer)

        self.filepath_panel = FilepathPanel(self)
        self.filepath_panel.set_template(self.settings["filename_template"])
        self.filepath_panel.update_filename()
        self.Sizer.Add(self.filepath_panel, wx.SizerFlags(1).Expand().Border())

        # Save/load experiment settings buttons.
        save_load_panel = wx.Panel(self)
        row_sizer = wx.BoxSizer(wx.HORIZONTAL)
        save_button = wx.Button(
            save_load_panel, -1, "Save experiment settings..."
        )
        save_button.Bind(wx.EVT_BUTTON, self.on_save_experiment)
        row_sizer.Add(save_button, 0, wx.ALL, 5)
        load_button = wx.Button(
            save_load_panel, -1, "Load experiment settings..."
        )
        load_button.Bind(wx.EVT_BUTTON, self.on_load_experiment)
        row_sizer.Add(load_button, 0, wx.ALL, 5)
        save_load_panel.SetSizerAndFit(row_sizer)
        self.sizer.Add(save_load_panel, 0, wx.LEFT, 5)

        self.SetSizerAndFit(self.sizer)

    def on_channel_check_box_toggle(self, event):
        possibles = []
        p = 1
        for channel, checkbox in self.channels_checked.items():
            self.channels_order[channel].SetSelection(-1)
            if checkbox.IsChecked():
                self.channels_order[channel].Enable()
                possibles.append(p)
                p += 1
            else:
                self.channels_order[channel].Disable()
        for choice in self.channels_order.values():
            choice.SetItems([str(i) for i in possibles])
            choice.SetSelection(-1)
        self.channels_order_possible = possibles

    def on_channel_order_choice(self, event):
        selected_choice = event.GetEventObject()
        selected_value = event.GetString()
        for choice in self.channels_order.values():
            if (
                choice is not selected_choice
                and choice.GetStringSelection() == selected_value
            ):
                choice.SetSelection(-1)

    def load_config(self):
        """Load values from config, and validate them -- since devices may get
        changed out from under us, rendering some config entries (e.g. dealing
        with light sources) invalid."""
        result = cockpit.util.user_config.get_value(
            self.config_key,
            default={
                "num_reps": "1",
                "rep_duration": "0",
                "sequenced_exposure_settings": [
                    ["" for l in self.all_lights] for c in self.all_cameras
                ],
                "exposure_mode": 0,
                "simultaneous_exposure_times": ["" for l in self.all_lights],
                "slice_height": ".15",
                "stack_height": "4",
                "z_position_mode": 0,
            },
        )
        # FIXME: cockpit.util.user_config.get_value(...default={...})
        #   does not work well with dicts because it will read the
        #   saved dict and not pick defaults for missing keys so we
        #   end up without a default value for those.  These are new
        #   keys but we should probably handle the whole defaults in
        #   some other manner.
        result = {
            "filename_template": "{time}.dv",
            **result,
        }

        for key in ["simultaneous_exposure_times"]:
            if len(result[key]) != len(self.all_lights):
                # Number of light sources has changed; invalidate the config.
                result[key] = ["" for light in self.all_lights]
        key = "sequenced_exposure_settings"
        if len(result[key]) != len(self.all_cameras) or len(
            result[key][0]
        ) != len(self.all_lights):
            # Number of lights and/or number of cameras has changed.
            result[key] = [
                ["" for l in self.all_lights] for c in self.all_cameras
            ]
        return result

    def on_experiment_type_choice(self, event=None):
        """User selected a different experiment type; show/hide specific
        experiment parameters as appropriate; depending on experiment type,
        some controls may be enabled/disabled."""
        new_type = self.experiment_type.GetStringSelection()
        for exp_string, module in self.experiment_string_to_module.items():
            if module in self.experiment_module_to_panel:
                # This experiment module has a special UI panel which needs
                # to be shown/hidden.
                panel = self.experiment_module_to_panel[module]
                panel.Show(exp_string == new_type)
                panel.Enable(exp_string == new_type)
        self.SetSizerAndFit(self.sizer)
        self.resize_callback(self)

    def on_exposure_mode(self, event=None):
        """User changed the exposure mode."""
        val = self.exposure_mode.GetSelection()
        # Show the relevant light panel. Disable the unused panel to
        # prevent validation of its controls.
        self.simultaneous_exposure_panel.Show(val == 0)
        self.simultaneous_exposure_panel.Enable(val == 0)
        self.sequenced_exposure_panel.Show(val == 1)
        self.sequenced_exposure_panel.Enable(val == 1)
        self.channel_exposure_panel.Show(val == 2)
        self.channel_exposure_panel.Enable(val == 2)
        self.SetSizerAndFit(self.sizer)
        self.resize_callback(self)

    def on_use_current_exposure_settings(self, event=None):
        """User clicked the "Use current settings" button; fill out the
        simultaneous-exposure settings text boxes with the current
        interactive-mode exposure settings."""
        for i, light in enumerate(self.all_lights):
            # Only have an exposure time if the light is enabled.
            val = ""
            if light.get_is_enabled():
                val = str(light.get_exposure_time())
            self.light_exposure_times[i].SetValue(val)

    def on_save_experiment(self, event=None):
        """User clicked the "Save experiment settings..." button; save the
        parameters for later use as a JSON dict."""
        settings = self.get_settings_dict()
        # Augment the settings with information pertinent to our current
        # experiment.
        experiment_type = self.experiment_type.GetStringSelection()
        settings["experiment_type"] = experiment_type
        module = self.experiment_string_to_module[experiment_type]
        if module in self.experiment_module_to_panel:
            # Have specific parameters for this experiment type; store them
            # too.
            settings["experiment_specific_values"] = (
                self.experiment_module_to_panel[module].get_settings_dict()
            )

        # Get the filepath to save settings to.
        dialog = wx.FileDialog(
            self,
            style=wx.FD_SAVE,
            wildcard="*.txt",
            message="Please select where to save the experiment.",
            defaultDir=wx.GetApp().config.getpath("global", "data-dir"),
        )
        if dialog.ShowModal() != wx.ID_OK:
            # User cancelled.
            return
        filepath = dialog.GetPath()
        handle = open(filepath, "w")
        try:
            handle.write(json.dumps(settings))
        except Exception as e:
            _logger.error("Couldn't save experiment settings: %s" % e)
            _logger.error(traceback.format_exc())
            _logger.error("Settings are:\n%s" % str(settings))
        handle.close()

    def on_load_experiment(self, event=None):
        """User clicked the "Load experiment settings..." button; load the
        parameters from a file."""
        dialog = wx.FileDialog(
            self,
            style=wx.FD_OPEN,
            wildcard="*.txt",
            message="Please select the experiment file to load.",
            defaultDir=wx.GetApp().config.getpath("global", "data-dir"),
        )
        if dialog.ShowModal() != wx.ID_OK:
            # User cancelled.
            return
        filepath = dialog.GetPath()
        # TODO: Wrap in with statement
        handle = open(filepath, "r")
        settings = json.loads(" ".join(handle.readlines()))
        handle.close()
        experiment_type = settings["experiment_type"]
        experiment_index = self.experiment_type.FindString(experiment_type)
        module = self.experiment_string_to_module[experiment_type]
        if module in self.experiment_module_to_panel:
            panel = self.experiment_module_to_panel[module]
            panel.save_settings(settings["experiment_specific_values"])
            del settings["experiment_specific_values"]
        cockpit.util.user_config.set_value(self.config_key, settings)
        # Reset the panel, destroying us and creating a new panel with
        # the proper values in all parameters, except for experiment type.
        panel = self.reset_callback()
        panel.experiment_type.SetSelection(experiment_index)
        panel.on_experiment_type_choice()

    def run_experiment(self):
        """Run the experiment per the user's settings."""
        # Returns True to close dialog box, None or False otherwise.
        self.save_settings()
        # Find the Z mover with the smallest range of motion, assumed
        # to be our experiment mover.
        mover = wx.GetApp().depot.get_sorted_stage_movers()[2][-1]
        # Only use active cameras and enabled lights.
        # Must do list(filter) because we will iterate over the list
        # many times.
        cameras = list(
            filter(
                lambda c: c.get_is_enabled(),
                wx.GetApp().depot.get_handlers_of_type(depot.CAMERA),
            )
        )
        if not cameras:
            wx.MessageBox(
                "No cameras are enabled, so the experiment cannot be run.",
                style=(wx.ICON_EXCLAMATION | wx.STAY_ON_TOP | wx.OK),
                parent=self,
            )
            return True
        # TODO: check that the filename is valid

        exposure_settings = []

        if self.exposure_mode.GetStringSelection() == "Expose simultaneously":
            light_time_pairs = [
                (
                    light,
                    gui_utils.try_parse_num(
                        self.light_exposure_times[i], decimal.Decimal
                    ),
                )
                for i, light in enumerate(self.all_lights)
                if (
                    self.all_lights[i].get_is_enabled()
                    and self.light_exposure_times[i].GetValue()
                )
            ]
            if not light_time_pairs and not gui_utils.get_user_permission(
                "No enabled light has a define exposure time. Are you sure you want to continue?"
            ):
                return True

            exposure_settings = [(cameras, light_time_pairs)]

        elif self.exposure_mode.GetStringSelection() == "Expose sequentially":
            # A separate exposure for each camera.
            for camera in cameras:
                camera_settings = self.camera_to_exposure_times[camera]
                settings = []
                for i, light in enumerate(self.all_lights):
                    if not light.get_is_enabled():
                        continue
                    time_control = camera_settings[i]
                    if time_control.GetValue():
                        settings.append(
                            (
                                light,
                                gui_utils.try_parse_num(
                                    time_control, decimal.Decimal
                                ),
                            )
                        )
                exposure_settings.append(([camera], settings))

        elif self.exposure_mode.GetStringSelection() == "Expose channels":
            # Check that there are active channels and they all have an order
            if not any(c.IsChecked() for c in self.channels_checked.values()):
                wx.MessageDialog(
                    self,
                    "No channels are enabled, so the experiment cannot be run.",
                    style=wx.ICON_EXCLAMATION | wx.STAY_ON_TOP | wx.OK,
                ).ShowModal()
                return True
            unsorted_exposure_settings = {}
            for channel, checkbox, order in zip(
                self.channels.names,
                self.channels_checked.values(),
                self.channels_order.values(),
            ):
                if checkbox.IsChecked():
                    if order.GetSelection() == -1:
                        wx.MessageDialog(
                            self,
                            "Channel %s has no order, so the experiment cannot be run."
                            % channel,
                            style=wx.ICON_EXCLAMATION | wx.STAY_ON_TOP | wx.OK,
                        ).ShowModal()
                        return True
                    channel_settings = self.channels.get(channel)
                    cameras = []
                    cameras.extend(
                        cam_handler
                        for cam_handler in self.all_cameras
                        if channel_settings[cam_handler.name]
                    )
                    light_time_pairs = [
                        (
                            light_handler,
                            decimal.Decimal(
                                float(
                                    channel_settings[light_handler.name][
                                        "exposure_time"
                                    ]
                                )
                            ),
                        )
                        for light_handler in self.all_lights
                        if channel_settings[light_handler.name]["is_enabled"]
                    ]
                    unsorted_exposure_settings[order.GetSelection()] = (
                        cameras,
                        light_time_pairs,
                    )

            exposure_settings = [
                unsorted_exposure_settings[i]
                for i in sorted(unsorted_exposure_settings.keys())
            ]

        altitude = cockpit.interfaces.stage_mover.get_position_for_axis(2)
        # Default to "current is bottom"
        alt_bottom = altitude
        z_height = gui_utils.try_parse_num(self.stack_height, float)
        if self.z_position_mode.GetStringSelection() == "Current is center":
            alt_bottom = altitude - z_height / 2
        elif (
            self.z_position_mode.GetStringSelection() == "Use saved top/bottom"
        ):
            alt_bottom = cockpit.interfaces.stage_mover.mover.saved_bottom
            z_height = (
                cockpit.interfaces.stage_mover.mover.saved_top - alt_bottom
            )

        slice_height = gui_utils.try_parse_num(self.slice_height, float)
        if z_height == 0:
            # 2D mode.
            z_height = 1e-6
            slice_height = 1e-6

        try:
            save_path = self.filepath_panel.get_path()
        except Exception:
            cockpit.gui.ExceptionBox(
                "Failed to get filename for data.", parent=self
            )
            return True

        params = {
            "num_reps": gui_utils.try_parse_num(self.num_reps),
            "rep_duration": gui_utils.try_parse_num(self.rep_duration, float),
            "z_positioner": mover,
            "alt_bottom": alt_bottom,
            "z_height": z_height,
            "slice_height": slice_height,
            "exposure_settings": exposure_settings,
            "save_path": save_path,
        }
        experiment_type = self.experiment_type.GetStringSelection()
        module = self.experiment_string_to_module[experiment_type]
        if module in self.experiment_module_to_panel:
            # Add on the special parameters needed by this experiment type.
            params = self.experiment_module_to_panel[module].augment_params(
                params
            )

        self.runner = module.EXPERIMENT_CLASS(**params)
        return self.runner.run()

    def get_settings_dict(self):
        """Generate a dict of our current settings."""
        sequenced_exposure_settings = []
        for i, camera in enumerate(self.all_cameras):
            sequenced_exposure_settings.append(
                [c.GetValue() for c in self.camera_to_exposure_times[camera]]
            )
        simultaneous_times = [c.GetValue() for c in self.light_exposure_times]

        new_settings = {
            "filename_template": self.filepath_panel.get_template(),
            "num_reps": self.num_reps.GetValue(),
            "rep_duration": self.rep_duration.GetValue(),
            "sequenced_exposure_settings": sequenced_exposure_settings,
            "exposure_mode": self.exposure_mode.GetSelection(),
            "simultaneous_exposure_times": simultaneous_times,
            "slice_height": self.slice_height.GetValue(),
            "stack_height": self.stack_height.GetValue(),
            "z_position_mode": self.z_position_mode.GetSelection(),
        }
        return new_settings

    def save_settings(self):
        """Save the current experiment settings to config."""
        cockpit.util.user_config.set_value(
            self.config_key, self.get_settings_dict()
        )


class FilepathPanel(wx.Panel):
    """Panel to select directory and filename based on template."""

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._dir_ctrl = wx.DirPickerCtrl(
            self, path=wx.GetApp().config.getpath("global", "data-dir")
        )
        self._template_ctrl = wx.TextCtrl(self)
        self._template_ctrl.SetToolTip(
            "Template for the name of the data file. The values '{date}' and"
            " '{time}' will be replaced with their current values."
        )

        self._filename_ctrl = wx.TextCtrl(self, value="")

        self._update_btn = wx.Button(self, label="Update")
        self._update_btn.SetToolTip("Update 'Filename' based on 'Template'.")
        self._update_btn.Bind(wx.EVT_BUTTON, self._on_update_filename)

        self.update_filename()

        grid = wx.BoxSizer(wx.HORIZONTAL)

        static_sizer = wx.SizerFlags(0).CentreVertical().Border()
        expand_sizer = wx.SizerFlags(1).CentreVertical().Border()

        def add_pair(label, ctrl):
            grid.Add(wx.StaticText(self, label=label), static_sizer)
            grid.Add(ctrl, expand_sizer)

        add_pair("Directory:", self._dir_ctrl)
        add_pair("Template:", self._template_ctrl)
        add_pair("Filename:", self._filename_ctrl)
        grid.Add(self._update_btn, static_sizer)

        self.SetSizer(grid)

    def update_filename(self, mappings: typing.Mapping[str, str] = {}) -> None:
        all_mappings = {
            "date": time.strftime("%Y%m%d"),
            "time": time.strftime("%H%M%S"),
            **mappings,
        }

        template = self._template_ctrl.GetValue()
        basename = template.format_map(Default(**all_mappings))
        self._filename_ctrl.SetValue(basename)

    def _on_update_filename(self, evt: wx.CommandEvent) -> None:
        del evt
        self.update_filename()

    def get_path(self) -> str:
        """Return full filepath to use."""
        dirname = self._dir_ctrl.GetPath()
        basename = self._filename_ctrl.GetValue()
        if not basename:
            raise Exception("Filename is empty")
        return os.path.join(dirname, basename)

    def get_template(self) -> str:
        return self._template_ctrl.GetValue()

    def set_template(self, template: str) -> None:
        self._template_ctrl.SetValue(template)


class Default(dict):
    def __missing__(self, key):
        return f"{{key}}"
