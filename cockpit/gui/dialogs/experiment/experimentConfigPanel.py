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

    The parent is required to implement onExperimentPanelResize so that changes
    in the panel size will be handled properly.
    """

    def __init__(
        self,
        parent,
        resizeCallback,
        resetCallback,
        configKey="singleSiteExperiment",
    ):
        """Instantiate the class.

        Pull default values out of the config file, and create the UI and
        layout.

        Args:
            parent (wx.Window): Parent window for the panel.
            resizeCallback (callable): Function to call when we have changed
                size.
            resetCallback (callable): Function to call to force a reset of the
                panel.
            configKey (str): String used to look up settings in the user
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

        self.configKey = configKey
        self.resizeCallback = resizeCallback
        self.resetCallback = resetCallback

        ## cockpit.experiment.Experiment subclass instance -- mostly preserved for
        # debugging, so we can examine the state of the experiment.
        self.runner = None

        self.allLights = wx.GetApp().depot.get_handlers_of_type(
            depot.LIGHT_TOGGLE
        )
        self.allLights.sort(key=lambda l: l.wavelength)
        self.allCameras = wx.GetApp().depot.get_handlers_of_type(depot.CAMERA)
        self.allCameras.sort(key=lambda c: c.name)

        self.channels = wx.GetApp().channels

        ## Map of default settings as loaded from config.
        self.settings = self.loadConfig()

        self.SetSizer(wx.BoxSizer(wx.VERTICAL))
        self.sizer = self.GetSizer()

        # Section for settings that are universal to all experiment types.
        universalSizer = wx.FlexGridSizer(2, 3, 5, 5)

        ## Maps experiment description strings to experiment modules.
        self.experimentStringToModule = collections.OrderedDict()
        for (
            module
        ) in cockpit.experiment.experiment_registry.getExperimentModules():
            self.experimentStringToModule[module.EXPERIMENT_NAME] = module

        self.experimentType = wx.Choice(
            self, choices=list(self.experimentStringToModule.keys())
        )
        self.experimentType.SetSelection(0)
        gui_utils.add_labeled_input(
            self,
            universalSizer,
            label="Experiment type:",
            control=self.experimentType,
        )

        self.numReps = gui_utils.add_labeled_input(
            self,
            universalSizer,
            label="Number of reps:",
            default_value=self.settings["num_reps"],
        )
        self.numReps.SetValidator(gui_utils.INTVALIDATOR)

        self.repDuration = gui_utils.add_labeled_input(
            self,
            universalSizer,
            label="Rep duration (s):",
            default_value=self.settings["rep_duration"],
            helper_string="Amount of time that must pass between the start "
                          + "of each rep. Use 0 if you don't want any wait time.",
        )
        self.repDuration.SetValidator(gui_utils.FLOATVALIDATOR)

        self.zPositionMode = wx.Choice(self, choices=Z_POSITION_MODES)
        self.zPositionMode.SetSelection(0)
        gui_utils.add_labeled_input(
            self,
            universalSizer,
            label="Z position mode:",
            control=self.zPositionMode,
        )

        self.stackHeight = gui_utils.add_labeled_input(
            self,
            universalSizer,
            label="Stack height (\u03bcm):",
            default_value=self.settings["stackHeight"],
        )
        self.stackHeight.SetValidator(gui_utils.FLOATVALIDATOR)

        self.sliceHeight = gui_utils.add_labeled_input(
            self,
            universalSizer,
            label="Slice height (\u03bcm):",
            default_value=self.settings["slice_height"],
        )
        self.sliceHeight.SetValidator(gui_utils.FLOATVALIDATOR)

        self.sizer.Add(universalSizer, 0, wx.ALL, border=5)

        ## Maps experiment modules to ExperimentUI instances holding the
        # UI for that experiment, if any.
        self.experimentModuleToPanel = {}
        for module in self.experimentStringToModule.values():
            if not hasattr(module, "ExperimentUI"):
                # This experiment type has no special UI to set up.
                continue
            panel = module.ExperimentUI(self, self.configKey)
            panel.Hide()
            self.sizer.Add(panel)
            self.experimentModuleToPanel[module] = panel
        self.experimentType.Bind(wx.EVT_CHOICE, self.onExperimentTypeChoice)
        self.onExperimentTypeChoice()

        # Section for exposure settings. We allow either setting per-laser
        # exposure times and activating all cameras as a group, or setting
        # them per-laser and per-camera (and activating each camera-laser
        # grouping in sequence).
        exposureSizer = wx.BoxSizer(wx.VERTICAL)

        ## Controls which set of exposure settings we enable.
        self.exposureMode = wx.RadioBox(
            self,
            label="Exposure mode",
            choices=[
                "Expose simultaneously",
                "Expose sequentially",
                "Expose channels",
            ],
            majorDimension=1,
        )
        exposureSizer.Add(self.exposureMode, 0, wx.ALL, border=5)

        ## Panel for holding controls for when we expose every camera
        # simultaneously.
        self.simultaneousExposurePanel = wx.Panel(
            self, name="simultaneous exposures"
        )
        simultaneousSizer = wx.BoxSizer(wx.VERTICAL)
        simultaneousSizer.Add(
            wx.StaticText(
                self.simultaneousExposurePanel,
                -1,
                "Exposure times for light sources:",
            ),
            0,
            wx.ALL,
            5,
        )

        ## Ordered list of exposure times for simultaneous exposure mode.
        self.lightExposureTimes, timeSizer = gui_utils.make_lights_controls(
            self.simultaneousExposurePanel,
            [str(l.name) for l in self.allLights],
            self.settings["simultaneousExposureTimes"],
        )
        simultaneousSizer.Add(timeSizer)
        useCurrentButton = wx.Button(
            self.simultaneousExposurePanel, -1, "Use current settings"
        )
        useCurrentButton.SetToolTip(
            wx.ToolTip(
                "Use the same settings as are currently used to take images with the '+' button"
            )
        )
        useCurrentButton.Bind(wx.EVT_BUTTON, self.onUseCurrentExposureSettings)
        simultaneousSizer.Add(useCurrentButton)

        self.simultaneousExposurePanel.SetSizerAndFit(simultaneousSizer)
        exposureSizer.Add(self.simultaneousExposurePanel, 0, wx.ALL, border=5)

        ## Panel for when we expose each camera in sequence.
        self.sequencedExposurePanel = wx.Panel(
            self, name="sequenced exposures"
        )
        ## Maps a camera handler to an ordered list of exposure times.
        self.cameraToExposureTimes = {}
        sequenceSizer = wx.FlexGridSizer(
            len(self.settings["sequencedExposureSettings"]) + 1,
            len(self.settings["sequencedExposureSettings"][0]) + 1,
            1,
            1,
        )
        for label in [""] + [str(l.name) for l in self.allLights]:
            sequenceSizer.Add(
                wx.StaticText(self.sequencedExposurePanel, -1, label),
                0,
                wx.ALIGN_RIGHT | wx.ALL,
                5,
            )
        for i, camera in enumerate(self.allCameras):
            sequenceSizer.Add(
                wx.StaticText(
                    self.sequencedExposurePanel, -1, str(camera.name)
                ),
                0,
                wx.TOP | wx.ALIGN_RIGHT,
                8,
            )
            times = []
            for label, defaultVal in zip(
                [str(l.name) for l in self.allLights],
                self.settings["sequencedExposureSettings"][i],
            ):
                exposureTime = wx.TextCtrl(
                    self.sequencedExposurePanel,
                    size=(40, -1),
                    name="exposure: %s for %s" % (label, camera.name),
                )
                exposureTime.SetValue(defaultVal)
                # allowEmpty=True lets validator know this control may be empty.
                exposureTime.SetValidator(gui_utils.FLOATVALIDATOR)
                exposureTime.allowEmpty = True
                sequenceSizer.Add(exposureTime, 0, wx.ALL, border=5)
                times.append(exposureTime)
            self.cameraToExposureTimes[camera] = times
        self.sequencedExposurePanel.SetSizerAndFit(sequenceSizer)
        exposureSizer.Add(self.sequencedExposurePanel, 0, wx.ALL, border=5)

        # Panel for when we expose channels.
        self.channelExposurePanel = wx.Panel(self, name="channel sequence")
        channelSizer = wx.FlexGridSizer(
            rows=len(self.channels.names), cols=3, vgap=5, hgap=10
        )
        channelSizer.AddGrowableCol(1)

        # A dictionary containing the order in which each channel is acquired
        self.channelsChecked = {}
        self.channelsOrder = {}
        self.channelsOrderPossible = []

        for channel in self.channels.names:
            # Checkboxes to activate channels
            channelCheckBox = wx.CheckBox(self.channelExposurePanel, label="")
            self.channelsChecked[channel] = channelCheckBox
            channelSizer.Add(
                channelCheckBox, flag=wx.ALL | wx.EXPAND, border=2
            )
            channelCheckBox.Bind(wx.EVT_CHECKBOX, self.onChannelCheckBoxToggle)

            # Channel labels
            channelLabel = wx.StaticText(
                self.channelExposurePanel, label=channel
            )
            channelSizer.Add(
                channelLabel, flag=wx.ALL | wx.ALIGN_CENTER_VERTICAL, border=2
            )

            # Choice of acquisition order
            choice = wx.Choice(self.channelExposurePanel, choices=[])
            choice.SetSelection(-1)
            choice.Disable()
            self.channelsOrder[channel] = choice
            channelSizer.Add(choice, flag=wx.ALL | wx.EXPAND, border=2)
            choice.Bind(wx.EVT_CHOICE, self.onChannelOrderChoice)

        self.channelExposurePanel.SetSizerAndFit(channelSizer)
        exposureSizer.Add(self.channelExposurePanel, 0, wx.ALL, border=5)

        # Toggle which panel is displayed based on the radio box.
        self.exposureMode.Bind(wx.EVT_RADIOBOX, self.onExposureMode)
        self.exposureMode.SetSelection(self.settings["exposure_mode"])
        self.onExposureMode()

        self.sizer.Add(exposureSizer)

        self.filepath_panel = FilepathPanel(self)
        self.filepath_panel.SetTemplate(self.settings["filenameTemplate"])
        self.filepath_panel.UpdateFilename()
        self.Sizer.Add(self.filepath_panel, wx.SizerFlags(1).Expand().Border())

        # Save/load experiment settings buttons.
        saveLoadPanel = wx.Panel(self)
        rowSizer = wx.BoxSizer(wx.HORIZONTAL)
        saveButton = wx.Button(
            saveLoadPanel, -1, "Save experiment settings..."
        )
        saveButton.Bind(wx.EVT_BUTTON, self.onSaveExperiment)
        rowSizer.Add(saveButton, 0, wx.ALL, 5)
        loadButton = wx.Button(
            saveLoadPanel, -1, "Load experiment settings..."
        )
        loadButton.Bind(wx.EVT_BUTTON, self.onLoadExperiment)
        rowSizer.Add(loadButton, 0, wx.ALL, 5)
        saveLoadPanel.SetSizerAndFit(rowSizer)
        self.sizer.Add(saveLoadPanel, 0, wx.LEFT, 5)

        self.SetSizerAndFit(self.sizer)

    def onChannelCheckBoxToggle(self, event):
        possibles = []
        p = 1
        for channel, checkbox in self.channelsChecked.items():
            self.channelsOrder[channel].SetSelection(-1)
            if checkbox.IsChecked():
                self.channelsOrder[channel].Enable()
                possibles.append(p)
                p += 1
            else:
                self.channelsOrder[channel].Disable()
        for choice in self.channelsOrder.values():
            choice.SetItems([str(i) for i in possibles])
            choice.SetSelection(-1)
        self.channelsOrderPossible = possibles

    def onChannelOrderChoice(self, event):
        selectedChoice = event.GetEventObject()
        selectedValue = event.GetString()
        for choice in self.channelsOrder.values():
            if (
                choice is not selectedChoice
                and choice.GetStringSelection() == selectedValue
            ):
                choice.SetSelection(-1)

    def loadConfig(self):
        """Load values from config, and validate them -- since devices may get
        changed out from under us, rendering some config entries (e.g. dealing
        with light sources) invalid."""
        result = cockpit.util.user_config.get_value(
            self.configKey,
            default={
                "num_reps": "1",
                "rep_duration": "0",
                "sequencedExposureSettings": [
                    ["" for l in self.allLights] for c in self.allCameras
                ],
                "exposure_mode": 0,
                "simultaneousExposureTimes": ["" for l in self.allLights],
                "slice_height": ".15",
                "stackHeight": "4",
                "ZPositionMode": 0,
            },
        )
        # FIXME: cockpit.util.user_config.get_value(...default={...})
        #   does not work well with dicts because it will read the
        #   saved dict and not pick defaults for missing keys so we
        #   end up without a default value for those.  These are new
        #   keys but we should probably handle the whole defaults in
        #   some other manner.
        result = {
            "filenameTemplate": "{time}.dv",
            **result,
        }

        for key in ["simultaneousExposureTimes"]:
            if len(result[key]) != len(self.allLights):
                # Number of light sources has changed; invalidate the config.
                result[key] = ["" for light in self.allLights]
        key = "sequencedExposureSettings"
        if len(result[key]) != len(self.allCameras) or len(
            result[key][0]
        ) != len(self.allLights):
            # Number of lights and/or number of cameras has changed.
            result[key] = [
                ["" for l in self.allLights] for c in self.allCameras
            ]
        return result

    def onExperimentTypeChoice(self, event=None):
        """User selected a different experiment type; show/hide specific
        experiment parameters as appropriate; depending on experiment type,
        some controls may be enabled/disabled."""
        newType = self.experimentType.GetStringSelection()
        for expString, module in self.experimentStringToModule.items():
            if module in self.experimentModuleToPanel:
                # This experiment module has a special UI panel which needs
                # to be shown/hidden.
                panel = self.experimentModuleToPanel[module]
                panel.Show(expString == newType)
                panel.Enable(expString == newType)
        self.SetSizerAndFit(self.sizer)
        self.resizeCallback(self)

    def onExposureMode(self, event=None):
        """User changed the exposure mode."""
        val = self.exposureMode.GetSelection()
        # Show the relevant light panel. Disable the unused panel to
        # prevent validation of its controls.
        self.simultaneousExposurePanel.Show(val == 0)
        self.simultaneousExposurePanel.Enable(val == 0)
        self.sequencedExposurePanel.Show(val == 1)
        self.sequencedExposurePanel.Enable(val == 1)
        self.channelExposurePanel.Show(val == 2)
        self.channelExposurePanel.Enable(val == 2)
        self.SetSizerAndFit(self.sizer)
        self.resizeCallback(self)

    def onUseCurrentExposureSettings(self, event=None):
        """User clicked the "Use current settings" button; fill out the
        simultaneous-exposure settings text boxes with the current
        interactive-mode exposure settings."""
        for i, light in enumerate(self.allLights):
            # Only have an exposure time if the light is enabled.
            val = ""
            if light.get_is_enabled():
                val = str(light.get_exposure_time())
            self.lightExposureTimes[i].SetValue(val)

    def onSaveExperiment(self, event=None):
        """User clicked the "Save experiment settings..." button; save the
        parameters for later use as a JSON dict."""
        settings = self.getSettingsDict()
        # Augment the settings with information pertinent to our current
        # experiment.
        experimentType = self.experimentType.GetStringSelection()
        settings["experimentType"] = experimentType
        module = self.experimentStringToModule[experimentType]
        if module in self.experimentModuleToPanel:
            # Have specific parameters for this experiment type; store them
            # too.
            settings["experimentSpecificValues"] = (
                self.experimentModuleToPanel[module].get_settings_dict()
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

    def onLoadExperiment(self, event=None):
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
        handle = open(filepath, "r")
        settings = json.loads(" ".join(handle.readlines()))
        handle.close()
        experimentType = settings["experimentType"]
        experimentIndex = self.experimentType.FindString(experimentType)
        module = self.experimentStringToModule[experimentType]
        if module in self.experimentModuleToPanel:
            panel = self.experimentModuleToPanel[module]
            panel.save_settings(settings["experimentSpecificValues"])
            del settings["experimentSpecificValues"]
        cockpit.util.user_config.set_value(self.configKey, settings)
        # Reset the panel, destroying us and creating a new panel with
        # the proper values in all parameters, except for experiment type.
        panel = self.resetCallback()
        panel.experimentType.SetSelection(experimentIndex)
        panel.onExperimentTypeChoice()

    def runExperiment(self):
        """Run the experiment per the user's settings."""
        # Returns True to close dialog box, None or False otherwise.
        self.saveSettings()
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

        exposureSettings = []

        if self.exposureMode.GetStringSelection() == "Expose simultaneously":
            lightTimePairs = [
                (
                    light,
                    gui_utils.try_parse_num(
                        self.lightExposureTimes[i], decimal.Decimal
                    ),
                )
                for i, light in enumerate(self.allLights)
                if (
                    self.allLights[i].get_is_enabled()
                    and self.lightExposureTimes[i].GetValue()
                )
            ]
            if not lightTimePairs and not gui_utils.get_user_permission(
                "No enabled light has a define exposure time. Are you sure you want to continue?"
            ):
                return True

            exposureSettings = [(cameras, lightTimePairs)]

        elif self.exposureMode.GetStringSelection() == "Expose sequentially":
            # A separate exposure for each camera.
            for camera in cameras:
                cameraSettings = self.cameraToExposureTimes[camera]
                settings = []
                for i, light in enumerate(self.allLights):
                    if not light.get_is_enabled():
                        continue
                    timeControl = cameraSettings[i]
                    if timeControl.GetValue():
                        settings.append(
                            (
                                light,
                                gui_utils.try_parse_num(
                                    timeControl, decimal.Decimal
                                ),
                            )
                        )
                exposureSettings.append(([camera], settings))

        elif self.exposureMode.GetStringSelection() == "Expose channels":
            # Check that there are active channels and they all have an order
            if not any(c.IsChecked() for c in self.channelsChecked.values()):
                wx.MessageDialog(
                    self,
                    "No channels are enabled, so the experiment cannot be run.",
                    style=wx.ICON_EXCLAMATION | wx.STAY_ON_TOP | wx.OK,
                ).ShowModal()
                return True
            unsortedExposureSettings = {}
            for channel, checkbox, order in zip(
                self.channels.names,
                self.channelsChecked.values(),
                self.channelsOrder.values(),
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
                    channelSettings = self.channels.get(channel)
                    cameras = []
                    cameras.extend(
                        cam_handler
                        for cam_handler in self.allCameras
                        if channelSettings[cam_handler.name]
                    )
                    lightTimePairs = [
                        (
                            light_handler,
                            decimal.Decimal(
                                float(
                                    channelSettings[light_handler.name][
                                        "exposure_time"
                                    ]
                                )
                            ),
                        )
                        for light_handler in self.allLights
                        if channelSettings[light_handler.name]["is_enabled"]
                    ]
                    unsortedExposureSettings[order.GetSelection()] = (
                        cameras,
                        lightTimePairs,
                    )

            exposureSettings = [
                unsortedExposureSettings[i]
                for i in sorted(unsortedExposureSettings.keys())
            ]

        altitude = cockpit.interfaces.stage_mover.get_position_for_axis(2)
        # Default to "current is bottom"
        altBottom = altitude
        zHeight = gui_utils.try_parse_num(self.stackHeight, float)
        if self.zPositionMode.GetStringSelection() == "Current is center":
            altBottom = altitude - zHeight / 2
        elif self.zPositionMode.GetStringSelection() == "Use saved top/bottom":
            altBottom = cockpit.interfaces.stage_mover.mover.saved_bottom
            zHeight = cockpit.interfaces.stage_mover.mover.saved_top - altBottom

        sliceHeight = gui_utils.try_parse_num(self.sliceHeight, float)
        if zHeight == 0:
            # 2D mode.
            zHeight = 1e-6
            sliceHeight = 1e-6

        try:
            savePath = self.filepath_panel.GetPath()
        except Exception:
            cockpit.gui.ExceptionBox(
                "Failed to get filename for data.", parent=self
            )
            return True

        params = {
            "num_reps": gui_utils.try_parse_num(self.numReps),
            "rep_duration": gui_utils.try_parse_num(self.repDuration, float),
            "z_positioner": mover,
            "alt_bottom": altBottom,
            "z_height": zHeight,
            "slice_height": sliceHeight,
            "exposure_settings": exposureSettings,
            "save_path": savePath,
        }
        experimentType = self.experimentType.GetStringSelection()
        module = self.experimentStringToModule[experimentType]
        if module in self.experimentModuleToPanel:
            # Add on the special parameters needed by this experiment type.
            params = self.experimentModuleToPanel[module].augment_params(params)

        self.runner = module.EXPERIMENT_CLASS(**params)
        return self.runner.run()

    def getSettingsDict(self):
        """Generate a dict of our current settings."""
        sequencedExposureSettings = []
        for i, camera in enumerate(self.allCameras):
            sequencedExposureSettings.append(
                [c.GetValue() for c in self.cameraToExposureTimes[camera]]
            )
        simultaneousTimes = [c.GetValue() for c in self.lightExposureTimes]

        newSettings = {
            "filenameTemplate": self.filepath_panel.GetTemplate(),
            "num_reps": self.numReps.GetValue(),
            "rep_duration": self.repDuration.GetValue(),
            "sequencedExposureSettings": sequencedExposureSettings,
            "exposure_mode": self.exposureMode.GetSelection(),
            "simultaneousExposureTimes": simultaneousTimes,
            "slice_height": self.sliceHeight.GetValue(),
            "stackHeight": self.stackHeight.GetValue(),
            "ZPositionMode": self.zPositionMode.GetSelection(),
        }
        return newSettings

    def saveSettings(self):
        """Save the current experiment settings to config."""
        cockpit.util.user_config.set_value(
            self.configKey, self.getSettingsDict()
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

        self._fname_ctrl = wx.TextCtrl(self, value="")

        self._update_btn = wx.Button(self, label="Update")
        self._update_btn.SetToolTip("Update 'Filename' based on 'Template'.")
        self._update_btn.Bind(wx.EVT_BUTTON, self._OnUpdateFilename)

        self.UpdateFilename()

        grid = wx.BoxSizer(wx.HORIZONTAL)

        static_sizer = wx.SizerFlags(0).CentreVertical().Border()
        expand_sizer = wx.SizerFlags(1).CentreVertical().Border()

        def add_pair(label, ctrl):
            grid.Add(wx.StaticText(self, label=label), static_sizer)
            grid.Add(ctrl, expand_sizer)

        add_pair("Directory:", self._dir_ctrl)
        add_pair("Template:", self._template_ctrl)
        add_pair("Filename:", self._fname_ctrl)
        grid.Add(self._update_btn, static_sizer)

        self.SetSizer(grid)

    def UpdateFilename(self, mappings: typing.Mapping[str, str] = {}) -> None:
        all_mappings = {
            "date": time.strftime("%Y%m%d"),
            "time": time.strftime("%H%M%S"),
            **mappings,
        }

        template = self._template_ctrl.GetValue()
        basename = template.format_map(Default(**all_mappings))
        self._fname_ctrl.SetValue(basename)

    def _OnUpdateFilename(self, evt: wx.CommandEvent) -> None:
        del evt
        self.UpdateFilename()

    def GetPath(self) -> str:
        """Return full filepath to use."""
        dirname = self._dir_ctrl.GetPath()
        basename = self._fname_ctrl.GetValue()
        if not basename:
            raise Exception("Filename is empty")
        return os.path.join(dirname, basename)

    def GetTemplate(self) -> str:
        return self._template_ctrl.GetValue()

    def SetTemplate(self, template: str) -> None:
        self._template_ctrl.SetValue(template)


class Default(dict):
    def __missing__(self, key):
        return f"{{key}}"
