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

import wx

import cockpit.gui.dialogs.enumerateSitesPanel
import cockpit.util.userConfig
from cockpit import depot
from cockpit.experiment import experimentSpecs
from cockpit.gui import guiUtils
from cockpit.gui.dialogs.experiment import experimentConfigPanel


## Minimum size of controls (counting their labels)
CONTROL_SIZE = (280, -1)
## Minimum size of text input fields.
FIELD_SIZE = (70, -1)

_FILENAME_TEMPLATE = "{date}-{time}_t{cycle}_p{site}.mrc"


## Ask the user for confirmation from any thread. The experiment runs in a
# background thread, but dialogs must be shown from the main thread.
def _confirmInMainThread(message):
    if threading.current_thread() is threading.main_thread():
        return guiUtils.getUserPermission(message)
    answer = []
    done = threading.Event()

    def ask():
        try:
            answer.append(guiUtils.getUserPermission(message))
        finally:
            done.set()

    wx.CallAfter(ask)
    done.wait()
    return bool(answer and answer[0])


## This class allows for configuring multi-site experiments.
class MultiSiteExperimentDialog(wx.Dialog):
    def __init__(self, parent):
        super().__init__(
            parent,
            title="Microscope multi-site experiment",
            style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER,
        )

        ## The last MultiSiteRunner started, for debugging.
        self.runner = None

        ## List of all light handlers.
        self.allLights = wx.GetApp().Depot.getHandlersOfType(
            depot.LIGHT_TOGGLE
        )

        ## User's last-used inputs.
        self.settings = cockpit.util.userConfig.getValue(
            "multiSiteExperiment",
            default={
                "numCycles": "10",
                "cycleDuration": "60",
                "delayBeforeStarting": "0",
                "delayBeforeImaging": "0",
                "shouldCustomizeLightFrequencies": False,
                "shouldOptimizeSiteOrder": True,
                "lightFrequencies": ["1" for l in self.allLights],
            },
        )

        ## Contains self.panel
        self.sizer = wx.BoxSizer(wx.VERTICAL)

        ## Contains all UI widgets.
        self.panel = wx.Panel(self)
        ## Sizer for self.panel.
        self.panelSizer = wx.BoxSizer(wx.VERTICAL)
        ## Sizer for all controls except the start/cancel/reset buttons.
        controlsSizer = wx.BoxSizer(wx.HORIZONTAL)
        ## Sizer for a single column of controls.
        columnSizer = wx.BoxSizer(wx.VERTICAL)
        ## Panel for selecting sites to visit.
        self.sitesPanel = (
            cockpit.gui.dialogs.enumerateSitesPanel.EnumerateSitesPanel(
                self.panel,
                label="Sites to visit:",
                size=(200, -1),
                minSize=CONTROL_SIZE,
            )
        )
        columnSizer.Add(self.sitesPanel)

        self.numCycles = guiUtils.addLabeledInput(
            self.panel,
            columnSizer,
            label="Number of cycles:",
            defaultValue=self.settings["numCycles"],
            size=FIELD_SIZE,
            minSize=CONTROL_SIZE,
        )

        self.cycleDuration = guiUtils.addLabeledInput(
            self.panel,
            columnSizer,
            label="Min cycle duration (s):",
            defaultValue=self.settings["cycleDuration"],
            size=FIELD_SIZE,
            minSize=CONTROL_SIZE,
            helperString="Minimum amount of time to pass between each cycle. If the "
            + "cycle finishes early, then I will wait until this much "
            + "time has passed. You can enter multiple values here "
            + "separated by commas; I will then use each wait time in "
            + 'sequence; e.g. "60,120,180" means the first '
            + "cycle takes one minute, the second two, the third three, "
            + "the fourth one, the fifth two, and so on.",
        )

        self.delayBeforeStarting = guiUtils.addLabeledInput(
            self.panel,
            columnSizer,
            label="Delay before starting (min):",
            defaultValue=self.settings["delayBeforeStarting"],
            size=FIELD_SIZE,
            minSize=CONTROL_SIZE,
            helperString="Amount of time to wait before starting the experiment. "
            + "This is useful if you have a lengthy period to wait for "
            + "your cells to reach the stage you're interested in, for "
            + "example.",
        )

        self.delayBeforeImaging = guiUtils.addLabeledInput(
            self.panel,
            columnSizer,
            label="Delay before imaging (s):",
            defaultValue=self.settings["delayBeforeImaging"],
            size=FIELD_SIZE,
            minSize=CONTROL_SIZE,
            helperString="Amount of time to wait after moving to a site before "
            + "I start imaging the site. This is mostly useful if "
            + "your stage needs time to stabilize after moving.",
        )

        controlsSizer.Add(columnSizer, 0, wx.ALL, 5)

        columnSizer = wx.BoxSizer(wx.VERTICAL)
        ## We don't necessarily have this option.
        self.shouldPowerDownWhenDone = None
        powerHandlers = wx.GetApp().Depot.getHandlersOfType(
            depot.POWER_CONTROL
        )
        if powerHandlers:
            # There are devices that we could potentially turn off at end of
            # experiment.
            self.shouldPowerDownWhenDone = guiUtils.addLabeledInput(
                self.panel,
                columnSizer,
                label="Power off devices when done:",
                control=wx.CheckBox(self.panel),
                labelHeightAdjustment=0,
                border=3,
                flags=wx.ALL,
                helperString="If checked, then at the end of the experiment, I will "
                + "power down all the devices I can.",
            )

        self.shouldOptimizeSiteOrder = guiUtils.addLabeledInput(
            self.panel,
            columnSizer,
            label="Optimize route:",
            defaultValue=self.settings["shouldOptimizeSiteOrder"],
            control=wx.CheckBox(self.panel),
            labelHeightAdjustment=0,
            border=3,
            flags=wx.ALL,
            helperString="If checked, then I will calculate an ordering of the sites "
            + "that will minimize the total time spent in transit; "
            + "otherwise, I will use the order you specify.",
        )

        self.shouldCustomizeLightFrequencies = guiUtils.addLabeledInput(
            self.panel,
            columnSizer,
            label="Customize light frequencies:",
            defaultValue=self.settings["shouldCustomizeLightFrequencies"],
            control=wx.CheckBox(self.panel),
            labelHeightAdjustment=0,
            border=3,
            flags=wx.ALL,
            helperString="This allows you to set up experiments where different "
            + "light sources are enabled for different cycles. If you "
            + "set a frequency of 5 for a given light, for example, "
            + "then that light will only be used for every 5th pass "
            + "(the 1st, 6th, 11th, etc. cycles). You can specify an "
            + 'offset, too: "5 + 1" would enable the light for the '
            + "2nd, 7th, 12th, etc. cycles.",
        )
        self.shouldCustomizeLightFrequencies.Bind(
            wx.EVT_CHECKBOX, self.onCustomizeLightFrequencies
        )
        self.lightFrequenciesPanel = wx.Panel(
            self.panel, style=wx.BORDER_SUNKEN | wx.TAB_TRAVERSAL
        )
        self.lightFrequencies, sizer = guiUtils.makeLightsControls(
            self.lightFrequenciesPanel,
            [str(l.wavelength) for l in self.allLights],
            self.settings["lightFrequencies"],
        )
        self.lightFrequenciesPanel.SetSizerAndFit(sizer)
        self.lightFrequenciesPanel.Show(
            self.settings["shouldCustomizeLightFrequencies"]
        )
        columnSizer.Add(
            self.lightFrequenciesPanel, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5
        )

        controlsSizer.Add(columnSizer, 0, wx.ALL, 5)
        self.panelSizer.Add(controlsSizer)

        ## Controls whether or not the scanning experiment's parameters are
        # shown.
        self.showScanButton = wx.Button(
            self.panel, -1, "Show experiment settings"
        )
        self.showScanButton.Bind(wx.EVT_BUTTON, self.onShowScanButton)
        self.panelSizer.Add(
            self.showScanButton, 0, wx.ALIGN_CENTER | wx.TOP, 5
        )
        ## This panel configures the experiment we perform when visiting sites.
        self.experimentPanel = experimentConfigPanel.ExperimentConfigPanel(
            self.panel,
            resizeCallback=self.onExperimentPanelResize,
            resetCallback=self.onExperimentPanelReset,
            configKey="multiSiteExperimentPanel",
        )
        self.experimentPanel.filepath_panel.SetTemplate(_FILENAME_TEMPLATE)
        self.panelSizer.Add(
            self.experimentPanel, 0, wx.ALIGN_CENTER | wx.ALL, 5
        )
        self.experimentPanel.Hide()

        buttonSizer = wx.BoxSizer(wx.HORIZONTAL)

        button = wx.Button(self.panel, -1, "Reset")
        button.SetToolTip(
            wx.ToolTip("Reload this window with all default values")
        )
        button.Bind(wx.EVT_BUTTON, self.onReset)
        buttonSizer.Add(button, 0, wx.ALL, 5)

        buttonSizer.Add((1, 0), 1, wx.EXPAND)

        button = wx.Button(self.panel, wx.ID_CANCEL, "Cancel")
        buttonSizer.Add(button, 0, wx.ALL, 5)

        button = wx.Button(self.panel, wx.ID_OK, "Start")
        button.SetToolTip(wx.ToolTip("Start the experiment"))
        button.Bind(wx.EVT_BUTTON, self.onStart)
        buttonSizer.Add(button, 0, wx.ALL, 5)

        self.panelSizer.Add(buttonSizer, 0, wx.ALL, 5)
        self.panel.SetSizerAndFit(self.panelSizer)
        self.sizer.Add(self.panel)
        self.SetSizerAndFit(self.sizer)

    ## User clicked the show/hide scanning experiment button.
    def onShowScanButton(self, event):
        self.experimentPanel.Show(not self.experimentPanel.IsShown())
        text = ["Show", "Hide"][self.experimentPanel.IsShown()]
        self.showScanButton.SetLabel("%s experiment settings" % text)
        self.panel.SetSizerAndFit(self.panelSizer)
        self.SetClientSize(self.panel.GetSize())

    ## User checked/unchecked the "customize light frequencies" button.
    def onCustomizeLightFrequencies(self, event):
        self.lightFrequenciesPanel.Show(
            self.shouldCustomizeLightFrequencies.GetValue()
        )
        self.panel.Layout()
        self.panel.SetSizerAndFit(self.panelSizer)
        self.SetClientSize(self.panel.GetSize())

    ## Our experiment panel resized itself.
    def onExperimentPanelResize(self, panel):
        self.panel.SetSizerAndFit(self.panelSizer)
        self.SetClientSize(self.panel.GetSize())

    ## Our experiment panel needs to be reset.
    def onExperimentPanelReset(self):
        self.panelSizer.Remove(self.experimentPanel)
        self.experimentPanel.Destroy()
        self.experimentPanel = experimentConfigPanel.ExperimentConfigPanel(
            self.panel,
            resizeCallback=self.onExperimentPanelResize,
            resetCallback=self.onExperimentPanelReset,
            configKey="multiSiteExperimentPanel",
        )
        self.experimentPanel.filepath_panel.SetTemplate(_FILENAME_TEMPLATE)
        # Put the experiment panel back into the sizer immediately after
        # the button that shows/hides it.
        for i, item in enumerate(self.panelSizer.GetChildren()):
            if item.GetWindow() is self.showScanButton:
                self.panelSizer.Insert(
                    i + 1, self.experimentPanel, 0, wx.ALIGN_CENTER | wx.ALL, 5
                )
        self.panelSizer.Layout()
        self.Refresh()
        self.panel.SetSizerAndFit(self.panelSizer)
        return self.experimentPanel

    ## Show an error message to the user.
    def showError(self, message):
        wx.MessageBox(
            message,
            "Error",
            wx.OK | wx.ICON_ERROR | wx.STAY_ON_TOP,
            parent=self,
        )

    ## Build a MultiSiteSpec from the user's settings. Return None, after
    # telling the user why, if the settings are unusable.
    def getMultiSiteSpec(self):
        sites, frequencies = self.sitesPanel.getSitesList()
        if not sites:
            self.showError(
                "You must select sites before running the experiment."
            )
            return None
        try:
            numCycles = int(self.numCycles.GetValue())
            cycleDurations = [
                float(s)
                for s in self.cycleDuration.GetValue().split(",")
                if s.strip()
            ] or [0]
            delayBeforeStarting = (
                float(self.delayBeforeStarting.GetValue() or 0) * 60
            )
            delayBeforeImaging = float(self.delayBeforeImaging.GetValue() or 0)
            lightFrequencies = None
            if self.shouldCustomizeLightFrequencies.GetValue():
                # Lights with no frequency are used on every cycle.
                lightFrequencies = {
                    light: experimentSpecs.parseLightFrequency(control.GetValue())
                    for light, control in zip(
                        self.allLights, self.lightFrequencies
                    )
                    if control.GetValue().strip()
                }
        except ValueError as e:
            self.showError("Invalid setting: %s" % e)
            return None

        siteExperiment = self.experimentPanel.getExperimentSpec(
            useFilenameTemplate=True
        )
        if siteExperiment is None:
            return None

        spec = experimentSpecs.MultiSiteSpec(
            siteExperiment=siteExperiment,
            sites=sites,
            frequencies=frequencies,
            numCycles=numCycles,
            cycleDurations=cycleDurations,
            delayBeforeStarting=delayBeforeStarting,
            delayBeforeImaging=delayBeforeImaging,
            optimizeOrder=self.shouldOptimizeSiteOrder.GetValue(),
            lightFrequencies=lightFrequencies,
            powerDownWhenDone=(
                self.shouldPowerDownWhenDone is not None
                and self.shouldPowerDownWhenDone.GetValue()
            ),
        )
        try:
            spec.sanityCheck()
        except ValueError as e:
            self.showError("Experiment cancelled:\n\n%s" % e)
            return None
        return spec

    ## Start the experiment. It runs in a background thread so the user can
    # interact with the UI while the experiment runs.
    def onStart(self, event=None):
        if self.runner is not None and self.runner.is_running():
            self.showError("A multi-site experiment is already running.")
            return
        spec = self.getMultiSiteSpec()
        if spec is None:
            return
        self.saveConfig()
        self.Hide()
        self.runner = spec.run(confirm=_confirmInMainThread)

    ## Save our settings to the config.
    def saveConfig(self):
        lightFrequencies = [l.GetValue() for l in self.lightFrequencies]
        cockpit.util.userConfig.setValue(
            "multiSiteExperiment",
            {
                "numCycles": self.numCycles.GetValue(),
                "cycleDuration": self.cycleDuration.GetValue(),
                "delayBeforeStarting": self.delayBeforeStarting.GetValue(),
                "delayBeforeImaging": self.delayBeforeImaging.GetValue(),
                "shouldCustomizeLightFrequencies": self.shouldCustomizeLightFrequencies.GetValue(),
                "shouldOptimizeSiteOrder": self.shouldOptimizeSiteOrder.GetValue(),
                "lightFrequencies": lightFrequencies,
            },
        )

    ## Blow away the dialog and recreate it from scratch.
    def onReset(self, event):
        parent = self.GetParent()
        global dialog
        dialog.Destroy()
        dialog = None
        showDialog(parent)


## Global singleton
dialog = None


def showDialog(parent):
    global dialog
    if not dialog:
        dialog = MultiSiteExperimentDialog(parent)
    dialog.Show()
