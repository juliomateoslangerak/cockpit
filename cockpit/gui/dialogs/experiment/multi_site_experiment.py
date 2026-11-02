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


import time

import wx

import cockpit.gui.dialogs.enumerate_sites_panel
import cockpit.interfaces.stage_mover
import cockpit.util.threads
import cockpit.util.user_config
from cockpit import depot, events
from cockpit.gui import gui_utils
from cockpit.gui.dialogs.experiment import experiment_config_panel


## Minimum size of controls (counting their labels)
CONTROL_SIZE = (280, -1)
## Minimum size of text input fields.
FIELD_SIZE = (70, -1)

_FILENAME_TEMPLATE = "{date}-{time}_t{cycle}_p{site}.mrc"


class MultiSiteExperimentDialog(wx.Dialog):
    """This class allows for configuring multi-site experiments."""

    def __init__(self, parent):
        super().__init__(
            parent,
            title="Microscope multi-site experiment",
            style=wx.DEFAULT_DIALOG_STYLE | wx.RESIZE_BORDER,
        )

        ## Whether or not we should abort the current experiment.
        self.should_abort = False
        events.subscribe(events.USER_ABORT, self.on_abort)

        ## List of all light handlers.
        self.all_lights = wx.GetApp().depot.get_handlers_of_type(
            depot.LIGHT_TOGGLE
        )
        ## List of booleans indicating which lights were active at the
        # start of the experiment.
        self.active_lights = [None for l in self.all_lights]

        ## User's last-used inputs.
        self.settings = cockpit.util.user_config.get_value(
            "multiSiteExperiment",
            default={
                "num_cycles": "10",
                "cycle_duration": "60",
                "delay_before_starting": "0",
                "delay_before_imaging": "0",
                "should_customize_light_frequencies": False,
                "should_optimize_site_order": True,
                "light_frequencies": ["1" for l in self.all_lights],
            },
        )

        ## Contains self.panel
        self.sizer = wx.BoxSizer(wx.VERTICAL)

        ## Contains all UI widgets.
        self.panel = wx.Panel(self)
        ## Sizer for self.panel.
        self.panel_sizer = wx.BoxSizer(wx.VERTICAL)
        ## Sizer for all controls except the start/cancel/reset buttons.
        controls_sizer = wx.BoxSizer(wx.HORIZONTAL)
        ## Sizer for a single column of controls.
        column_sizer = wx.BoxSizer(wx.VERTICAL)
        ## Panel for selecting sites to visit.
        self.sites_panel = (
            cockpit.gui.dialogs.enumerate_sites_panel.EnumerateSitesPanel(
                self.panel,
                label="Sites to visit:",
                size=(200, -1),
                min_size=CONTROL_SIZE,
            )
        )
        column_sizer.Add(self.sites_panel)

        self.num_cycles = gui_utils.add_labeled_input(
            self.panel,
            column_sizer,
            label="Number of cycles:",
            default_value=self.settings["num_cycles"],
            size=FIELD_SIZE,
            min_size=CONTROL_SIZE,
        )

        self.cycle_duration = gui_utils.add_labeled_input(
            self.panel,
            column_sizer,
            label="Min cycle duration (s):",
            default_value=self.settings["cycle_duration"],
            size=FIELD_SIZE,
            min_size=CONTROL_SIZE,
            helper_string="Minimum amount of time to pass between each cycle. If the "
                          + "cycle finishes early, then I will wait until this much "
                          + "time has passed. You can enter multiple values here "
                          + "separated by commas; I will then use each wait time in "
                          + 'sequence; e.g. "60,120,180" means the first '
                          + "cycle takes one minute, the second two, the third three, "
                          + "the fourth one, the fifth two, and so on.",
        )

        self.delay_before_starting = gui_utils.add_labeled_input(
            self.panel,
            column_sizer,
            label="Delay before starting (min):",
            default_value=self.settings["delay_before_starting"],
            size=FIELD_SIZE,
            min_size=CONTROL_SIZE,
            helper_string="Amount of time to wait before starting the experiment. "
                          + "This is useful if you have a lengthy period to wait for "
                          + "your cells to reach the stage you're interested in, for "
                          + "example.",
        )

        self.delay_before_imaging = gui_utils.add_labeled_input(
            self.panel,
            column_sizer,
            label="Delay before imaging (s):",
            default_value=self.settings["delay_before_imaging"],
            size=FIELD_SIZE,
            min_size=CONTROL_SIZE,
            helper_string="Amount of time to wait after moving to a site before "
                          + "I start imaging the site. This is mostly useful if "
                          + "your stage needs time to stabilize after moving.",
        )

        controls_sizer.Add(column_sizer, 0, wx.ALL, 5)

        column_sizer = wx.BoxSizer(wx.VERTICAL)
        ## We don't necessarily have this option.
        self.should_power_down_when_done = None
        power_handlers = wx.GetApp().depot.get_handlers_of_type(
            depot.POWER_CONTROL
        )
        if power_handlers:
            # There are devices that we could potentially turn off at end of
            # experiment.
            self.should_power_down_when_done = gui_utils.add_labeled_input(
                self.panel,
                column_sizer,
                label="Power off devices when done:",
                control=wx.CheckBox(self.panel),
                label_height_adjustment=0,
                border=3,
                flags=wx.ALL,
                helper_string="If checked, then at the end of the experiment, I will "
                              + "power down all the devices I can.",
            )

        self.should_optimize_site_order = gui_utils.add_labeled_input(
            self.panel,
            column_sizer,
            label="Optimize route:",
            default_value=self.settings["should_optimize_site_order"],
            control=wx.CheckBox(self.panel),
            label_height_adjustment=0,
            border=3,
            flags=wx.ALL,
            helper_string="If checked, then I will calculate an ordering of the sites "
                          + "that will minimize the total time spent in transit; "
                          + "otherwise, I will use the order you specify.",
        )

        self.should_customize_light_frequencies = gui_utils.add_labeled_input(
            self.panel,
            column_sizer,
            label="Customize light frequencies:",
            default_value=self.settings["should_customize_light_frequencies"],
            control=wx.CheckBox(self.panel),
            label_height_adjustment=0,
            border=3,
            flags=wx.ALL,
            helper_string="This allows you to set up experiments where different "
                          + "light sources are enabled for different cycles. If you "
                          + "set a frequency of 5 for a given light, for example, "
                          + "then that light will only be used for every 5th pass "
                          + "(the 1st, 6th, 11th, etc. cycles). You can specify an "
                          + 'offset, too: "5 + 1" would enable the light for the '
                          + "2nd, 7th, 12th, etc. cycles.",
        )
        self.should_customize_light_frequencies.Bind(
            wx.EVT_CHECKBOX, self.on_customize_light_frequencies
        )
        self.light_frequencies_panel = wx.Panel(
            self.panel, style=wx.BORDER_SUNKEN | wx.TAB_TRAVERSAL
        )
        self.light_frequencies, sizer = gui_utils.make_lights_controls(
            self.light_frequencies_panel,
            [str(l.wavelength) for l in self.all_lights],
            self.settings["light_frequencies"],
        )
        self.light_frequencies_panel.SetSizerAndFit(sizer)
        self.light_frequencies_panel.Show(
            self.settings["should_customize_light_frequencies"]
        )
        column_sizer.Add(
            self.light_frequencies_panel, 0, wx.LEFT | wx.RIGHT | wx.BOTTOM, 5
        )

        controls_sizer.Add(column_sizer, 0, wx.ALL, 5)
        self.panel_sizer.Add(controls_sizer)

        ## Controls whether or not the scanning experiment's parameters are
        # shown.
        self.show_scan_button = wx.Button(
            self.panel, -1, "Show experiment settings"
        )
        self.show_scan_button.Bind(wx.EVT_BUTTON, self.on_show_scan_button)
        self.panel_sizer.Add(
            self.show_scan_button, 0, wx.ALIGN_CENTER | wx.TOP, 5
        )
        ## This panel configures the experiment we perform when visiting sites.
        self.experiment_panel = experiment_config_panel.ExperimentConfigPanel(
            self.panel,
            resize_callback=self.on_experiment_panel_resize,
            reset_callback=self.on_experiment_panel_reset,
            config_key="multi_site_experiment_panel",
        )
        self.experiment_panel.filepath_panel.set_template(_FILENAME_TEMPLATE)
        self.panel_sizer.Add(
            self.experiment_panel, 0, wx.ALIGN_CENTER | wx.ALL, 5
        )
        self.experiment_panel.Hide()

        button_sizer = wx.BoxSizer(wx.HORIZONTAL)

        button = wx.Button(self.panel, -1, "Reset")
        button.SetToolTip(
            wx.ToolTip("Reload this window with all default values")
        )
        button.Bind(wx.EVT_BUTTON, self.on_reset)
        button_sizer.Add(button, 0, wx.ALL, 5)

        button_sizer.Add((1, 0), 1, wx.EXPAND)

        button = wx.Button(self.panel, wx.ID_CANCEL, "Cancel")
        button_sizer.Add(button, 0, wx.ALL, 5)

        button = wx.Button(self.panel, wx.ID_OK, "Start")
        button.SetToolTip(wx.ToolTip("Start the experiment"))
        button.Bind(wx.EVT_BUTTON, self.on_start)
        button_sizer.Add(button, 0, wx.ALL, 5)

        self.panel_sizer.Add(button_sizer, 0, wx.ALL, 5)
        self.panel.SetSizerAndFit(self.panel_sizer)
        self.sizer.Add(self.panel)
        self.SetSizerAndFit(self.sizer)

    def on_show_scan_button(self, event):
        """User clicked the show/hide scanning experiment button."""
        self.experiment_panel.Show(not self.experiment_panel.IsShown())
        text = ["Show", "Hide"][self.experiment_panel.IsShown()]
        self.show_scan_button.SetLabel("%s experiment settings" % text)
        self.panel.SetSizerAndFit(self.panel_sizer)
        self.SetClientSize(self.panel.GetSize())

    def on_customize_light_frequencies(self, event):
        """User checked/unchecked the "customize light frequencies" button."""
        self.light_frequencies_panel.Show(
            self.should_customize_light_frequencies.GetValue()
        )
        self.panel.Layout()
        self.panel.SetSizerAndFit(self.panel_sizer)
        self.SetClientSize(self.panel.GetSize())

    def on_experiment_panel_resize(self, panel):
        """Our experiment panel resized itself."""
        self.panel.SetSizerAndFit(self.panel_sizer)
        self.SetClientSize(self.panel.GetSize())

    def on_experiment_panel_reset(self):
        """Our experiment panel needs to be reset."""
        self.panel_sizer.Remove(self.experiment_panel)
        self.experiment_panel.Destroy()
        self.experiment_panel = experiment_config_panel.ExperimentConfigPanel(
            self.panel,
            resize_callback=self.on_experiment_panel_resize,
            reset_callback=self.on_experiment_panel_reset,
            config_key="multi_site_experiment_panel",
        )
        self.experiment_panel.filepath_panel.set_template(_FILENAME_TEMPLATE)
        # Put the experiment panel back into the sizer immediately after
        # the button that shows/hides it.
        for i, item in enumerate(self.panel_sizer.GetChildren()):
            if item.GetWindow() is self.show_scan_button:
                self.panel_sizer.Insert(
                    i + 1, self.experiment_panel, 0, wx.ALIGN_CENTER | wx.ALL, 5
                )
        self.panel_sizer.Layout()
        self.Refresh()
        self.panel.SetSizerAndFit(self.panel_sizer)
        return self.experiment_panel

    def choose_site_visit_order(self):
        """Analyze the user's chosen sites to visit and how often they should
        be visited, and come up with a sequence of sites to visit on each cycle
        that minimizes total travel time.

        Return a tuple of (total number of site lists, mapping of cycle number
        to site list).
        """
        base_indices, frequencies = self.sites_panel.get_sites_list()
        # Check for sites that have been deleted
        base_order = []
        base_frequencies = []
        for i, site_id in enumerate(base_indices):
            if cockpit.interfaces.stage_mover.does_site_exist(site_id):
                base_order.append(site_id)
                base_frequencies.append(frequencies[i])
        cycle_rate = 1
        seen_freqs = set()
        # Generate a cycle rate (number of unique sets of sites to visit).
        # Our approach will result in some redundancies, but that's
        # not a huge deal.
        for freq in base_frequencies:
            if freq not in seen_freqs:
                cycle_rate *= freq
                seen_freqs.add(freq)
        cycle_num_to_sites_list = []
        for i in range(cycle_rate):
            sites_list = []
            for site_id, frequency in zip(base_order, base_frequencies):
                if i % frequency == 0:
                    sites_list.append(site_id)
            if self.should_optimize_site_order.GetValue():
                cycle_num_to_sites_list.append(
                    cockpit.interfaces.stage_mover.optimised_site_order(sites_list)
                )
            else:
                cycle_num_to_sites_list.append(sites_list)
        return (cycle_rate, cycle_num_to_sites_list)

    def sanity_check(self):
        """Run sanity checks before starting the experiment.

        Return True if all is well, False otherwise.
        """
        if not self.sites_panel.get_sites_list():
            wx.MessageBox(
                "You must select sites before running the experiment.",
                "Error",
                wx.OK | wx.ICON_ERROR | wx.STAY_ON_TOP,
            )
            return False
        # Verify that all sites are reachable; the user may have
        # restarted the cockpit (thus resetting motion safeties) and then
        # loaded a list of sites which we cannot now reach.
        for site_id in self.sites_panel.get_sites_list()[0]:
            if not cockpit.interfaces.stage_mover.does_site_exist(site_id):
                wx.MessageBox(
                    "Experiment cancelled:\n\nSite %s does not exist."
                    % site_id,
                    "Error",
                    wx.OK | wx.ICON_ERROR | wx.STAY_ON_TOP,
                )
                return False
            if not cockpit.interfaces.stage_mover.can_reach_site(site_id):
                wx.MessageBox(
                    "Experiment cancelled:\n\nSite %s cannot be reached."
                    % site_id,
                    "Error",
                    wx.OK | wx.ICON_ERROR | wx.STAY_ON_TOP,
                )
                return False
        return True

    @cockpit.util.threads.call_in_new_thread
    def on_start(self, event=None):
        """Run the experiment.

        We spin this off to a background thread so the user can interact with
        the UI while the experiment runs.
        """
        if not self.sanity_check():
            return
        self.Hide()
        self.save_config()

        self.active_lights = [l.get_is_enabled() for l in self.all_lights]

        self.should_abort = False
        delay = float(self.delay_before_starting.GetValue()) * 60
        self.wait_for(delay)
        if self.should_abort:
            return

        experiment_start = time.localtime()
        cycle_duration = 0
        if self.cycle_duration.GetValue():
            cycle_duration = float(self.cycle_duration.GetValue())
        cycle_period, cycle_num_to_sites_list = self.choose_site_visit_order()
        num_cycles = int(self.num_cycles.GetValue())
        cycle_start_time = time.time()
        for cycle_num in range(num_cycles):
            site_ids = cycle_num_to_sites_list[cycle_num % cycle_period]
            if cycle_num != 0:
                # Move to the first site.
                cockpit.interfaces.stage_mover.wait_for_stop()
                cockpit.interfaces.stage_mover.go_to_site(
                    site_ids[0], should_block=True
                )
                # Wait for when the next cycle should start.
                wait_time = cycle_start_time + cycle_duration - time.time()
                if not self.wait_for(wait_time):
                    print(
                        f"Couldn't finish cycle in time; off by {-wait_time:.2f} seconds"
                    )
            print(
                f"Starting cycle {cycle_num + 1} of {num_cycles} at {time.time():.2f}"
            )
            cycle_start_time = time.time()
            if self.should_customize_light_frequencies.GetValue():
                self.activate_lights(cycle_num)
            for site_id in site_ids:
                if self.should_abort:
                    break
                print(f"Imaging site {site_id} at {time.time():.2f}")
                self.image_site(site_id, cycle_num, experiment_start)

            if self.should_abort:
                break
            if self.should_abort:
                break

        self.clean_up()

    def clean_up(self):
        """Clean up after the experiment ends."""
        for i, should_activate in enumerate(self.active_lights):
            self.all_lights[i].set_enabled(should_activate)
        if (
            self.should_power_down_when_done is not None
            and self.should_power_down_when_done.GetValue()
        ):
            handlers = wx.GetApp().depot.get_handlers_of_type(depot.POWER_CONTROL)
            for handler in handlers:
                handler.disable()
        events.publish(events.UPDATE_STATUS_LIGHT, "device waiting", "")

    def activate_lights(self, cycleNum):
        """Select the appropriate light sources for this cycle."""
        for i, control in enumerate(self.light_frequencies):
            string = control.GetValue()
            frequency = 1
            offset = 0
            if "+" in string:
                # There's both a frequency and an offset.
                frequency, offset = [int(s) for s in string.split("+")]
            else:
                # Just a frequency.
                frequency = int(string)
            # Only activate a light if it was enabled when the experiment
            # started.
            if self.active_lights[i]:
                self.all_lights[i].set_enabled(cycleNum % frequency == offset)

    def image_site(self, site_id, cycle_num, experiment_start):
        """Go to the specified site and run our experiment on it."""
        events.publish(
            events.UPDATE_STATUS_LIGHT,
            "device waiting",
            "Waiting for stage motion",
        )
        cockpit.interfaces.stage_mover.wait_for_stop()
        cockpit.interfaces.stage_mover.go_to_site(site_id, should_block=True)
        self.wait_for(float(self.delay_before_imaging.GetValue()))
        if self.should_abort:
            return
        # Try casting the site ID to an int, which it probably is, so that
        # we can use %03d (fills with zeros) instead of %s (variable width,
        # so screws with sorting).
        try:
            site_id = "%03d" % int(site_id)
        except ValueError:
            # Not actually an int.
            pass
        self.experiment_panel.filepath_panel.update_filename(
            {
                "date": time.strftime("%Y%m%d", experiment_start),
                "time": time.strftime("%H%M", experiment_start),
                "cycle": ("%03d" % cycle_num),
                "site": site_id,
            }
        )
        start = time.time()
        events.execute_and_wait_for(
            events.EXPERIMENT_COMPLETE, self.experiment_panel.run_experiment
        )
        print(f"Imaging took {(time.time() - start):.2f} seconds")

    def on_abort(self, *args):
        """User clicked the abort button."""
        self.should_abort = True

    def wait_for(self, seconds):
        """Wait for some time, allowing the user to abort the wait.

        Return True if we were successful (i.e. handed a valid amount of time
        to wait for).
        """
        if seconds <= 0:
            return False
        print(f"Waiting for {seconds:.2f} seconds until next cycle")
        end_time = time.time() + seconds
        cur_time = time.time()
        while cur_time < end_time and not self.should_abort:
            if int(cur_time + 0.25) != int(cur_time):
                remaining = end_time - cur_time
                # Advanced to a new second; update the status light.
                display_minutes = remaining // 60
                display_seconds = (remaining - display_minutes * 60) // 1
                events.publish(
                    events.UPDATE_STATUS_LIGHT,
                    "device waiting",
                    (
                        "Waiting for %02d:%02d"
                        % (display_minutes, display_seconds)
                    ),
                )
            time.sleep(0.25)
            cur_time = time.time()
        return True

    def save_config(self):
        """Save our settings to the config."""
        light_frequencies = [l.GetValue() for l in self.light_frequencies]
        cockpit.util.user_config.set_value(
            "multiSiteExperiment",
            {
                "num_cycles": self.num_cycles.GetValue(),
                "cycle_duration": self.cycle_duration.GetValue(),
                "delay_before_starting": self.delay_before_starting.GetValue(),
                "delay_before_imaging": self.delay_before_imaging.GetValue(),
                "should_customize_light_frequencies": self.should_customize_light_frequencies.GetValue(),
                "should_optimize_site_order": self.should_optimize_site_order.GetValue(),
                "light_frequencies": light_frequencies,
            },
        )

    def on_reset(self, event):
        """Blow away the dialog and recreate it from scratch."""
        parent = self.GetParent()
        global dialog
        dialog.Destroy()
        dialog = None
        show_dialog(parent)


## Global singleton
dialog = None


def show_dialog(parent):
    global dialog
    if not dialog:
        dialog = MultiSiteExperimentDialog(parent)
    dialog.Show()
