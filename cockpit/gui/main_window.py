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

# This module creates the primary window.  This window houses widgets
# to control the most important hardware elements.  It is only
# responsible for setting up the user interface; it assume that the
# devices have already been initialized.

import io
import os.path
import typing
from configparser import ConfigParser
from importlib.metadata import version
from itertools import chain

import wx
import wx.adv

import cockpit.experiment.experiment
import cockpit.gui
import cockpit.gui.file_viewer_window
import cockpit.gui.gui_utils
import cockpit.interfaces.channels
import cockpit.util.user_config
from cockpit import depot, events
from cockpit.gui import (
    file_viewer_window,
    joystick,
    keyboard,
    main_panels,
    view_file_drop_target,
)
from cockpit.gui.dialogs.experiment import (
    multiSiteExperiment,
    singleSiteExperiment,
)
from cockpit.util.csv_plotter import CSVPlotter
from cockpit.util.intensity import IntensityProfilerFrame
from cockpit.util.value_logger import ValueLogger


ROW_SPACER = 12
COL_SPACER = 8


class MainWindowPanel(wx.Panel):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Find out what devices we have to work with.
        light_toggles = wx.GetApp().depot.get_handlers_of_type(depot.LIGHT_TOGGLE)

        ## Maps LightSource handlers to their associated panels of controls.
        self.light_to_panel = {}

        root_sizer = wx.BoxSizer(wx.VERTICAL)

        # A row of buttons for various actions we know we can take.
        button_sizer = wx.BoxSizer(wx.HORIZONTAL)
        # Abort button
        abort_button = wx.Button(self, wx.ID_ANY, label="ABORT")
        abort_button.SetForegroundColour(wx.RED)
        abort_button.SetFont(abort_button.GetFont().Bold().Larger())
        abort_button.Bind(
            wx.EVT_BUTTON, lambda event: events.publish(events.USER_ABORT)
        )
        button_sizer.Add(abort_button, 1, wx.EXPAND)

        # Snap image button
        snap_button = wx.Button(self, wx.ID_ANY, "Snap\nimage")
        snap_button.Bind(
            wx.EVT_BUTTON, lambda evt: wx.GetApp().imager.take_image()
        )
        button_sizer.Add(snap_button, 1, wx.EXPAND)

        # Video mode button
        video_button = wx.ToggleButton(self, wx.ID_ANY, "Live")
        video_button.Bind(
            wx.EVT_TOGGLEBUTTON, lambda evt: wx.GetApp().imager.video_mode()
        )
        events.subscribe(
            cockpit.events.VIDEO_MODE_TOGGLE,
            lambda state: video_button.SetValue(state),
        )
        button_sizer.Add(video_button, 1, wx.EXPAND)

        # Experiment & review buttons
        for label, fn in (
            (
                "Single-site\nexperiment",
                lambda event: singleSiteExperiment.showDialog(self),
            ),
            (
                "Multi-site\nexperiment",
                lambda event: multiSiteExperiment.showDialog(self),
            ),
            ("View last\nfile", self.on_view_last_file),
        ):
            button = wx.Button(self, wx.ID_ANY, label)
            button.Bind(wx.EVT_BUTTON, fn)
            button_sizer.Add(button, 1, wx.EXPAND)

        # Increase font size in top row buttons.
        for w in [child.GetWindow() for child in button_sizer.Children]:
            w.SetFont(w.GetFont().Larger())
        root_sizer.Add(button_sizer)
        root_sizer.AddSpacer(ROW_SPACER)

        # Make UIs for any other handlers / devices and insert them into
        # our window, if possible.
        # Light power things will be handled later.
        light_power_things = wx.GetApp().depot.get_handlers_of_type(
            depot.LIGHT_POWER
        )
        light_power_things.sort(key=lambda l: l.wavelength)
        # Camera UIs are drawn separately. Currently, they are drawn first,
        # but this separation may make it easier to implement cameras in
        # ordered slots, giving the user control over exposure order.
        camera_things = wx.GetApp().depot.get_handlers_of_type(depot.CAMERA)
        # Ignore anything that is handled specially.
        ignore_things = light_toggles + light_power_things
        ignore_things += camera_things
        # Remove ignore_things from the full list of devices.
        other_things = list(wx.GetApp().depot.get_all_devices())
        other_things.sort(key=lambda d: d.__class__.__name__)
        other_things.extend(wx.GetApp().depot.get_all_handlers())
        row_sizer = wx.WrapSizer(wx.HORIZONTAL)

        # Add objective control
        button_sizer.Add(
            main_panels.ObjectiveControls(self, wx.GetApp().objectives),
            flag=wx.LEFT,
            border=2,
        )
        ignore_things.extend(wx.GetApp().objectives.get_handlers())

        # Make the UI elements for the cameras.
        row_sizer.Add(main_panels.CameraControlsPanel(self))
        row_sizer.AddSpacer(COL_SPACER)

        # Add light controls.
        light_filters = sorted(
            wx.GetApp().depot.get_handlers_of_type(depot.LIGHT_FILTER)
        )
        ignore_things.extend(light_filters)

        # Add filterwheel controls.
        row_sizer.Add(main_panels.FilterControls(self))

        # Make the UI elements for everything else.
        for thing in ignore_things:
            if thing in other_things:
                other_things.remove(thing)
        for thing in sorted(other_things):
            if wx.GetApp().depot.get_handler(thing, depot.CAMERA):
                # Camera UIs already drawn.
                continue
            item = thing.make_ui(self)
            if item is not None:
                item_sizer = wx.BoxSizer(wx.VERTICAL)
                item_sizer.Add(
                    cockpit.gui.main_panels.PanelLabel(self, thing.name)
                )
                item_sizer.Add(item, 0, wx.EXPAND)
                if row_sizer.GetChildren():
                    # Add a spacer.
                    row_sizer.AddSpacer(COL_SPACER)
                row_sizer.Add(item_sizer, 1, flag=wx.EXPAND)

        root_sizer.Add(row_sizer, 1, flag=wx.EXPAND)
        root_sizer.AddSpacer(ROW_SPACER)

        lights_sizer = wx.BoxSizer(wx.HORIZONTAL)
        lights_sizer.Add(
            main_panels.LightControlsPanel(self), 0, flag=wx.EXPAND
        )
        lights_sizer.Add(main_panels.ChannelsPanel(self), 0, flag=wx.EXPAND)
        root_sizer.Add(lights_sizer, 1, flag=wx.EXPAND)
        self.SetSizer(root_sizer)
        self.Layout()

        keyboard.set_keyboard_handlers(self)
        self.joystick = joystick.Joystick(self)

        self.SetDropTarget(view_file_drop_target.ViewFileDropTarget(self))

        # subscribe to camera UPDATE_ROI events so we can ask to apply it to all
        # cameras or not.
        self.update_roi = False
        events.subscribe(events.UPDATE_ROI, self.update_cam_roi)

    def update_cam_roi(self, cam_name):
        """function fired on camera ROI update.

        It looks to see if there are other active cameras and asks if you want
        to set all cameras to that ROI or not.
        """
        # track if already updating cameras, only need to ask once
        if self.update_roi:
            return
        self.update_roi = True
        asked = False
        update = False
        active = wx.GetApp().depot.get_handler_with_name(cam_name)
        cameras = sorted(
            wx.GetApp().depot.get_handlers_of_type(depot.CAMERA),
            key=lambda c: c.name,
        )
        roi = active.get_roi()
        for camera in cameras:
            if camera.is_enabled and (camera.name is not cam_name):
                # another active camera camera so ask.
                title = "Apply to other Cameras?"
                msg = (
                    " You have added a new ROI to camera  '%s'."
                    " Do you wish to apply the same ROI to all "
                    " other cameras?" % (cam_name)
                )
                if not asked:
                    asked = True
                    if cockpit.gui.gui_utils.get_user_permission(msg, title):
                        # update ROI on this camera.
                        update = True
                if update:
                    camera.set_roi(roi)
        self.update_roi = False

    def on_view_last_file(self, event=None):
        """User clicked the "view last file" button; open the last experiment's
        file in an image viewer.

        A bit tricky when there's multiple files generated due to the splitting
        logic. We just view the first one in that case.
        """
        filenames = cockpit.experiment.experiment.get_last_filenames()
        if filenames:
            window = file_viewer_window.FileViewer(filenames[0], self)
            if len(filenames) > 1:
                print(
                    "Opening first of %d files. Others can be viewed by dragging them from the filesystem onto the main window of the Cockpit."
                    % len(filenames)
                )


def _show_configparser_frame(
    config: ConfigParser, frame_title="Configuration (parsed)"
) -> None:
    config_str = io.StringIO()
    config.write(config_str)

    frame = wx.Frame(parent=None, title=frame_title)
    panel = wx.Panel(parent=frame)
    text_ctrl = cockpit.gui.create_monospaced_multiline_text_ctrl(
        parent=panel, text=config_str.getvalue(), min_rows=24, min_cols=80
    )

    panel_sizer = wx.BoxSizer(wx.HORIZONTAL)
    panel_sizer.Add(text_ctrl, flags=wx.SizerFlags(1).Expand())
    panel.SetSizerAndFit(panel_sizer)

    frame_sizer = wx.BoxSizer(wx.HORIZONTAL)
    frame_sizer.Add(panel, flags=wx.SizerFlags(1).Expand())
    frame.SetSizerAndFit(frame_sizer)
    frame.Show()


class EditMenu(wx.Menu):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        menu_item = self.Append(wx.ID_ANY, item="Reset User Configuration")
        self.Bind(wx.EVT_MENU, self.on_reset_user_config, menu_item)

    def on_reset_user_config(self, evt: wx.CommandEvent) -> None:
        cockpit.util.user_config.clear_all_values()


class ChannelsMenu(wx.Menu):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        control_items = [
            ("Add channel…", self.on_add_channel),
            ("Remove channel…", self.on_remove_channel),
            ("Export channels…", self.on_export_channels),
            ("Import channels…", self.on_import_channels),
        ]
        for label, method in control_items:
            menu_item = self.Append(wx.ID_ANY, item=label)
            self.Bind(wx.EVT_MENU, method, menu_item)
        self.AppendSeparator()
        self._n_control_items = len(control_items) + 1  # +1 for the separator
        for name in wx.GetApp().channels.names:
            self.add_channel_item(name)

        wx.GetApp().channels.Bind(
            cockpit.interfaces.channels.EVT_CHANNEL_ADDED, self.on_channel_added
        )
        wx.GetApp().channels.Bind(
            cockpit.interfaces.channels.EVT_CHANNEL_REMOVED,
            self.on_channel_removed,
        )

    @property
    def channel_items(self) -> typing.List[wx.MenuItem]:
        """List of channel items in the menu."""
        channel_items = []
        for i, menu_item in enumerate(self.MenuItems):
            if i < self._n_control_items:
                continue  # skip control items
            channel_items.append(menu_item)
        return channel_items

    def add_channel_item(self, name: str) -> None:
        menu_item = self.Append(wx.ID_ANY, item=name)
        self.Bind(wx.EVT_MENU, self.on_channel, menu_item)

    def find_channel_item(self, channel_name: str) -> wx.MenuItem:
        """Find the channel menu item with the given channel name."""
        # wx.Menu.FindItem works perfectly to find the channel by name
        # but we deploy our own logic to cover the case of channels
        # named like one of our control menu items.  Sure, the only
        # reason to name a channel like that is to piss us off, but we
        # still need to handle it.
        for menu_item in self.channel_items:
            if menu_item.ItemLabelText == channel_name:
                return menu_item
        else:
            raise ValueError("There is no menu item named '%s'" % channel_name)

    def on_channel_added(self, event: wx.CommandEvent) -> None:
        channel_name = event.GetString()
        self.add_channel_item(channel_name)
        event.Skip()

    def on_channel_removed(self, event: wx.CommandEvent) -> None:
        channel_name = event.GetString()
        menu_item = self.find_channel_item(channel_name)
        self.Delete(menu_item)
        event.Skip()

    def on_add_channel(self, event: wx.CommandEvent) -> None:
        """Add current channel configuration."""
        name = wx.GetTextFromUser(
            "Enter name for new channel:", caption="Add new channel"
        )
        if not name:
            return

        if name in wx.GetApp().channels.names:
            answer = wx.MessageBox(
                'There is already a channel named "%s".' " Replace it?" % name,
                caption="Channel already exists",
                style=wx.YES_NO,
            )
            if answer == wx.YES:
                channel = cockpit.interfaces.channels.current_channel()
                wx.GetApp().channels.change(name, channel)
        else:
            channel = cockpit.interfaces.channels.current_channel()
            wx.GetApp().channels.add(name, channel)

    def on_remove_channel(self, event: wx.CommandEvent) -> None:
        """Remove one channel."""
        if not wx.GetApp().channels.names:
            wx.MessageBox(
                "There are no channels to be removed.",
                caption="Failed to remove channel",
                style=wx.OK,
            )
            return

        name = wx.GetSingleChoice(
            "Choose channel to be removed:",
            caption="Remove a channel",
            aChoices=wx.GetApp().channels.names,
        )
        if not name:
            return
        wx.GetApp().channels.remove(name)

    def on_export_channels(self, event: wx.CommandEvent) -> None:
        """Save all channels to a file."""
        filepath = wx.SaveFileSelector("Select file to export", "")
        if not filepath:
            return
        try:
            cockpit.interfaces.channels.save_to_file(
                filepath, wx.GetApp().channels
            )
        except:
            cockpit.gui.ExceptionBox("Failed to write to '%s'" % filepath)

    def on_import_channels(self, event: wx.CommandEvent) -> None:
        """Add all channels in a file."""
        filepath = wx.LoadFileSelector("Select file to import", "")
        if not filepath:
            return
        try:
            new_channels = cockpit.interfaces.channels.load_from_file(filepath)
        except:
            cockpit.gui.ExceptionBox("Failed to read to '%s'" % filepath)
        current_names = wx.GetApp().channels.names
        duplicated = [n for n in new_channels.names if n in current_names]
        if duplicated:
            answer = wx.MessageBox(
                "The import will overwrite the following"
                " channels: %s. Do you want to continue?"
                % ", ".join(duplicated),
                caption="Duplicated channels on loaded file",
                style=wx.YES_NO,
            )
            if answer != wx.YES:
                return
        wx.GetApp().channels.update(new_channels)

    def on_channel(self, event: wx.CommandEvent) -> None:
        """Apply channel with same name as the menu item."""
        name = self.FindItemById(event.GetId()).ItemLabelText
        channel = wx.GetApp().channels.get(name)
        cockpit.interfaces.channels.apply_channel(channel)


class WindowsMenu(wx.Menu):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self._id_to_window = {}  # type: typing.Dict[int, wx.Frame]

        menu_item = self.Append(wx.ID_ANY, item="Reset window positions")
        self.Bind(wx.EVT_MENU, self.on_reset_window_positions, menu_item)

        # A separator between "Reset window positions" and the Cockpit
        # window menu items.
        self.AppendSeparator()
        self._first_cockpit_window_pos = self.GetMenuItemCount()
        self._next_cockpit_window_pos = self.GetMenuItemCount()

        # A separator between the Cockpit window menu items and the
        # other extra windows.
        self.AppendSeparator()

        # Add item to launch valueLogViewer (XXX: this should be
        # handled by some sort of plugin system and not hardcoded).
        menu_item = self.Append(wx.ID_ANY, "Launch ValueLogViewer")
        self.Bind(wx.EVT_MENU, self.on_launch_value_log_viewer, menu_item)
        if not ValueLogger.get_log_files():
            menu_item.Enable(False)

        # Add item to launch SIM Intensity profile (XXX: this should
        # be handled by some sort of plugin system and not hardcoded).
        menu_item = self.Append(wx.ID_ANY, "Launch SIM Intensity Profile")
        self.Bind(wx.EVT_MENU, self.on_launch_sim_intensity_profile, menu_item)

        # This is only for the piDIO and executor, both of which are a
        # window to set lines high/low.  We should probably have a
        # general window for this which we could use for all executor
        # handlers (probably piDIO device should provide an executor
        # handler).
        for obj in chain(
            wx.GetApp().depot.get_all_handlers(),
            wx.GetApp().depot.get_all_devices(),
        ):
            if hasattr(obj, "show_debug_window"):
                label = "debug %s (%s)" % (obj.name, obj.__class__.__name__)
                menu_item = self.Append(wx.ID_ANY, label)
                self.Bind(
                    wx.EVT_MENU,
                    lambda e, obj=obj: obj.show_debug_window(),
                    menu_item,
                )

        # When the menu is created the windows don't exist yet so we
        # will update it each time the menu is open.
        self.Bind(wx.EVT_MENU_OPEN, self.on_menu_open)

    def on_menu_open(self, event: wx.MenuEvent) -> None:
        if event.GetMenu() is not self:
            # We may be just opening one of the submenus but we only
            # want to do this when opening the main menu.
            event.Skip()
            return

        main_window = wx.GetApp().GetTopWindow()
        all_windows = {
            w for w in wx.GetTopLevelWindows() if w is not main_window
        }

        new_menu_items = False
        for window in all_windows.difference(self._id_to_window.values()):
            ## Windows with this attribute need to exist but may be
            ## hidden (see #745)
            should_be_listed = getattr(window, "LIST_AS_COCKPIT_WINDOW", False)
            if not should_be_listed:
                continue
            menu_item = wx.MenuItem(self, wx.ID_ANY, window.Title)
            self.Bind(wx.EVT_MENU, self.on_window_title, menu_item)
            self._id_to_window[menu_item.Id] = window

            # Place this menu item between the separators that group
            # the cockpit windows.
            self.Insert(self._next_cockpit_window_pos, menu_item)
            self._next_cockpit_window_pos += 1
            new_menu_items = True

        if new_menu_items:
            self._sort_cockpit_window_menu_items()

    def _sort_cockpit_window_menu_items(self) -> None:
        # Remove menu items from the menu, pair them with the window,
        # sort the menu items based on the window name, and then
        # re-insert the menu items by order.
        menu_item_and_window_pairs = []
        for menu_item_id, window in self._id_to_window.items():
            menu_item = self.Remove(menu_item_id)
            menu_item_and_window_pairs.append((menu_item, window))

        menu_item_and_window_pairs.sort(key=lambda x: x[1].GetTitle())

        for i, (menu_item, _) in enumerate(menu_item_and_window_pairs):
            self.Insert(self._first_cockpit_window_pos + i, menu_item)

    def on_reset_window_positions(self, event: wx.CommandEvent) -> None:
        del event
        wx.GetApp().SetWindowPositions()

    def on_window_title(self, event: wx.CommandEvent) -> None:
        """Action when user selects the menu item with the window title."""
        window = self._id_to_window[event.GetId()]
        # Don't just call Restore() without checking if the window is
        # really iconized otherwise it might unmaximize a maximized
        # window when the user only wanted to bring it to the front.
        if window.IsIconized():
            window.Restore()
        # On GTK3 calling Raise() would be enough since it also calls
        # Show(), but on other platforms we do need to call Show()
        # first (see issue #599).  It's unclear what is the expected
        # wx behaviour (see https://trac.wxwidgets.org/ticket/18762)
        window.Show()
        window.Raise()

        # On Windows and OSX, when adding/removing displays, it is
        # possible that a window is at a position that no longer
        # exists.  So ensure that the window is shown at valid
        # coordinates.
        if wx.Display.GetFromWindow(window) == wx.NOT_FOUND:
            window.SetPosition(wx.GetMousePosition())

    def on_launch_value_log_viewer(self, event: wx.CommandEvent) -> None:
        log_files = ValueLogger.get_log_files()
        window = CSVPlotter(None)
        window.add_data_sources(log_files, defer_open=True)
        window.Show()

    def on_launch_sim_intensity_profile(self, event: wx.CommandEvent) -> None:
        window = IntensityProfilerFrame(wx.GetApp().MainWindow)
        window.Show()


class MainWindow(wx.Frame):
    def __init__(self):
        super().__init__(parent=None, title="Cockpit")
        panel = MainWindowPanel(self)

        menu_bar = wx.MenuBar()

        file_menu = wx.Menu()
        menu_item = file_menu.Append(wx.ID_OPEN)
        self.Bind(wx.EVT_MENU, self.on_open, menu_item)
        menu_item = file_menu.Append(wx.ID_ANY, item="View Cockpit config")
        self.Bind(wx.EVT_MENU, self.on_view_config, menu_item)
        menu_item = file_menu.Append(wx.ID_ANY, item="View depot config")
        self.Bind(wx.EVT_MENU, self.on_view_depot, menu_item)
        menu_item = file_menu.Append(wx.ID_EXIT)
        self.Bind(wx.EVT_MENU, self.on_quit, menu_item)
        menu_bar.Append(file_menu, "&File")

        edit_menu = EditMenu()
        menu_bar.Append(edit_menu, "&Edit")

        channels_menu = ChannelsMenu()
        menu_bar.Append(channels_menu, "&channels")

        menu_bar.Append(WindowsMenu(), "&Windows")

        help_menu = wx.Menu()
        menu_item = help_menu.Append(wx.ID_ANY, item="Online repository")
        self.Bind(
            wx.EVT_MENU,
            lambda evt: wx.LaunchDefaultBrowser(
                "https://github.com/microscope-cockpit/cockpit/"
            ),
            menu_item,
        )
        menu_item = help_menu.Append(wx.ID_ABOUT)
        self.Bind(wx.EVT_MENU, self._on_about, menu_item)
        menu_bar.Append(help_menu, "&Help")

        self.SetMenuBar(menu_bar)

        self.SetStatusBar(StatusLights(parent=self))

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(panel, 1, flag=wx.EXPAND | wx.ALL)
        sizer.Layout()
        sizer.SetSizeHints(self)
        self.SetSizer(sizer)

        self.Bind(wx.EVT_CLOSE, self.on_close)

        # Because mainPanels.PanelLabel uses a font larger than the
        # default, we need to recompute the Frame size at show time.
        # Workaround for https://trac.wxwidgets.org/ticket/16088
        if "gtk3" in wx.PlatformInfo:
            self.Bind(wx.EVT_SHOW, self.on_show)

        self.Bind(wx.EVT_SIZE, self.on_size)

    def on_size(self, event: wx.SizeEvent) -> None:
        self.Layout()
        self.SetMinSize(self.GetSizer().GetMinSize())
        self.Update()
        event.Skip()

    def on_show(self, event: wx.ShowEvent) -> None:
        self.Layout()
        event.Skip()

    def on_open(self, event: wx.CommandEvent) -> None:
        filepath = wx.LoadFileSelector("Select file to open", "", parent=self)
        if not filepath:
            return
        try:
            cockpit.gui.file_viewer_window.FileViewer(filepath, parent=self)
        except Exception as ex:
            cockpit.gui.ExceptionBox(
                "Failed to open '%s'" % filepath, parent=self
            )

    def on_view_config(self, event: wx.CommandEvent) -> None:
        _show_configparser_frame(
            wx.GetApp().config, frame_title="Cockpit configuration (parsed)"
        )

    def on_view_depot(self, event: wx.CommandEvent) -> None:
        _show_configparser_frame(
            wx.GetApp().config.depot_config,
            frame_title="depot configuration (parsed)",
        )

    def on_quit(self, event: wx.CommandEvent) -> None:
        self.Close()

    def on_close(self, event):
        """Close the main window, leads to close cockpit program.

        Do any necessary GUI pre-shutdown events here instead of
        CockpitApp.OnExit, since in that function all of the wx
        objects have been destroyed already.
        """
        if not event.CanVeto():
            MainWindow.destroy_all_windows()
        else:
            wx.GetApp()._save_window_positions()
            MainWindow.destroy_all_windows()
            # Let the default event handler handle the frame
            # destruction.
            event.Skip()

    def _on_about(self, event):
        wx.adv.AboutBox(CockpitAboutInfo(), parent=self)

    @staticmethod
    def destroy_all_windows():
        for w in wx.GetTopLevelWindows():
            wx.CallAfter(w.Destroy)


class StatusLights(wx.StatusBar):
    """A window status bar with the Cockpit status lights.

    The status bar can have any number of status light, each status
    light being a separate field.  New lights are created on the fly
    as required by publishing `UPDATE_STATUS_LIGHT` events.  The same
    event is used to update its text.
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        # Maps status light names to the light field/pane index.
        self._name_to_field = {}  # type: typing.Dict[str, int]
        self._defaultBackgroundColour = self.GetBackgroundColour()
        self._notificationColour = wx.YELLOW

        listener = cockpit.gui.EvtEmitter(self, events.UPDATE_STATUS_LIGHT)
        listener.Bind(cockpit.gui.EVT_COCKPIT, self._on_new_status)

        # Some lights that we know we need.
        events.publish(events.UPDATE_STATUS_LIGHT, "image count", "")
        events.publish(events.UPDATE_STATUS_LIGHT, "device waiting", "")

    def _add_new_light(self, light_name: str) -> None:
        """Append new status light to the status bar."""
        new_field_index = self.GetFieldsCount()  # type: int
        if not self._name_to_field:
            # If the map is empty, this is the first light.  However,
            # a status bar always has at least one field, so use the
            # existing field if this is the first light.
            assert new_field_index == 1
            new_field_index = 0
        else:
            self.SetFieldsCount(new_field_index + 1)
        self.SetStatusStyles([wx.SB_SUNKEN] * (new_field_index + 1))
        self._name_to_field[light_name] = new_field_index

    def _on_new_status(self, event: cockpit.gui.cockpit_event) -> None:
        """Update text of specified status light."""
        assert len(event.EventData) == 2
        light_name = event.EventData[0]  # type: str
        text = event.EventData[1]  # type: str
        if light_name not in self._name_to_field:
            self._add_new_light(light_name)
        self.SetStatusText(text, self._name_to_field[light_name])

        # This changes the colour of the whole bar, not only the
        # status (see issue #565).
        if any([self.GetStatusText(i) for i in range(self.FieldsCount)]):
            self.SetBackgroundColour(self._notificationColour)
        else:
            self.SetBackgroundColour(self._defaultBackgroundColour)
        # On Windows, we need to call Refresh() after
        # SetBackgroundColour() (see issue #654).
        self.Refresh()


def CockpitAboutInfo() -> wx.adv.AboutDialogInfo:
    # TODO: we should be reading all of the stuff here from somewhere
    # that is shared with setup.py.  Maybe we need our own metadata
    # class which this function would then convert.
    info = wx.adv.AboutDialogInfo()
    info.SetName("Cockpit")

    info.SetVersion(version("cockpit"))
    info.SetDescription("Hardware agnostic microscope user interface")
    info.SetCopyright(
        "Copyright © 2020\n"
        "\n"
        "Cockpit comes with absolutely no warranty.\n"
        "See the GNU General Public Licence, version 3 or later,"
        " for details."
    )

    # Authors are sorted alphabetically.
    for dev_name in [
        "Chris Weisiger",
        "Danail Stoychev",
        "David Miguel Susano Pinto",
        "Eric Branlund",
        "Ian Dobbie",
        "Julio Mateos-Langerak",
        "Mick Phillips",
        "Nicholas Hall",
        "Sebastian Hasse",
    ]:
        info.AddDeveloper(dev_name)

    # wxWidgets has native and generic implementations for the about
    # dialog.  However, native implementations other than GTK are
    # limited on the info they can include.  If website, custom icon
    # (instead of inherited from the parent), and license are used on
    # platforms other than GTK the generic dialog is used which we
    # want to avoid.
    if wx.Platform == "__WXGTK__":
        info.SetWebSite("https://microscope-cockpit.org/")

        # We should not have to set this, it should be set later via
        # the AboutBox parent icon.  We don't yet have icons working
        # (issue #388), but remove this when it is.
        info.SetIcon(
            wx.Icon(os.path.join(cockpit.gui.IMAGES_PATH, "cockpit-8bit.ico"))
        )

        info.SetLicence(
            "Cockpit is free software: you can redistribute it"
            " and/or modify\nit under the terms of the GNU General"
            " Public License as published by\nthe Free Software"
            " Foundation, either version 3 of the License, or\n(at"
            " your option) any later version\n"
            "\n"
            "Cockpit is distributed in the hope that it will be"
            " useful,\nbut WITHOUT ANY WARRANTY; without even the"
            " implied warranty of\nMERCHANTABILITY or FITNESS FOR A"
            " PARTICULAR PURPOSE.  See the\nGNU General Public"
            " License for more details.\n"
            "\n"
            "You should have received a copy of the GNU General"
            " Public License\nalong with Cockpit.  If not, see "
            " <http://www.gnu.org/licenses/>."
        )
    return info


def make_window():
    """Create the window."""
    window = MainWindow()
    return window
