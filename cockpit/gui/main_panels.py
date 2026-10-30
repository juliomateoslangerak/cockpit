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

import wx

import cockpit.interfaces.channels
from cockpit import depot
from cockpit.gui import safe_controls
from cockpit.gui.device import EnableButton
from cockpit.util.colors import wavelength_to_color


class PanelLabel(wx.StaticText):
    """A formatted label for panels of controls."""

    def __init__(self, parent, label=""):
        super().__init__(parent, label=label)
        self.SetFont(self.GetFont().Bold().Larger().Larger())
        self.Fit()


class LightPanel(wx.Panel):
    """A panel of controls for a single light source."""

    def __init__(self, parent, light_toggle, light_power=None, light_filters=[]):
        super().__init__(parent, style=wx.BORDER_RAISED)
        self.light = light_toggle
        self.light_power = light_power
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.button = EnableButton(self, self.light)
        self.button.set_state(self.light.state)

        self.exposure_ctrl = safe_controls.SafeSpinCtrlDouble(self, inc=5)
        self.exposure_ctrl.Bind(
            safe_controls.EVT_SAFE_CONTROL_COMMIT,
            lambda event: self.light.set_exposure_time(event.Value),
        )
        light_toggle.add_watch("exposure_time", self.exposure_ctrl.SetValue)
        self.exposure_ctrl.SetValue(light_toggle.exposure_time)

        self.sizer.Add(self.button, flag=wx.EXPAND)
        self.sizer.AddSpacer(2)
        line_height = int(self.GetFont().GetFractionalPointSize() / 2.0)
        line = wx.StaticText(self, label="", size=(-1, line_height))
        line.SetBackgroundColour(wavelength_to_color(self.light.wavelength))
        self.sizer.Add(line, flag=wx.EXPAND)

        self.sizer.Add(
            wx.StaticText(self, label="Exposure / ms"),
            flag=wx.ALIGN_CENTER_HORIZONTAL,
        )
        self.sizer.Add(self.exposure_ctrl, flag=wx.EXPAND)

        if light_power is not None:
            self.sizer.AddSpacer(4)
            self.sizer.Add(
                wx.StaticText(self, label="Power (%)"),
                flag=wx.ALIGN_CENTER_HORIZONTAL,
            )
            self.power_ctrl = safe_controls.SpinGauge(
                self,
                minValue=0.0,
                maxValue=100.0,
                fetch_current=lambda: light_power.get_power() * 100.0,
            )
            self.power_ctrl.SetValue(light_power.power_set_point * 100.0)
            light_power.add_watch("power_set_point", self._set_power_percent)
            self.power_ctrl.Bind(
                safe_controls.EVT_SAFE_CONTROL_COMMIT,
                lambda event: light_power.set_power(event.Value / 100.0),
            )
            self.sizer.Add(self.power_ctrl, 1, flag=wx.EXPAND)
            self._timer = wx.Timer(self)
            # set timer for 500 ms for each channel
            self._timer.Start(500)
            self.Bind(wx.EVT_TIMER, self.on_timer)

        if light_filters:
            self.sizer.AddSpacer(4)
            self.sizer.Add(
                wx.StaticText(self, label="Filters"),
                flag=wx.ALIGN_CENTER_HORIZONTAL,
            )
            for f in light_filters:
                self.sizer.Add(f.make_selector(self), flag=wx.EXPAND)

        self.Bind(wx.EVT_WINDOW_DESTROY, self.on_destroy)

        self.sizer.Layout()
        self.SetSizerAndFit(self.sizer)

    def on_timer(self, evt):
        """function that wx.timer calls to check remote power."""
        self.light_power.last_power = self.light_power.callbacks["get_power"]()

    def on_destroy(self, event: wx.WindowDestroyEvent) -> None:
        self.light.remove_watch("exposure_time", self.exposure_ctrl.SetValue)
        if self.light_power is not None:
            self.light_power.remove_watch("power_set_point", self._set_power_percent)
            self._timer.Stop()
        event.Skip()

    def _set_power_percent(self, power):
        self.power_ctrl.SetValue(power * 100.0)

    def SetFocus(self):
        # Sets focus to the main button to avoid accidental data entry
        # in power or exposure controls.
        self.button.SetFocus()

    def on_status(self, event):
        light, state = event.EventData
        if light != self.light:
            return
        self.button.set_state(state)


class LightControlsPanel(wx.Panel):
    """Creates a LightPanel for each light source."""

    def __init__(self, parent):
        super().__init__(parent)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(PanelLabel(self, label="Lights"))
        single_light_sizer = wx.BoxSizer(wx.HORIZONTAL)
        self.sizer.Add(single_light_sizer)

        light_toggles = sorted(
            depot.get_handlers_of_type(depot.LIGHT_TOGGLE),
            key=lambda l: l.wavelength,
        )
        light_powers = depot.get_handlers_of_type(depot.LIGHT_POWER)
        light_filters = list(
            filter(
                lambda f: f.lights, depot.get_handlers_of_type(depot.LIGHT_FILTER)
            )
        )
        self.panels = {}
        for light in light_toggles:
            power = next(
                filter(lambda p: p.group_name == light.group_name, light_powers),
                None,
            )
            filters = list(
                filter(lambda f: light.name in f.lights, light_filters)
            )
            panel = LightPanel(self, light, power, filters)
            single_light_sizer.Add(panel, flag=wx.EXPAND)
            self.panels[light] = panel
            single_light_sizer.AddSpacer(4)
        self.sizer.Layout()
        self.SetSizerAndFit(self.sizer)


class CameraPanel(wx.Panel):
    """A panel of controls for a single camera."""

    def __init__(self, parent, camera):
        super().__init__(parent, style=wx.BORDER_RAISED)
        self.camera = camera
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.button = EnableButton(self, self.camera)
        self.button.set_state(self.camera.state)
        self.sizer.Add(self.button, flag=wx.EXPAND)
        self.sizer.AddSpacer(2)

        line_height = int(self.GetFont().GetFractionalPointSize() / 2.0)
        self.line = wx.StaticText(self, label="", size=(-1, line_height))
        self.line.SetBackgroundColour(
            wavelength_to_color(self.camera.wavelength or 0)
        )
        self.sizer.Add(self.line, flag=wx.EXPAND)
        # If there are problems here, it's because the inline function below is
        # being called outside of the main thread and needs taking out and
        # wrapping with wx.CallAfter.
        camera.add_watch("wavelength", self.on_wavelength_change)
        self.sizer.AddSpacer(2)

        if camera.callbacks.get("make_ui", None):
            self.sizer.Add(
                camera.callbacks["make_ui"](self), wx.SizerFlags().Expand()
            )

        self.Bind(wx.EVT_WINDOW_DESTROY, self.on_destroy)

        self.sizer.Layout()
        self.SetSizerAndFit(self.sizer)

    def on_destroy(self, event: wx.WindowDestroyEvent) -> None:
        self.camera.remove_watch("wavelength", self.on_wavelength_change)
        event.Skip()

    def on_wavelength_change(self, wavelength):
        """Change the colour of our wavelength indicator."""
        self.line.SetBackgroundColour(wavelength_to_color(wavelength or 0))
        # Explicit refresh required under MSW.
        wx.CallAfter(self.line.Refresh)

    def SetFocus(self):
        # Sets focus to the main button to avoid accidental data entry
        # in power or exposure controls.
        self.button.SetFocus()

    def on_status(self, event):
        camera, state = event.EventData
        if camera != self.camera:
            return
        self.button.set_state(state)


class CameraControlsPanel(wx.Panel):
    """Creates a CameraPanel for each camera."""

    def __init__(self, parent):
        super().__init__(parent)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        single_camera_sizer = wx.BoxSizer(wx.HORIZONTAL)
        label = PanelLabel(self, label="Cameras")
        self.sizer.Add(label)
        self.sizer.Add(single_camera_sizer)

        cameras = sorted(
            depot.get_handlers_of_type(depot.CAMERA), key=lambda c: c.name
        )

        self.panels = {}

        for cam in cameras:
            panel = CameraPanel(self, cam)
            single_camera_sizer.Add(panel, flag=wx.EXPAND)
            self.panels[cam] = panel
            single_camera_sizer.AddSpacer(4)
        self.sizer.Layout()
        self.SetSizerAndFit(self.sizer)


class ObjectiveControls(wx.Panel):
    """A panel with an objective selector."""

    def __init__(
        self,
        parent: wx.Window,
        interface: cockpit.interfaces.Objectives,
    ) -> None:
        super().__init__(parent)
        self._interface = interface
        label = PanelLabel(self, label="Objective")
        self._choice = wx.Choice(self, choices=interface.get_names_sorted())
        if not self._choice.SetStringSelection(interface.get_name()):
            raise Exception(
                "failed to find objective '%s'" % interface.get_name()
            )

        self._choice.Bind(wx.EVT_CHOICE, self._on_objective_choice)
        self._interface.Bind(
            cockpit.interfaces.EVT_OBJECTIVE_CHANGED,
            self._on_objective_changed,
        )

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(label)
        sizer.Add(self._choice)
        sizer.Layout()
        self.SetSizerAndFit(sizer)

    def _on_objective_choice(self, event: wx.CommandEvent) -> None:
        self._interface.change_objective(event.GetString())

    def _on_objective_changed(self, event: wx.CommandEvent) -> None:
        if not self._choice.SetStringSelection(event.GetString()):
            raise Exception(
                "failed to find objective '%s'" % event.GetString()
            )
        event.Skip()


class FilterControls(wx.Panel):
    """A panel with controls for all filter wheels."""

    def __init__(self, parent):
        super().__init__(parent)
        self.sizer = wx.BoxSizer(wx.VERTICAL)
        self.sizer.Add(PanelLabel(self, label="Filters"))
        subpanel = wx.Panel(self, style=wx.BORDER_RAISED)
        self.sizer.Add(subpanel, 1, wx.EXPAND)
        subpanel.Sizer = wx.WrapSizer(orient=wx.VERTICAL)

        filters = depot.get_handlers_of_type(depot.LIGHT_FILTER)
        if not filters:
            self.Hide()
            return

        for i, f in enumerate(filters):
            subpanel.Sizer.Add(
                f.make_ui(subpanel), 0, wx.EXPAND | wx.RIGHT | wx.BOTTOM, 8
            )
        self.sizer.Layout()
        self.SetSizerAndFit(self.sizer)


class ChannelsPanel(wx.Panel):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        label = PanelLabel(self, label="channels")
        self._buttons_sizer = wx.WrapSizer(wx.VERTICAL)

        for name in wx.GetApp().channels.names:
            self.add_button(name)

        wx.GetApp().channels.Bind(
            cockpit.interfaces.channels.EVT_CHANNEL_ADDED, self.on_channel_added
        )
        wx.GetApp().channels.Bind(
            cockpit.interfaces.channels.EVT_CHANNEL_REMOVED,
            self.on_channel_removed,
        )

        sizer = wx.BoxSizer(wx.VERTICAL)
        sizer.Add(label)
        sizer.Add(self._buttons_sizer, wx.SizerFlags().Expand())
        sizer.Layout()
        self.SetSizerAndFit(sizer)

    def _layout_with_frame(self):
        self.Layout()
        # When we add a new button, we may require a new column.  When
        # that happens, it's not enough to call Layout(), the parent
        # sizer also needs to make space for our new needs, which
        # comes all the way up from the frame sizer itself, which is
        # why also call Layout() on the Frame. See
        # https://stackoverflow.com/questions/62411713
        frame = wx.GetTopLevelParent(self)
        frame.Layout()
        # But even calling Layout() on the frame may not be enough if
        # the frame itself needs to be resized.  But we can't just
        # call Fit() otherwise we may shrink the window.  We only want
        # to make it wider if required.
        if frame.BestSize[0] > frame.Size[0]:
            frame.SetSize(frame.BestSize[0], frame.Size[1])

    def add_button(self, name: str) -> None:
        button = wx.Button(self, label=name)
        button.Bind(wx.EVT_BUTTON, self.on_button)
        self._buttons_sizer.Add(button, wx.SizerFlags().Expand())
        self._layout_with_frame()

    def remove_button(self, name: str) -> None:
        button = self.get_button_by_label(name)
        self._buttons_sizer.Detach(button)
        button.Destroy()
        self._layout_with_frame()

    def get_button_by_label(self, name: str) -> wx.Button:
        for sizer_item in self._buttons_sizer.Children:
            if sizer_item.Window.LabelText == name:
                return sizer_item.Window
        else:
            raise ValueError("There is no button named '%s'" % name)

    def on_channel_added(self, event: wx.CommandEvent) -> None:
        channel_name = event.GetString()
        self.add_button(channel_name)

    def on_channel_removed(self, event: wx.CommandEvent) -> None:
        channel_name = event.GetString()
        self.remove_button(channel_name)

    def on_button(self, event: wx.CommandEvent) -> None:
        """Apply channel with same name as the button."""
        name = event.EventObject.Label
        channel = wx.GetApp().channels.get(name)
        cockpit.interfaces.channels.apply_channel(channel)
