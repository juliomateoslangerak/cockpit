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


"""Microscope devices.

   Supports devices that implement the interface defined in
   microscope.devices

For a direct connection::

    [device]
    type: cockpit.devices.a_module.SomeClass
    uri: PYRO:SomeDevice@host.port

For connection via a controller::

    [device]
    type: cockpit.devices.a_module.SomeClass
    controller: some_controller
    controller.name: remote_device    # optional

    [some_controller]
    type: cockpit.devices.another_module.AnotherClass
    uri: PYRO:SomeControler@host.port

"""

import collections.abc
import logging
import re
import time
import typing

import Pyro4
import wx
from microscope import AxisLimits

import cockpit.gui.device
import cockpit.gui.gui_utils
import cockpit.handlers.device
import cockpit.handlers.digital_io
import cockpit.handlers.filter
import cockpit.handlers.light_power
import cockpit.handlers.light_source
import cockpit.util.colors
import cockpit.util.listener
import cockpit.util.threads
import cockpit.util.user_config
from cockpit import depot, events
from cockpit.devices import device
from cockpit.gui.device import SettingsEditor
from cockpit.handlers.stage_positioner import PositionerHandler
from cockpit.interfaces import stage_mover
from cockpit.util import value_logger


_logger = logging.getLogger(__name__)


# Pseudo-enum to track whether device defaults in place.
DEFAULTS_NONE, DEFAULTS_PENDING, DEFAULTS_SENT = range(3)


class MicroscopeBase(device.Device):
    """A class to communicate with the UniversalDevice interface."""

    def __init__(self, name, config):
        super().__init__(name, config)
        self.handlers = []
        self.panel = None
        # Pyro proxy
        self._proxy = None
        self.settings = {}
        self.cached_settings = {}
        self.settings_editor = None
        self.defaults = DEFAULTS_NONE
        self.enabled = True
        # Placeholders for methods deferred to proxy.
        self.get_all_settings = None
        self.get_setting = None
        self.set_setting = None
        self.describe_setting = None
        self.describe_settings = None

    def initialize(self):
        super().initialize()
        # Connect to the proxy.
        if "controller" not in self.config:
            self._proxy = Pyro4.Proxy(self.uri)
        else:
            c = depot.get_device_with_name(self.config["controller"])
            c_name = self.config.get("controller.name", None)
            if c_name is not None:
                try:
                    self._proxy = c._proxy.devices[c_name]
                except:
                    raise Exception(
                        "%s: device not found on controller '%s'."
                        % (self.name, c.name)
                    )
            elif len(c._proxy.devices) == 0:
                raise Exception(
                    "%s: no devices found on controller." % self.name
                )
            elif len(c._proxy.devices) == 1:
                self._proxy = next(iter(c._proxy.devices.values()))
            else:
                raise Exception(
                    "%s: More than one device found on controller, "
                    "so must specify controller.name." % self.name
                )
        self.get_all_settings = self._proxy.get_all_settings
        self.get_setting = self._proxy.get_setting
        self.set_setting = self._proxy.set_setting
        self.describe_setting = self._proxy.describe_setting
        self.describe_settings = self._proxy.describe_settings

    def on_exit(self) -> None:
        if self._proxy is not None:
            self._proxy._pyroRelease()
        super().on_exit()

    def finalize_initialization(self):
        super().finalize_initialization()
        # Set default settings on remote device. These will be over-
        # ridden by any defaults in userConfig, later.
        # Currently, settings are an 'advanced' feature --- the remote
        # interface relies on us to send it valid data, so we have to
        # convert our strings to the appropriate type here.
        ss = self.config.get("settings")
        settings = {}
        if ss:
            settings.update(
                (
                    [
                        m.groups()
                        for kv in ss.split("\n")
                        for m in [re.match(r"(.*?)\s*[:=]\s*(.*?)$", kv)]
                        if m
                    ]
                )
            )
        for k, v in settings.items():
            try:
                desc = self.describe_setting(k)
            except:
                print("%s ignoring unknown setting '%s'." % (self.name, k))
                continue
            if desc["type"] == "str":
                pass
            elif desc["type"] == "int":
                v = int(v)
            elif desc["type"] == "float":
                v = float(v)
            elif desc["type"] == "bool":
                v = v.lower() in ["1", "true"]
            elif desc["type"] == "tuple":
                print(
                    "%s ignoring tuple setting '%s' - not yet supported."
                    % (self.name, k)
                )
                continue
            elif desc["type"] == "enum":
                if v.isdigit():
                    v = int(v)
                else:
                    vmap = dict((k, v) for v, k in desc["values"])
                    v = vmap.get(v, None)
                if v is None:
                    print(
                        "%s ignoring enum setting '%s' with unrecognised value."
                        % (self.name, k)
                    )
                    continue
            self.set_setting(k, v)
        self._read_user_config()

    def get_handlers(self):
        """Return device handlers. Derived classes may override this."""
        result = cockpit.handlers.device.DeviceHandler(
            "%s" % self.name,
            "universal devices",
            False,
            {"make_ui": self.make_ui},
            depot.GENERIC_DEVICE,
        )
        self.handlers = [result]
        return [result]

    def show_settings(self, evt):
        if not self.settings_editor:
            # TODO - there's a problem with abstraction here. The settings
            # editor needs the describe/get/set settings functions from the
            # proxy, but it also needs to be able to invalidate the cache
            # on the handler. The handler should probably expose the
            # settings interface.
            self.set_any_defaults()
            if self.handlers and isinstance(
                self.handlers, collections.abc.Sequence
            ):
                h = self.handlers[0]
            elif self.handlers:
                h = self.handlers
            else:
                h = None
            parent = evt.EventObject.Parent
            self.settings_editor = SettingsEditor(self, parent, handler=h)
            self.settings_editor.Show()
        self.settings_editor.SetPosition(wx.GetMousePosition())
        self.settings_editor.Raise()

    def update_settings(self, settings=None):
        if settings is not None:
            self._proxy.update_settings(settings)
        self.settings.update(self._proxy.get_all_settings())
        events.publish(events.SETTINGS_CHANGED % str(self))

    def set_any_defaults(self):
        # Set any defaults found in userConfig.
        if self.defaults != DEFAULTS_PENDING:
            # nothing to do
            return
        try:
            self._proxy.update_settings(self.settings)
        except Exception as e:
            print(e)
        else:
            self.defaults = DEFAULTS_SENT

    def _read_user_config(self):
        idstr = self.name + "_SETTINGS"
        defaults = cockpit.util.user_config.get_value(idstr)
        if defaults is None:
            self.defaults = DEFAULTS_NONE
            return
        self.settings.update(defaults)
        self.defaults = DEFAULTS_PENDING
        self.set_any_defaults()

    def prepare_for_experiment(self, name, experiment):
        """Make the hardware ready for an experiment."""
        self.cached_settings.update(self.settings)


class MicroscopeGenericDevice(MicroscopeBase):
    def get_handlers(self):
        """Return device handlers."""
        result = cockpit.handlers.device.DeviceHandler(
            "%s" % self.name,
            "universal devices",
            False,
            {"make_ui": self.make_ui},
            depot.GENERIC_DEVICE,
        )
        self.handlers = [result]
        return [result]

    def make_ui(self, parent):
        """UI functions."""
        self.panel = wx.Panel(parent)
        sizer = wx.BoxSizer(wx.VERTICAL)
        adv_button = cockpit.gui.device.Button(
            parent=self.panel, label=self.name, leftAction=self.show_settings
        )
        sizer.Add(adv_button)
        self.panel.SetSizerAndFit(sizer)
        return self.panel


class MicroscopeSwitchableDevice(MicroscopeBase):
    def get_handlers(self):
        """Return device handlers."""
        result = cockpit.handlers.device.DeviceHandler(
            "%s" % self.name,
            "universal devices",
            False,
            {"make_ui": self.make_ui},
            depot.GENERIC_DEVICE,
        )
        self.handlers = [result]
        return [result]

    def make_ui(self, parent):
        """UI functions."""
        self.panel = wx.Panel(parent)
        sizer = wx.BoxSizer(wx.VERTICAL)
        adv_button = cockpit.gui.device.Button(
            parent=self.panel, label=self.name, leftAction=self.show_settings
        )
        sizer.Add(adv_button)
        self.panel.SetSizerAndFit(sizer)
        return self.panel

    def finalize_initialization(self):
        super().finalize_initialization()
        self.enabled = self._proxy.get_is_enabled()


class MicroscopeLaser(MicroscopeBase):
    """A light source with power control.

    Sample config entry:
      [488nm]
      type: MicroscopeLaser
      uri: PYRO:DeepstarLaser@192.168.0.2:7001
      wavelength: 488
      triggerSource: trigsource
      triggerLine: 1

      [trigsource]
      type: ExecutorDevice
      ...
    """

    def _set_enabled(self, on):
        if on:
            self._proxy.enable()
        else:
            self._proxy.disable()

    def get_handlers(self):
        self.handlers.append(
            cockpit.handlers.light_power.LightPowerHandler(
                self.name + " power",  # name
                self.name + " light source",  # group_name
                {
                    "set_power": self._set_power,
                    "get_power": self._get_power,
                },
                self.config.get("wavelength", None),
                current_power=0.2,
                is_enabled=True,
            )
        )
        trigger_source = self.config.get("triggersource", None)
        trigger_line = self.config.get("triggerline", None)
        if trigger_source:
            trigger_handler = depot.get_handler(trigger_source, depot.EXECUTOR)
        else:
            trigger_handler = None
        self._exposure_time = 100
        self.handlers.append(
            cockpit.handlers.light_source.LightHandler(
                self.name,
                self.name + " light source",
                {
                    "set_enabled": lambda name, on: self._set_enabled(on),
                    "set_exposure_time": lambda name, value: setattr(
                        self, "_exposure_time", value
                    ),
                    "get_exposure_time": lambda name: self._exposure_time,
                },
                self.config.get("wavelength", None),
                100,
                trigger_handler,
                trigger_line,
            )
        )

        return self.handlers

    @cockpit.util.threads.call_in_new_thread
    def _set_power(self, power: float) -> None:
        self._proxy.power = power

    def _get_power(self) -> float:
        return self._proxy.power

    def finalize_initialization(self):
        # This should probably work the other way around:
        # after init, the handlers should query for the current state,
        # rather than the device pushing state info to the handlers as
        # we currently do here.
        ph = self.handlers[0]  # power_handler
        ph.power_set_point = self._proxy.get_set_power()
        # Set lightHandler to enabled if light source is on.
        lh = self.handlers[-1]
        lh.state = int(self._proxy.get_is_on())


class MicroscopeFilter(MicroscopeBase):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Cameras
        cdefs = self.config.get("cameras", None)
        if cdefs:
            self.cameras = re.split(r"[,;]\s*", cdefs)
        else:
            self.cameras = None

        # Lights
        ldefs = self.config.get("lights", None)
        if ldefs:
            self.lights = re.split(r"[,;]\s*", ldefs)
        else:
            self.lights = None

        # Filters
        fdefs = self.config.get("filters")
        if fdefs is None:
            raise Exception(
                "Missing 'filters' value for device '%s'" % self.name
            )
        fdefs = [
            re.split(r":\s*|,\s*", f) for f in re.split(r"\n", fdefs) if f
        ]
        self.filters = [
            cockpit.handlers.filter.Filter(*f) for f in fdefs
        ]

    def get_handlers(self):
        """Return device handlers."""
        h = cockpit.handlers.filter.FilterHandler(
            self.name,
            "filters",
            False,
            {
                "set_position": self.set_position,
                "get_position": self.get_position,
                "get_filters": self.get_filters,
            },
            self.cameras,
            self.lights,
        )
        self.handlers = [h]
        return self.handlers

    def set_position(self, position, callback=None):
        asproxy = Pyro4.Proxy(self._proxy._pyroUri)
        asproxy._pyroAsync()
        result = asproxy.set_position(position).then(callback)

    def get_position(self):
        return self._proxy.get_position()

    def get_filters(self):
        return self.filters


class _MicroscopeStageAxis:
    """Wrap a Python microscope StageAxis for a cockpit PositionerHandler.

    Args:
        axis: an instance of `microscope.devices.StageAxis`.
        index: the cockpit axis index value for this axis (0 for X, 1
            for Y, or 2 for Z).
        units_per_micron: the number of units, or steps, used by the
            device per µm.
        stage_name: the name of the stage device, used to construct
            the handler name.
    """

    def __init__(
        self, axis, index: int, units_per_micron: float, stage_name: str
    ) -> None:
        self._axis = axis
        self._units_per_micron = units_per_micron
        self._name = "%d %s" % (index, stage_name)

        limits = AxisLimits(
            self._axis.limits.lower / self._units_per_micron,
            self._axis.limits.upper / self._units_per_micron,
        )

        group_name = "%d stage motion" % index
        eligible_for_experiments = False
        # TODO: to make it eligible for experiments, we need a
        # get_movement_time callback (see issue #614).
        callbacks = {
            "get_movement_time": self.get_movement_time,
            "get_position": self.get_position,
            "move_absolute": self.move_absolute,
            "move_relative": self.move_relative,
        }

        self._handler = PositionerHandler(
            self._name,
            group_name,
            eligible_for_experiments,
            callbacks,
            index,
            limits,
        )

    def get_handler(self) -> PositionerHandler:
        return self._handler

    def get_movement_time(
        self, index: int, start: float, end: float
    ) -> float:
        # TODO: this is not implemented yet but it shouldn't be called
        # anyway because we are not eligible for experiments.
        del index
        raise NotImplementedError("")

    def get_position(self, index: int) -> float:
        """Get the position for the specified axis."""
        del index
        return self._axis.position / self._units_per_micron

    def move_absolute(self, index: int, position: float) -> None:
        """Move axis to the given position in microns."""
        # Currently, the move methods of a Python Microscope stage
        # blocks until the move is done.  When there is an async move
        # stage on Microscope, we don't have to block here and can
        # send STAGE_MOVER events as the move happens and
        # STAGE_STOPPED when it is done (whatever that means).
        self._axis.move_to(position * self._units_per_micron)
        events.publish(events.STAGE_MOVER, index)
        events.publish(events.STAGE_STOPPED, self._name)

    def move_relative(self, index: int, delta: float) -> None:
        """Move the axis by the specified delta, in microns."""
        # See comments on move_absolute about async moves.
        self._axis.move_by(delta * self._units_per_micron)
        events.publish(events.STAGE_MOVER, index)
        events.publish(events.STAGE_STOPPED, self._name)


class MicroscopeStage(MicroscopeBase):
    """Device class for a Python microscope StageDevice.

    This device requires two configurations per axis:

    1. The ``axis-name`` configuration specifies the name of the axis
       on the Python microscope ``StageDevice``.  Usually, this is
       something like ``X`` or ``Y`` but can be any string.  Refer to
       the device documentation.

    2. The ``units-per-micron`` configuration specifies the number of
       units, or steps, used by the device in a µm.  This value is
       used to convert between the device units into physical units.

    For example, a remote XY stage with 0.1µm steps, and a separate Z
    stage with 25nm steps, would have a configuration entry like so::

      [XY stage]
      type: cockpit.devices.microscope_device.MicroscopeStage
      uri: PYRO:SomeXYStage@192.168.0.2:7001
      x-axis-name: X
      y-axis-name: Y
      # Each step is 0.1µm, therefore 10 steps per µm
      x-units-per-micron: 10 # 1 step == 0.1µm
      y-units-per-micron: 10 # 1 step == 0.1µm

      [Z stage]
      type: cockpit.devices.microscope_device.MicroscopeStage
      uri: PYRO:SomeZStage@192.168.0.2:7002
      z-axis-name: Z
      # Each step is 25nm, therefore 40 steps per µm
      x-units-per-micron: 40

    """

    def __init__(self, name: str, config: typing.Mapping[str, str]) -> None:
        super().__init__(name, config)
        self._axes = []  # type: typing.List[_MicroscopeStageAxis]

    def initialize(self) -> None:
        super().initialize()

        if self._proxy.may_move_on_enable():
            # Motors will home during enable.
            title = "stage needs to move"
            msg = (
                "The '%s' stage needs to find the home position."
                " Homing may move it so please ensure that there are"
                " no obstructions, then press 'OK' to home the stage."
                " If you press 'Cancel' the stage will not be homed"
                " and its behaviour will be unpredictable." % (self.name)
            )
            if cockpit.gui.gui_utils.get_user_permission(msg, title):
                self._proxy.enable()
        else:
            self._proxy.enable()

        # The names of the axiss we have already configured, to avoid
        # handling the same one under different names, and to ensure
        # that we have all axis configured.
        handled_axis_names = set()

        their_axes_map = self._proxy.axes

        for one_letter_name in "xyz":
            axis_config_name = one_letter_name + "-axis-name"
            if axis_config_name not in self.config:
                # This stage does not have this axis.
                continue

            their_name = self.config[axis_config_name]
            if their_name not in their_axes_map:
                raise Exception("unknown axis named '%s'" % their_name)

            units_config_name = one_letter_name + "-units-per-micron"
            if units_config_name not in self.config:
                raise Exception(
                    "missing '%s' value in the configuration"
                    % units_config_name
                )
            units_per_micron = float(self.config[units_config_name])
            if units_per_micron <= 0.0:
                raise ValueError(
                    "'%s' configuration must be a positive value"
                    % units_config_name
                )

            their_axis = their_axes_map[their_name]
            cockpit_index = stage_mover.AXIS_MAP[one_letter_name]
            self._axes.append(
                _MicroscopeStageAxis(
                    their_axis, cockpit_index, units_per_micron, self.name
                )
            )
            handled_axis_names.add(their_name)

        # Ensure that there isn't a non handled axis left behind.
        for their_axis_name in their_axes_map.keys():
            if their_axis_name not in handled_axis_names:
                # FIXME: maybe this should be a warning instead?  What
                # if this is a stage with more than XYZ axes and it's
                # not configured simply because cockpit can't handle
                # them?
                raise Exception(
                    "No configuration for the axis named '%s'"
                    % their_axis_name
                )
        # check if we need to poll for updates of position
        # This is usually required as there is a manual joystick which can move
        # the stage without cockpit knowing about it.
        # Note: currently this variable isnt checked, all stages are polled
        # in macrostageXY.
        self.poll_stage = bool(self.config.get("poll-stage", False))
        if self.poll_stage:
            if len(self._axes) == int(self.config.get("num-stage-axes", 0)):
                # need to know in advance how many axis need to initialized
                self.poll_interval = float(self.config.get("poll-interval", 10))
                self._position_cache = [0.0 for x in range(len(self._axes))]

    def get_handlers(self) -> typing.List[PositionerHandler]:
        # Override MicroscopeBase.getHandlers.  Do not call super.
        return [x.get_handler() for x in self._axes]


class MicroscopeDIO(MicroscopeBase):
    """Device class for asynchronous Digital Inout and Output signals.
    This class enables the configuration of named buttons in main GUI window
    to control for situation such a switchable excitation paths.

    Additionally it provides a debug window which allow control of the
    state of all output lines and the direction (input or output) of each
    control line assuming the hardware support this.
    """

    def __init__(self, name: str, config: typing.Mapping[str, str]) -> None:
        super().__init__(name, config)
        self.name = name

    def initialize(self) -> None:
        super().initialize()
        self.num_lines = self._proxy.get_num_lines()
        # cache which we can read from if we dont want a roundtrip
        # to the remote.
        self._cache = [False] * self.num_lines
        self.labels = [""] * self.num_lines
        self.io_map = [None] * self.num_lines

        # read config entries if they exist to
        io_map_config = self.config.get("io_map", [None] * self.num_lines)
        if io_map_config[0] is not None:
            # config is defined so read it into a bool variable,
            # else it is all [Nones]
            io_map = io_map_config.split(",")
            for i, state in enumerate(io_map):
                self.io_map[i] = bool(int(state))
        labels = self.config.get("labels", None)
        paths = self.config.get("paths", None)

        if self.io_map[0] is not None:
            # first entry is not None so map defined
            self._proxy.set_all_IO_state(self.io_map)
        else:
            # no map so set all lines to output
            self._proxy.set_all_IO_state([True] * self.num_lines)
        # start all output lines as false
        for i in range(self.num_lines):
            if self.io_map[i]:
                self.write_line(i, False)

        ##extract names of lines from file, too many are ignored,
        ## too few are padded with str(line number)
        templabels = []
        if labels:
            templabels = eval(labels)
        for i in range(self.num_lines):
            if i < len(templabels):
                self.labels[i] = templabels[i]
            else:
                self.labels[i] = "Line %d" % i
        # extract defined paths
        if paths:
            self.paths = eval(paths)
        else:
            self.paths = {}
        # Lister to receive data back from hardware
        self.listener = cockpit.util.listener.Listener(
            self._proxy, lambda *args: self.receiveData(*args)
        )
        # log to record line state chnages
        self.logger = value_logger.ValueLogger(self.name, keys=self.labels)
        events.subscribe(events.DIO_INPUT, self.log_state_change)
        events.subscribe(events.DIO_OUTPUT, self.log_state_change)

    def read_line(self, line: int, cache=False, update_gui=True) -> int:
        if cache:
            return self._cache[line]
        state = self._proxy.read_line(line)
        if update_gui:
            # prevent a loop by calling this read line in the button
            # toggle code
            events.publish(events.DIO_INPUT, line, state)
        return state

    def read_all_lines(self, cache=False):
        if cache:
            return self._cache
        states = self._proxy.read_all_lines()
        for i in range(len(states)):
            events.publish(events.DIO_INPUT, i, states[i])
        return states

    def write_line(self, line: int, state: bool) -> None:
        self._proxy.write_line(line, state)
        events.publish(events.DIO_OUTPUT, line, state)

    def write_all_lines(self, array):
        self._proxy.write_all_lines(array)
        for i in range(len(array)):
            events.publish(events.DIO_OUTPUT, i, array[i])

    def get_io_state(self, line, cache=False):
        if cache:
            return self.io_map[line]
        state = self._proxy.get_IO_state(line)
        self.io_map[line] = state
        return state

    def set_io_state(self, line, state):
        self.io_map[line] = state
        self._proxy.set_IO_state(line, state)

    def enable(self, state):
        if state:
            self._proxy.enable()
            self.listener.connect()
            return True
        else:
            self._proxy.disable()
            self.listener.disconnect()
            return False

    def log_state_change(self, line, state):
        # log befroe we update cache to get sharp transitions.
        self.logger.log(list(map(int, self._cache)))
        self._cache[line] = state
        # need to map bool's to ints for valuelogviewer
        self.logger.log(list(map(int, self._cache)))

    def show_debug_window(self):
        """Debugging function: display a debug window."""
        self.dio_debug_window = DIOOutputWindow(
            self, parent=wx.GetApp().MainWindow
        ).Show()

    def get_handlers(self):
        """Return device handlers."""
        ##nneds functions to get and set signals for save and load
        ##channel functionality.
        h = cockpit.handlers.digital_io.DigitalIOHandler(
            self.name,
            "DIO",
            False,
            {
                "set_outputs": self.write_all_lines,
                "set_io_state": self.set_io_state,
                "get_outputs": self.read_all_lines,
                "get_io_state": self.get_io_state,
                "get_paths": self.get_paths,
                "write_line": self.write_line,
                "get_labels": self.get_labels,
                "enable": self.enable,
            },
        )
        self.handlers = [h]
        return self.handlers

    def get_labels(self):
        return self.labels

    def get_paths(self):
        return self.paths

    def receiveData(self, *args):
        """This function is called when input line state is received from
        the hardware."""
        (line, state), timestamp = args
        if self.io_map[line]:
            # this is meant to be an output line!
            raise Exception("Input signal received on an output digital line")
        # State changed send event to interested parties.
        events.publish(events.DIO_INPUT, line, state)


## This debugging window lets each digital lineout of the DIO device
## be manipulated individually.


class DIOOutputWindow(wx.Frame):
    def __init__(self, dio, parent, *args, **kwargs):
        super().__init__(parent, *args, **kwargs)

        ## piDevice instance.
        self.dio = dio
        # Contains all widgets.
        panel = wx.Panel(self)
        main_sizer = wx.BoxSizer(wx.VERTICAL)
        toggle_sizer = wx.GridSizer(1, dio.num_lines, 1, 1)
        button_sizer = wx.GridSizer(1, dio.num_lines, 1, 1)

        ## Maps buttons to their lines.
        self.line_to_button = {}
        self.state = self.dio._proxy.read_all_lines()
        # Set up the digital lineout buttons.
        for i in range(dio.num_lines):
            # state of IO , output or Input
            toggle = wx.ToggleButton(panel, wx.ID_ANY)
            toggle.Bind(wx.EVT_TOGGLEBUTTON, lambda evt: self.update_state())
            toggle_sizer.Add(toggle, 1, wx.EXPAND)
            io_state = self.dio.get_io_state(i)
            toggle.SetValue(io_state)
            if io_state:
                toggle.SetLabel("Output")
            else:
                toggle.SetLabel("Input")
            # Button to toggle state of output lines.
            button = wx.ToggleButton(panel, wx.ID_ANY, self.dio.labels[i])
            button.Bind(wx.EVT_TOGGLEBUTTON, lambda evt: self.toggle())
            button_sizer.Add(button, 1, wx.EXPAND)
            self.line_to_button[i] = [toggle, button]
            if self.state[i] is not None:
                button.SetValue(bool(self.state[i]))
            else:
                # if no state reported from remote set to false
                button.SetValue(False)
            if io_state == False:
                # need to do something like colour the button red
                button.Disable()
                button.SetLabel(str(int(self.dio.read_line(i))))
            else:
                button.Enable()

        main_sizer.Add(toggle_sizer)
        main_sizer.Add(button_sizer)
        panel.SetSizerAndFit(main_sizer)
        self.SetClientSize(panel.GetSize())
        events.subscribe(events.DIO_OUTPUT, self.output_changed)
        events.subscribe(events.DIO_INPUT, self.input_changed)

    def output_changed(self, line, state):
        """functions to updated changes and GUI displays when DIO state
        changes."""
        # check this is an output line
        if self.dio.io_map:
            self.line_to_button[line][1].SetValue(state)
            self.update_state(line, bool(state))

    def input_changed(self, line, state):
        self.update_state(line, bool(state))

    def toggle(self):
        """One of our buttons was clicked; update the debug output."""
        for line, (toggle, button) in self.line_to_button.items():
            if self.dio.get_io_state(line):
                self.dio.write_line(line, button.GetValue())
            else:
                # read input state.
                button.SetValue = bool(
                    self.dio.read_line(line, update_gui=False)
                )

    @cockpit.util.threads.call_in_main_thread
    def update_state(self, line=None, state=None):
        """One of our buttons was clicked; update the debug output."""
        if (line is not None) and (state is not None):
            _logger.debug("Line %d returned %s", line, str(state))
            if self.dio.get_io_state(line):
                # output button have names
                self.line_to_button[line][1].SetLabel(self.dio.labels[line])
            else:
                self.line_to_button[line][1].SetLabel(str(int(state)))
            return ()
        for line, (toggle, button) in self.line_to_button.items():
            state = toggle.GetValue()
            self.dio.set_io_state(line, state)
            if state:
                button.Enable()
                toggle.SetLabel("Output")
                button.SetLabel(self.dio.labels[line])
            else:
                button.Disable()
                toggle.SetLabel("Input")
                state = self.dio.read_line(line, update_gui=False)
                button.SetLabel(str(int(state)))


class MicroscopeValueLogger(MicroscopeBase):
    """Device class for asynchronous Digital Input and Output signals.
    This class enables the configuration of named buttons in main GUI window
    to control for situation such a switchable excitation paths.

    Additionally it provides a debug window which allow control of the
    state of all output lines and the direction (input or output) of each
    control line assuming the hardware support this.
    """

    def __init__(self, name: str, config: typing.Mapping[str, str]) -> None:
        super().__init__(name, config)
        self.name = name

    def initialize(self) -> None:
        super().initialize()
        self.num_sensors = self._proxy.get_num_sensors()
        # cache which we can read from if we dont want a roundtrip
        # to the remote.
        self._cache = [False] * self.num_sensors
        self.labels = [""] * self.num_sensors
        labels = self.config.get("labels", None)
        ##do we get data pushed or do we pull it from the remote.
        self.pull_data = self.config.get("pulldata", False)
        self.poll_interval = int(self.config.get("pollinterval", 20))
        ##extract names of lines from file, too many are ignored,
        ## too few are padded with str(line number)
        temp_labels = []
        if labels:
            temp_labels = eval(labels)
        for i in range(self.num_sensors):
            if i < len(temp_labels):
                self.labels[i] = temp_labels[i]
            else:
                self.labels[i] = "Sensor %d" % i
        # Lister to receive data back from hardware
        if not self.pull_data:
            # data is pushed so we need the start a listerner to receive the data
            self.listener = cockpit.util.listener.Listener(
                self._proxy, lambda *args: self.receiveData(*args)
            )
            # log to record line state changes
            self.logger = value_logger.ValueLogger(self.name, keys=self.labels)
        else:
            self.logger = value_logger.PollingLogger(
                self.name,
                self.poll_interval,
                self.get_remote_values,
                keys=self.labels,
            )

        self.enable(True)

    def receiveData(self, *args):
        """This function is called when sensors push data from the remote and
        return data from the hardware."""
        data, timestamp = args
        events.publish(events.VALUELOGGER_INPUT, data)
        self.logger.log(data)

    def get_remote_values(self):
        """This calls the remote getValues() function to
        pull data from the remote hardware."""
        data = self._proxy.getValues()
        events.publish(events.VALUELOGGER_INPUT, data)
        return data

    def enable(self, state):
        if state:
            self._proxy.enable()
            if not self.pull_data:
                self.listener.connect()
            return True
        else:
            self._proxy.disable()
            if not self.pull_data:
                self.listener.disconnect()
            else:
                # Is this a race condition should we stop the event then
                # disable the proxy?
                self.logger.__stopEvent.set()
            return False

    def on_exit(self):
        """ensure we stop the polling if we are shutting down."""
        self.enable(False)
