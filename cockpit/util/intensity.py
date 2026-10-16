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


"""SIM intensity profiling tool.

This can be used on its own from the command line, or can be included
as part of another wx app.
"""

import gc
from contextlib import contextmanager
from itertools import chain
from operator import add

import numpy as np
import wx
import wx.lib.plot as plot
from wx.lib.floatcanvas import FloatCanvas

from cockpit.util.mrc import Mrc


ICON_SIZE = (16, 16)
BITMAP_SIZE = (512, 512)


class IntensityProfiler:
    """A class to profile intensity and store calculation variables."""

    def __init__(self):
        self._data = None
        self._data_source = None
        self._projection = None
        self._bead_centre = None
        self._half_width = 25
        self._phases = 5
        self.results = None

    @contextmanager
    def open_data(self):
        """A context manager to avoid holding files open.

        mrc.bind_file uses numpy mem-mapping. The only way to
        close a mem-mapped file is to delete all references to
        the memmap object, which will then be cleaned up by
        the garbage collector.
        """
        try:
            # Test if this is a reentrant call.
            is_outermost_call = self._data is None
            if is_outermost_call:
                src = self._data_source
                self._data = Mrc(src, "r").data_with_mrc(src)
            yield
        except IOError:
            wx.MessageBox(
                "Could not open data file: it may have been moved or deleted.",
                caption="IO Error",
                style=wx.OK,
                parent=wx.GetTopLevelWindows()[0],
            )
            yield
        finally:
            if is_outermost_call:
                self._data = None
                gc.collect()

    def calculate_intensity(self):
        """Do the calculation."""
        if self._data_source is None:
            return False
        with self.open_data():
            if self._bead_centre is None:
                self.guess_bead_centre()
            n_phases = self._phases
            nz, ny, nx = self._data.shape
            if not self._half_width:
                self.set_half_width(min(nx / 10, ny / 10))
            half_width = self.get_half_width()
            peakx, peaky = self._bead_centre
            # Use a the fifth of the data around the bead, or to edge of dataset.
            data_subset = self._data[
                :,
                max(0, int(peaky - half_width)): min(
                    int(ny), int(peaky + half_width)
                ),
                max(0, int(peakx - half_width)): min(
                    int(nx), int(peakx + half_width)
                ),
            ]
            # Estimate background from image corners.
            bkg = np.min(
                [
                    np.mean(self._data[:, : nx // 10, : ny // 10]),
                    np.mean(self._data[:, : -nx // 10, : ny // 10]),
                    np.mean(self._data[:, : -nx // 10, : -ny // 10]),
                    np.mean(self._data[:, : nx // 10, : -ny // 10]),
                ]
            )
            phase_arr = np.sum(np.sum(data_subset - bkg, axis=2), axis=1)
            phase_arr = np.reshape(phase_arr, (-1, n_phases)).astype(np.float32)
            sep_arr = np.dot(self.sepmatrix(), phase_arr.transpose())
            mag = np.zeros((n_phases // 2 + 1, nz // n_phases)).astype(
                np.float32
            )
            phi = np.zeros((n_phases // 2 + 1, nz // n_phases)).astype(
                np.float32
            )
            mag[0] = sep_arr[0]

            for order in range(1, 3):
                mag[order] = np.sqrt(
                    sep_arr[2 * order - 1] ** 2 + sep_arr[2 * order] ** 2
                )
                phi[order] = np.arctan2(
                    sep_arr[2 * order], sep_arr[2 * order - 1]
                )
            # Average a few points around the peak
            bead_average = np.average(
                np.average(
                    self._data[
                        :,
                        int(peaky - 2) : int(peaky + 2),
                        int(peakx - 2) : int(peakx + 2),
                    ],
                    axis=2,
                ),
                axis=1,
            )
            avg_peak = np.reshape(bead_average, (-1, n_phases))
            avg_peak = np.average(avg_peak, 1)
            avg_peak -= avg_peak.min()
            avg_peak *= mag[1].max() / avg_peak.max()

            peak = np.reshape(
                self._data[:, int(peaky), int(peakx)], (-1, n_phases)
            )
            peak = np.average(peak, 1)
            peak -= peak.min()
            peak *= mag[1].max() / peak.max()

            self.results = dict(
                peak=peak, avg=avg_peak, mag=mag, phi=phi, sep=sep_arr
            )

    def set_data_source(self, filename):
        """Set data source, clearing invalidated variables."""
        self._data_source = filename
        self._projection = None
        self._bead_centre = None
        self._results = None
        with self.open_data():
            self.z_delta = self._data.Mrc.hdr.d[-1]
            self.set_half_width(min(self._data.shape[1:]) / 10)

    def guess_bead_centre(self, refine=True):
        """Estimate the bead centre from position of maximum data value.

        refine =
            True: search around current _bead_centre
            False: search around middle of dataset.
        """
        if self._data_source is None:
            return
        with self.open_data():
            nz, ny, nx = self._data.shape
            if self._bead_centre is None or not refine:
                # Search around centre of dataset.
                middle = self._data[
                    :, 3 * ny // 8 : 5 * ny // 8, 3 * nx // 8 : 5 * nx // 8
                ]
                x_offset = nx // 2 - middle.shape[-1] // 2
                y_offset = ny // 2 - middle.shape[-2] // 2
            else:
                # Search around current _bead_centre.
                n = 24
                x0, y0 = self._bead_centre
                middle = self._data[
                    :, y0 - n // 2 : y0 + n // 2, x0 - n // 2 : x0 + n // 2
                ]
                x_offset = x0 - n // 2
                y_offset = y0 - n // 2
            peak_position = np.argmax(middle)
            z, y, x = np.unravel_index(peak_position, middle.shape)
            self._bead_centre = (x + x_offset, y + y_offset)
            return self._bead_centre

    def get_projection(self):
        """Calculates a Z-projection and returns a copy."""
        if self._projection is None:
            with self.open_data():
                nz = self._data.shape[0]
                dz = min(100, nz // 3)
                subset = self._data[nz // 2 - dz : nz // 2 + dz, :, :].copy()
            # Single step np.mean leaves open refs to self._data, for some reason.
            # self._projection = np.mean(subset, axis=0)
            # Create empty array and use indexed mean to avoid stray refs.
            self._projection = np.zeros(subset.shape[1:3])
            self._projection[:, :] = np.mean(subset, axis=0)
        return self._projection

    def has_data(self):
        """Do I have data?"""
        return not self._data_source is None

    def sepmatrix(self):
        """Return SIM separation matrix.

        Depends on self._phases."""
        nphases = self._phases
        sepmat = np.zeros((nphases, nphases)).astype(np.float32)
        norders = (nphases + 1) // 2
        phi = 2 * np.pi / nphases
        for j in range(nphases):
            sepmat[0, j] = 1.0 / nphases
            for order in range(1, norders):
                sepmat[2 * order - 1, j] = (
                    2.0 * np.cos(j * order * phi) / nphases
                )
                sepmat[2 * order, j] = 2.0 * np.sin(j * order * phi) / nphases
        return sepmat

    def get_bead_centre(self):
        """Return the bead centre co-ordinates."""
        return self._bead_centre

    def set_bead_centre(self, pos):
        """Set the bead centre to a client-provided value."""
        self._bead_centre = pos

    def get_half_width(self):
        """Return the box half width."""
        return self._half_width

    def set_half_width(self, val):
        """Set the box half width."""
        self._half_width = int(val)

    def set_phases(self, n):
        """Set the number of phases to use in sepmatrix."""
        self._phases = n


class IntensityProfilerFrame(wx.Frame):
    """This class provides a UI for IntensityProfiler."""

    SHOW_DEFAULT = False

    def __init__(self, parent=None):
        super().__init__(parent, title="SIM intensity profile")
        self.profiler = IntensityProfiler()
        # Outermost sizer.
        vbox = wx.BoxSizer(wx.VERTICAL)

        ## Toolbar
        toolbar = wx.ToolBar(self, -1)
        # Open file
        open_tool = toolbar.AddTool(
            wx.ID_ANY,
            "Open",
            wx.ArtProvider.GetBitmap(
                wx.ART_FILE_OPEN, wx.ART_TOOLBAR, ICON_SIZE
            ),
            shortHelp="Open a dataset.",
        )
        toolbar.AddSeparator()
        # Number of phases
        phase_label = wx.StaticText(
            toolbar, wx.ID_ANY, label="# phases: ", style=wx.TRANSPARENT_WINDOW
        )
        phase_label.Bind(wx.EVT_ERASE_BACKGROUND, lambda event: None)
        toolbar.AddControl(phase_label)
        phases_tool = wx.SpinCtrl(
            toolbar,
            wx.ID_ANY,
            value="5",
            size=(48, -1),
            min=1,
            max=5,
            initial=5,
            style=wx.SP_ARROW_KEYS | wx.TE_PROCESS_ENTER,
        )
        phases_tool.Bind(
            wx.EVT_SPINCTRL,
            lambda event: self.profiler.set_phases(event.GetInt()),
        )
        phases_tool.Bind(
            wx.EVT_TEXT_ENTER,
            lambda event: self.profiler.set_phases(event.GetInt()),
        )
        toolbar.AddControl(control=phases_tool)
        toolbar.AddSeparator()
        # Box size.
        box_label = wx.StaticText(
            toolbar, wx.ID_ANY, label="box size: ", style=wx.TRANSPARENT_WINDOW
        )
        box_label.Bind(wx.EVT_ERASE_BACKGROUND, lambda event: None)
        toolbar.AddControl(box_label)
        box_tool = wx.SpinCtrl(
            toolbar,
            wx.ID_ANY,
            value="25",
            size=(48, -1),
            min=10,
            max=2**16,
            initial=25,
            style=wx.SP_ARROW_KEYS | wx.TE_PROCESS_ENTER,
        )
        box_tool.Bind(
            wx.EVT_TEXT_ENTER, lambda event: self.set_box_size(event.GetInt())
        )
        box_tool.Bind(
            wx.EVT_SPINCTRL, lambda event: self.set_box_size(event.GetInt())
        )
        toolbar.AddControl(control=box_tool)
        self.box_tool = box_tool
        toolbar.AddSeparator()
        # Calculate profile.
        go_tool = toolbar.AddTool(
            wx.ID_ANY,
            "Go",
            wx.ArtProvider.GetBitmap(wx.ART_TIP, wx.ART_TOOLBAR, ICON_SIZE),
            shortHelp="Evaluate intensity profile",
        )
        toolbar.Realize()
        self.Bind(wx.EVT_TOOL, self.load_file, open_tool)
        self.Bind(wx.EVT_TOOL, self.calculate, go_tool)
        vbox.Add(toolbar, 0, border=5)

        ## Canvases
        hbox = wx.BoxSizer(wx.HORIZONTAL)
        # Image canvas
        self.canvas = FloatCanvas.FloatCanvas(
            self, size=(512, 512), style=wx.WANTS_CHARS
        )

        img = wx.Image(BITMAP_SIZE[0], BITMAP_SIZE[1], clear=True)
        self.bitmap = self.canvas.AddBitmap(img, (0, 0), "cc", False)
        self.circle = self.canvas.AddCircle((0, 0), 10, "#ff0000")
        self.rectangle = self.canvas.AddRectangle((0, 0), (20, 20), "#ff0000")
        self.canvas.Bind(FloatCanvas.EVT_LEFT_UP, self.on_click_canvas)
        hbox.Add(self.canvas)
        self.canvas.Bind(wx.EVT_CHAR, self.on_keys)
        # Plot canvas
        self.plot_canvas = plot.PlotCanvas(self, wx.ID_ANY)
        self.plot_canvas.canvas.Bind(wx.EVT_LEFT_UP, self.on_click_plot_canvas)
        self.plot_canvas.MinSize = (512, 512)
        self.plot_canvas.SetSize((512, 512))
        hbox.Add(self.plot_canvas)

        vbox.Add(hbox)

        ## Status bar.
        self.sb = self.CreateStatusBar()
        self.sb.SetFieldsCount(2)
        self.sb.default_text = (
            "Cursors, PgUp/Dn to set box. "
            "(Shift) space to (global) find bead. "
            "Enter or Return to calculate."
        )
        self.sb.SetStatusText(self.sb.default_text)
        self.SetSizerAndFit(vbox)

        toolbar.SetToolLongHelp(0, self.sb.default_text)

        self.box_tool = box_tool

    def calculate(self, event=None):
        """Calculate the profile."""
        # Check that the profiler has data.
        if not self.profiler.has_data():
            self.sb.SetStatusText("No data loaded.")
        # Do the calculation
        if self.profiler.calculate_intensity() is False:
            return
        ## Generate line graphs
        # Raw intensity at one point in XY.
        peak_y = self.profiler.results["peak"][1:]
        peak_x = np.arange(len(peak_y)) * (self.profiler.z_delta or 1)
        peak = plot.PolyLine(list(zip(peak_x, peak_y)), colour="red")
        # Average intensity over a few XY points around the peak.
        # The raw intensity plot can vary greatly when the z-profile is taken
        # just one pixel away; this average plot can help show if a dip in the
        # raw data is a feature of the bead, or due to noise.
        avg_y = self.profiler.results["avg"][1:]
        avg_x = np.arange(len(avg_y)) * (self.profiler.z_delta or 1)
        avg = plot.PolyLine(list(zip(avg_x, avg_y)), colour="red", style=wx.DOT)
        # First order.
        first_y = self.profiler.results["mag"][1, 1:]
        first_x = np.arange(len(first_y)) * (self.profiler.z_delta or 1)
        first = plot.PolyLine(list(zip(first_x, first_y)), colour="green")
        # Second order.
        second_y = self.profiler.results["mag"][2, 1:]
        second_x = np.arange(len(second_y)) * (self.profiler.z_delta or 1)
        second = plot.PolyLine(list(zip(second_x, second_y)), colour="blue")
        # Add line graphs to a graphics context.
        if self.profiler.z_delta is None:
            x_label = "Z slice"
        else:
            x_label = "Z"
        gc = plot.PlotGraphics(
            [peak, avg, first, second], "Intensity profiles", x_label, "counts"
        )
        # Clear any old graphs.
        self.plot_canvas.Clear()
        # Draw the graphics context.
        self.plot_canvas.Draw(gc)

    def load_file(self, event):
        """Open a data file."""
        ## Display the file chooser.
        top = wx.GetApp().TopWindow
        dlg = wx.FileDialog(
            top, "Open", "", "", "", wx.FD_OPEN | wx.FD_FILE_MUST_EXIST
        )
        if dlg.ShowModal() == wx.ID_OK:
            filename = dlg.GetPath()
            self.sb.SetStatusText(self.sb.default_text)
        else:
            self.sb.SetStatusText("Cancelled - no data loaded.")
            return
        # Set the profiler data source .
        self.profiler.set_data_source(filename)
        self.box_tool.Value = self.profiler.get_half_width()

        # Guess a bead position
        xpos, ypos = self.profiler.guess_bead_centre()
        # Move indicator circle to the bead position.
        self.circle.SetPoint(self.canvas.PixelToWorld((xpos, ypos)))

        # Update the bitmap.
        proj = self.profiler.get_projection()
        proj -= np.min(proj)
        proj /= np.max(proj)
        proj = (proj * 255).astype(np.uint8)
        img = np.dstack((proj, proj, proj))
        nx, ny = proj.shape
        self.bitmap.Bitmap.SetSize((nx, ny))
        self.bitmap.Bitmap.CopyFromBuffer(img.tobytes())

        # Update the canvas.
        self.plot_canvas.Clear()
        self.update_canvas()

    def on_click_plot_canvas(self, event):
        """Show the mouse graph-space coords in status bar."""
        pos = event.GetPosition()
        uv = self.plot_canvas.PositionScreenToUser(pos)
        # self.sb.SetStatusText('u:%3f, v:%3f' % uv, 1)
        self.sb.SetStatusText(
            "Z: {0[0]:3.3g},  counts: {0[1]:3.3g}".format(uv), 1
        )

    def on_click_canvas(self, event):
        """Respond to click to choose a new bead position."""
        # Position in pixels from upper left corner.
        pos = event.GetPosition()
        # Update profiler bead centre.
        self.profiler.set_bead_centre(pos)
        # Update the canvas.
        self.update_canvas()

    def update_canvas(self):
        pos = self.profiler.get_bead_centre()
        w = self.profiler.get_half_width()
        # Translate from pixel to world co-ords.
        centre = self.canvas.PixelToWorld(pos)
        wh = self.canvas.ScalePixelToWorld((w, w))
        xy = centre - wh
        # Redraw circle to mark bead.
        self.circle.SetPoint(centre)
        # Redraw the rectangle to mark data subset.
        self.rectangle.SetShape(xy, 2 * wh)
        # Redraw.
        self.canvas.Draw(Force=True)

    def set_box_size(self, value):
        self.profiler.set_half_width(value)
        # Update the canvas.
        self.update_canvas()

    def on_keys(self, event):
        keys = {
            "move": [
                wx.WXK_LEFT,
                wx.WXK_RIGHT,
                wx.WXK_UP,
                wx.WXK_DOWN,
            ],
            "size": [wx.WXK_PAGEDOWN, wx.WXK_PAGEUP],
            "calc": [wx.WXK_RETURN, wx.WXK_NUMPAD_ENTER],
            "find": [wx.WXK_SPACE, wx.WXK_NUMPAD0, wx.WXK_NUMPAD_DECIMAL],
        }

        keycode = event.KeyCode
        keymod = event.GetModifiers()
        if keycode not in [c for c in chain.from_iterable(keys.values())]:
            return
        elif keycode in keys["move"]:
            delta = [1, 6][keymod & wx.MOD_SHIFT > 0]
            # Map keys to position changes.
            d_pos = {
                wx.WXK_LEFT: (-delta, 0),
                wx.WXK_RIGHT: (delta, 0),
                wx.WXK_DOWN: (0, delta),
                wx.WXK_UP: (0, -delta),
            }

            pos = self.profiler.get_bead_centre()
            new_pos = list(map(add, pos, d_pos[keycode]))

            self.profiler.set_bead_centre(new_pos)
            self.update_canvas()
        elif keycode in keys["size"]:
            if keycode == wx.WXK_PAGEUP:
                delta = [1, 6][keymod & wx.MOD_SHIFT > 0]
            else:
                delta = [-1, -6][keymod & wx.MOD_SHIFT > 0]
            # Update spin control.
            self.box_tool.SetValue(self.box_tool.GetValue() + delta)
            # Set the box size. Spin control may limit value so use GetValue.
            self.set_box_size(self.box_tool.GetValue())
        elif keycode in keys["calc"]:
            self.calculate()
        elif keycode in keys["find"]:
            pos = self.profiler.guess_bead_centre(
                refine=keymod & wx.MOD_SHIFT == 0
            )
            self.update_canvas()


def main():
    """Run as a standalone app."""
    import wx.lib.inspection

    app = wx.App(False)
    ip = IntensityProfilerFrame()
    ip.Show()
    wx.lib.inspection.InspectionTool().Show()
    app.MainLoop()


def make_window(parent):
    """Call from another app to get a single window instance."""
    window = IntensityProfilerFrame(parent)


if __name__ == "__main__":
    main()
