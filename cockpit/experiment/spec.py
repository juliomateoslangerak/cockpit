#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2026 Centre National de la Recherche Scientifique (CNRS)
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

"""Widget-free description of an experiment.

An `ExperimentSpec` holds everything needed to create an `Experiment`,
except what can only be known at the moment it is run: the current stage
position (for Z stacks relative to it) and values used to fill in the
filename template (date, time, cycle, site, ...).  This lets experiments
be defined once, from the GUI or from a script, and then built and run
any number of times, e.g. at every site of a multi-site experiment.

Example, from the cockpit shell::

    from decimal import Decimal
    from cockpit.experiment import spec, zStack

    s = spec.ExperimentSpec(
        z-stack.ZStackExperiment,
        exposure_settings=[([camera], [(light, Decimal(50))])],
        z_mode=spec.ZMode.CENTER,
        z_height=5,
        slice_height=0.5,
        save_dir="/data/run1",
        filename_template="{date}-{time}.mrc",
    )
    s.run(confirm=lambda message: True)
"""

import dataclasses
import enum
import os.path
import time
import typing

import cockpit.interfaces.stage_mover
from cockpit import depot


class ZMode(enum.Enum):
    """How the Z range of the stack is placed relative to the stage."""

    ## Values match the labels used in the experiment configuration panel.
    CENTER = "Current is center"
    BOTTOM = "Current is bottom"
    SAVED = "Use saved top/bottom"


class _KeepMissing(dict):
    # Leave unknown placeholders untouched instead of raising KeyError.
    def __missing__(self, key):
        return f"{{{key}}}"


def expand_filename(
    template: str, mappings: typing.Optional[typing.Mapping[str, str]] = None
) -> str:
    """Fill in a filename template.

    `{date}` and `{time}` default to the current date and time; other
    placeholders are taken from `mappings`.  Unknown placeholders are left
    as they are.
    """
    all_mappings = {
        "date": time.strftime("%Y%m%d"),
        "time": time.strftime("%H%M%S"),
        **(mappings or {}),
    }
    return template.format_map(_KeepMissing(all_mappings))


@dataclasses.dataclass
class ExperimentSpec:
    """Experiment subclass to instantiate, e.g. zStack.ZStackExperiment.
    """
    experiment_class: type
    ## List of ([cameras], [(light, exposure time)]) tuples, as expected by
    # Experiment.
    exposure_settings: list
    num_reps: int = 1
    ## Seconds per repetition, or 0 to go as fast as possible.
    rep_duration: float = 0.0
    z_mode: ZMode = ZMode.BOTTOM
    ## Height of the stack. Ignored for ZMode.SAVED. 0 means 2D.
    z_height: float = 0.0
    slice_height: float = 0.0
    ## Directory to save to. Nothing is saved if filename_template is empty.
    save_dir: str = ""
    filename_template: str = ""
    ## Extra keyword arguments for experiment_class, e.g. from the
    # experiment-specific panels' augmentParams.
    extra_params: dict = dataclasses.field(default_factory=dict)
    ## Z positioner handler. None uses the one with the smallest range of
    # motion, which is what the GUI does.
    z_positioner: typing.Any = None

    def get_save_path(self, mappings=None) -> str:
        if not self.filename_template:
            return ""
        return os.path.join(
            self.save_dir, expand_filename(self.filename_template, mappings)
        )

    ## Return (alt_bottom, z_height, slice_height) for the current stage
    # position.
    def resolve_z(self):
        z_height = self.z_height
        slice_height = self.slice_height
        if self.z_mode is ZMode.SAVED:
            mover = cockpit.interfaces.stage_mover.mover
            alt_bottom = mover.SavedBottom
            z_height = mover.SavedTop - alt_bottom
        else:
            alt_bottom = cockpit.interfaces.stage_mover.get_position_for_axis(2)
            if self.z_mode is ZMode.CENTER:
                alt_bottom -= z_height / 2
        if z_height == 0:
            # 2D mode.
            z_height = 1e-6
            slice_height = 1e-6
        return alt_bottom, z_height, slice_height

    ## Create the Experiment for the current stage position.
    # \param mappings Values for the filename template placeholders.
    def build(self, mappings=None):
        z_positioner = self.z_positioner
        if z_positioner is None:
            z_positioner = depot.get_sorted_stage_movers()[2][-1]
        alt_bottom, z_height, slice_height = self.resolve_z()
        params = {
            "num_reps": self.num_reps,
            "rep_duration": self.rep_duration,
            "z_positioner": z_positioner,
            "alt_bottom": alt_bottom,
            "z_height": z_height,
            "slice_height": slice_height,
            "exposure_settings": self.exposure_settings,
            "save_path": self.get_save_path(mappings),
            **self.extra_params,
        }
        return self.experiment_class(**params)

    ## Build and run the experiment. Return the Experiment if it was
    # started, None otherwise. See Experiment.run for `confirm`.
    def run(self, confirm=None, mappings=None):
        experiment = self.build(mappings)
        if experiment.run(confirm):
            return experiment
        return None
