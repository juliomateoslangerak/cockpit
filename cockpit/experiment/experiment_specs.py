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
    from cockpit.experiment import experiment_specs as specs, z_stack
    from cockpit.interfaces import stage_mover

    s = spec.ExperimentSpec(
        z-stack.ZStackExperiment,
        exposure_settings=[([camera], [(light, Decimal(50))])],
        z_mode=spec.ZMode.CENTER,
        z_height=5,
        slice_height=0.5,
        save_dir="/data/run1",
        filename_template="{date}-{time}.dv",
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
from cockpit.experiment import multi_site_runner


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


def parse_light_frequency(text: str) -> typing.Tuple[int, int]:
    """Parse "frequency" or "frequency + offset" into (frequency, offset).

    A light with frequency 5 is used on cycles 0, 5, 10, ...; with
    "5 + 1" it is used on cycles 1, 6, 11, ...
    """
    if "+" in text:
        frequency, offset = [int(s) for s in text.split("+")]
    else:
        frequency, offset = int(text), 0
    if frequency < 1 or not 0 <= offset < frequency:
        raise ValueError(
            "Invalid light frequency '%s': need frequency >= 1 and"
            " 0 <= offset < frequency" % text
        )
    return frequency, offset


@dataclasses.dataclass
class MultiSiteSpec:
    ## Experiment to run at each site.
    site_experiment: ExperimentSpec
    ## Site IDs, see cockpit.interfaces.stageMover.
    sites: typing.List[int]
    ## Visit sites[i] every frequencies[i]-th cycle. None means every cycle.
    frequencies: typing.Optional[typing.List[int]] = None
    num_cycles: int = 1
    ## Minimum time, in seconds, between the start of consecutive cycles.
    # Used in sequence: [60, 120] makes cycles alternate between one and
    # two minutes.
    cycle_durations: typing.Sequence[float] = (0,)
    ## Seconds to wait before the first cycle.
    delay_before_starting: float = 0
    ## Seconds to wait after moving to a site before imaging it.
    delay_before_imaging: float = 0
    ## Reorder the sites to minimise travel time.
    optimize_order: bool = True
    ## Maps light handlers to (frequency, offset), see parseLightFrequency.
    # Lights not in the mapping are used on every cycle.
    light_frequencies: typing.Optional[dict] = None
    ## Power off all POWER_CONTROL devices at the end.
    power_down_when_done: bool = False

    ## Raise ValueError if the experiment cannot be run.
    def sanity_check(self):
        if not self.sites:
            raise ValueError("No sites selected.")
        if self.frequencies is not None:
            if len(self.frequencies) != len(self.sites):
                raise ValueError("Need one frequency per site.")
            if any(f < 1 for f in self.frequencies):
                raise ValueError("Site frequencies must be at least 1.")
        if self.num_cycles < 1:
            raise ValueError("Need at least one cycle.")
        if not self.cycle_durations:
            raise ValueError("Need at least one cycle duration.")
        # Verify that all sites are reachable; the user may have restarted
        # cockpit (thus resetting motion safeties) and then loaded a list
        # of sites which we cannot now reach.
        for site_id in self.sites:
            if not cockpit.interfaces.stage_mover.does_site_exist(site_id):
                raise ValueError("Site %s does not exist." % site_id)
            if not cockpit.interfaces.stage_mover.can_reach_site(site_id):
                raise ValueError("Site %s cannot be reached." % site_id)

    ## Start a MultiSiteRunner in the background and return it.
    # \param confirm See cockpit.experiment.experiment.Experiment.run. It is
    #        called from the runner's thread.
    def run(self, confirm=None):
        runner = multi_site_runner.MultiSiteRunner(self)
        runner.run(confirm)
        return runner
