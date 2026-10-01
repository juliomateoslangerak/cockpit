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

"""Widget-free descriptions of experiments.

Experiments are split into what to do and how to do it:

- The specs in this module describe an experiment. They are reusable and
  hold values that are only resolved when the experiment runs, such as a Z
  stack relative to the current stage position, or a filename template.
  Define them from a script or, as the GUI dialogs do, from widgets.

  - `ExperimentSpec` describes one acquisition. `build()` creates the
    `cockpit.experiment.experiment.Experiment` for the current stage
    position.
  - `MultiSiteSpec` describes an `ExperimentSpec` repeated over stage
    sites and cycles.

- The runners do the work and are single-use:

  - `cockpit.experiment.experiment.Experiment` (and its subclasses, e.g.
    `zStack.ZStackExperiment`) runs one acquisition at one position.
  - `cockpit.experiment.multiSiteRunner.MultiSiteRunner` visits the sites
    of a `MultiSiteSpec`, building and running an `Experiment` at each.

Both specs have a `run(confirm)` shortcut which starts its runner and
returns it, so that callers can `wait()` for it to finish. `confirm` is
called with a message whenever the user would be asked to confirm
something, e.g. overwriting a file; see `Experiment.run`.

Example, from the cockpit shell::

    from decimal import Decimal
    from cockpit.experiment import experimentSpecs as specs, zStack
    from cockpit.interfaces import stageMover

    siteExperiment = specs.ExperimentSpec(
        zStack.ZStackExperiment,
        exposureSettings=[([camera], [(light, Decimal(50))])],
        zMode=specs.ZMode.CENTER,
        zHeight=5,
        sliceHeight=0.5,
        saveDir="/data/run1",
        filenameTemplate="{date}-{time}_t{cycle}_p{site}.mrc",
    )

    # A single acquisition here.
    siteExperiment.run(confirm=lambda message: True).wait()

    # The same acquisition at every G2/M site, every 10 minutes.
    runner = specs.MultiSiteSpec(
        siteExperiment,
        sites=stageMover.sitesInGroup("G2/M"),
        numCycles=10,
        cycleDurations=[600],
    ).run(confirm=lambda message: True)
    runner.wait()
"""

import dataclasses
import enum
import os.path
import time
import typing

import cockpit.interfaces.stageMover
from cockpit import depot
from cockpit.experiment import multiSiteRunner


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


def expandFilename(
    template: str, mappings: typing.Optional[typing.Mapping[str, str]] = None
) -> str:
    """Fill in a filename template.

    `{date}` and `{time}` default to the current date and time; other
    placeholders are taken from `mappings`.  Unknown placeholders are left
    as they are.
    """
    allMappings = {
        "date": time.strftime("%Y%m%d"),
        "time": time.strftime("%H%M%S"),
        **(mappings or {}),
    }
    return template.format_map(_KeepMissing(allMappings))


@dataclasses.dataclass
class ExperimentSpec:
    ## Experiment subclass to instantiate, e.g. zStack.ZStackExperiment.
    experimentClass: type
    ## List of ([cameras], [(light, exposure time)]) tuples, as expected by
    # Experiment.
    exposureSettings: list
    numReps: int = 1
    ## Seconds per repetition, or 0 to go as fast as possible.
    repDuration: float = 0.0
    zMode: ZMode = ZMode.BOTTOM
    ## Height of the stack. Ignored for ZMode.SAVED. 0 means 2D.
    zHeight: float = 0.0
    sliceHeight: float = 0.0
    ## Directory to save to. Nothing is saved if filenameTemplate is empty.
    saveDir: str = ""
    filenameTemplate: str = ""
    ## Extra keyword arguments for experimentClass, e.g. from the
    # experiment-specific panels' augmentParams.
    extraParams: dict = dataclasses.field(default_factory=dict)
    ## Z positioner handler. None uses the one with the smallest range of
    # motion, which is what the GUI does.
    zPositioner: typing.Any = None

    def getSavePath(self, mappings=None) -> str:
        if not self.filenameTemplate:
            return ""
        return os.path.join(
            self.saveDir, expandFilename(self.filenameTemplate, mappings)
        )

    ## Return (altBottom, zHeight, sliceHeight) for the current stage
    # position.
    def resolveZ(self):
        zHeight = self.zHeight
        sliceHeight = self.sliceHeight
        if self.zMode is ZMode.SAVED:
            mover = cockpit.interfaces.stageMover.mover
            altBottom = mover.SavedBottom
            zHeight = mover.SavedTop - altBottom
        else:
            altBottom = cockpit.interfaces.stageMover.getPositionForAxis(2)
            if self.zMode is ZMode.CENTER:
                altBottom -= zHeight / 2
        if zHeight == 0:
            # 2D mode.
            zHeight = 1e-6
            sliceHeight = 1e-6
        return altBottom, zHeight, sliceHeight

    ## Create the Experiment for the current stage position.
    # \param mappings Values for the filename template placeholders.
    def build(self, mappings=None):
        zPositioner = self.zPositioner
        if zPositioner is None:
            zPositioner = depot.getSortedStageMovers()[2][-1]
        altBottom, zHeight, sliceHeight = self.resolveZ()
        params = {
            "numReps": self.numReps,
            "repDuration": self.repDuration,
            "zPositioner": zPositioner,
            "altBottom": altBottom,
            "zHeight": zHeight,
            "sliceHeight": sliceHeight,
            "exposureSettings": self.exposureSettings,
            "savePath": self.getSavePath(mappings),
            **self.extraParams,
        }
        return self.experimentClass(**params)

    ## Build and run the experiment. Return the Experiment if it was
    # started, None otherwise. See Experiment.run for `confirm`.
    def run(self, confirm=None, mappings=None):
        experiment = self.build(mappings)
        if experiment.run(confirm):
            return experiment
        return None


def parseLightFrequency(text: str) -> typing.Tuple[int, int]:
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
    siteExperiment: ExperimentSpec
    ## Site IDs, see cockpit.interfaces.stageMover.
    sites: typing.List[int]
    ## Visit sites[i] every frequencies[i]-th cycle. None means every cycle.
    frequencies: typing.Optional[typing.List[int]] = None
    numCycles: int = 1
    ## Minimum time, in seconds, between the start of consecutive cycles.
    # Used in sequence: [60, 120] makes cycles alternate between one and
    # two minutes.
    cycleDurations: typing.Sequence[float] = (0,)
    ## Seconds to wait before the first cycle.
    delayBeforeStarting: float = 0
    ## Seconds to wait after moving to a site before imaging it.
    delayBeforeImaging: float = 0
    ## Reorder the sites to minimise travel time.
    optimizeOrder: bool = True
    ## Maps light handlers to (frequency, offset), see parseLightFrequency.
    # Lights not in the mapping are used on every cycle.
    lightFrequencies: typing.Optional[dict] = None
    ## Power off all POWER_CONTROL devices at the end.
    powerDownWhenDone: bool = False

    ## Raise ValueError if the experiment cannot be run.
    def sanityCheck(self):
        if not self.sites:
            raise ValueError("No sites selected.")
        if self.frequencies is not None:
            if len(self.frequencies) != len(self.sites):
                raise ValueError("Need one frequency per site.")
            if any(f < 1 for f in self.frequencies):
                raise ValueError("Site frequencies must be at least 1.")
        if self.numCycles < 1:
            raise ValueError("Need at least one cycle.")
        if not self.cycleDurations:
            raise ValueError("Need at least one cycle duration.")
        # Verify that all sites are reachable; the user may have restarted
        # cockpit (thus resetting motion safeties) and then loaded a list
        # of sites which we cannot now reach.
        for siteId in self.sites:
            if not cockpit.interfaces.stageMover.doesSiteExist(siteId):
                raise ValueError("Site %s does not exist." % siteId)
            if not cockpit.interfaces.stageMover.canReachSite(siteId):
                raise ValueError("Site %s cannot be reached." % siteId)

    ## Start a MultiSiteRunner in the background and return it.
    # \param confirm See cockpit.experiment.experiment.Experiment.run. It is
    #        called from the runner's thread.
    def run(self, confirm=None):
        runner = multiSiteRunner.MultiSiteRunner(self)
        runner.run(confirm)
        return runner
