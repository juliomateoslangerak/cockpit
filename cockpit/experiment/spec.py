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
        zStack.ZStackExperiment,
        exposureSettings=[([camera], [(light, Decimal(50))])],
        zMode=spec.ZMode.CENTER,
        zHeight=5,
        sliceHeight=0.5,
        saveDir="/data/run1",
        filenameTemplate="{date}-{time}.mrc",
    )
    s.run(confirm=lambda message: True)
"""

import dataclasses
import enum
import os.path
import time
import typing

import cockpit.interfaces.stageMover
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
