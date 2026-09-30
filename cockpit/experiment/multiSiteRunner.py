#!/usr/bin/env python
# -*- coding: utf-8 -*-

## Copyright (C) 2026 Centre National de la Recherche Scientifique (CNRS)
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

"""Run a multi-site experiment.

A `MultiSiteRunner` executes a
`cockpit.experiment.experimentSpecs.MultiSiteSpec`: it visits the sites
cycle after cycle and, at each site, builds the spec's `ExperimentSpec`
into a fresh `cockpit.experiment.experiment.Experiment` (so that Z stacks
are placed relative to that site and the filename gets the cycle and site
numbers), runs it and waits for it to finish.

This module only holds the execution logic; see
`cockpit.experiment.experimentSpecs` for how to describe the experiment.
Usually a runner is created with `MultiSiteSpec.run()`.
"""

import dataclasses
import logging
import threading
import time

import cockpit.interfaces.stageMover
from cockpit import depot, events


_logger = logging.getLogger(__name__)


def siteVisitOrder(sites, frequencies=None, optimize=True):
    """Work out which sites to visit on each cycle.

    Site `sites[i]` is visited every `frequencies[i]`-th cycle. Returns a
    list of site lists; cycle `n` visits `result[n % len(result)]`.
    """
    if frequencies is None:
        frequencies = [1] * len(sites)
    # Number of unique sets of sites to visit. This results in some
    # redundancies, but that's not a huge deal.
    cycleRate = 1
    for frequency in set(frequencies):
        cycleRate *= frequency
    cycleNumToSitesList = []
    for i in range(cycleRate):
        sitesList = [
            siteId
            for siteId, frequency in zip(sites, frequencies)
            if i % frequency == 0
        ]
        if optimize:
            sitesList = cockpit.interfaces.stageMover.optimisedSiteOrder(
                sitesList
            )
        cycleNumToSitesList.append(sitesList)
    return cycleNumToSitesList


## Return exposureSettings without the lights that should not be used on
# the given cycle. Exposures left with no light at all are dropped.
def filterExposureSettings(exposureSettings, lightFrequencies, cycleNum):
    def isActive(light):
        if light not in lightFrequencies:
            return True
        frequency, offset = lightFrequencies[light]
        return cycleNum % frequency == offset

    result = []
    for cameras, lightTimePairs in exposureSettings:
        activePairs = [(l, t) for l, t in lightTimePairs if isActive(l)]
        if lightTimePairs and not activePairs:
            continue
        result.append((cameras, activePairs))
    return result


## Runs a MultiSiteSpec. Single-use, like Experiment.
class MultiSiteRunner:
    def __init__(self, spec):
        ## The MultiSiteSpec being run.
        self.spec = spec
        self.shouldAbort = False
        self.confirm = None
        ## Experiment currently being run, for debugging.
        self.currentExperiment = None
        self._thread = None

    ## Start the experiment in a background thread and return immediately.
    # \param confirm See cockpit.experiment.experiment.Experiment.run. It is
    #        called from the background thread.
    def run(self, confirm=None):
        if self._thread is not None:
            raise RuntimeError("This multi-site experiment was already run.")
        self.spec.sanityCheck()
        self.confirm = confirm
        self.shouldAbort = False
        self._thread = threading.Thread(
            target=self.execute, name="MultiSiteRunner", daemon=True
        )
        self._thread.start()

    ## Block until the experiment has finished. Return True if it finished,
    # False on timeout.
    def wait(self, timeout=None):
        if self._thread is None:
            return True
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    ## User clicked the abort button.
    def onAbort(self, *args):
        self.shouldAbort = True

    ## Run all cycles. Runs in the thread started by run().
    def execute(self):
        events.subscribe(events.USER_ABORT, self.onAbort)
        try:
            self._execute()
        except Exception:
            _logger.exception("Multi-site experiment failed")
        finally:
            events.unsubscribe(events.USER_ABORT, self.onAbort)
            self.cleanup()

    def _execute(self):
        self.waitFor(self.spec.delayBeforeStarting)
        if self.shouldAbort:
            return

        experimentStart = time.localtime()
        cycleNumToSitesList = siteVisitOrder(
            self.spec.sites, self.spec.frequencies, self.spec.optimizeOrder
        )
        cycleStartTime = time.time()
        for cycleNum in range(self.spec.numCycles):
            siteIds = cycleNumToSitesList[cycleNum % len(cycleNumToSitesList)]
            if cycleNum != 0:
                cycleDuration = self.spec.cycleDurations[
                    (cycleNum - 1) % len(self.spec.cycleDurations)
                ]
                # Move to the first site.
                if siteIds:
                    cockpit.interfaces.stageMover.waitForStop()
                    cockpit.interfaces.stageMover.goToSite(
                        siteIds[0], shouldBlock=True
                    )
                # Wait for when the next cycle should start.
                waitTime = cycleStartTime + cycleDuration - time.time()
                if not self.waitFor(waitTime) and cycleDuration > 0:
                    _logger.warning(
                        "Couldn't finish cycle in time; off by %.2f seconds",
                        -waitTime,
                    )
            if self.shouldAbort:
                break
            _logger.info(
                "Starting cycle %d of %d", cycleNum + 1, self.spec.numCycles
            )
            cycleStartTime = time.time()
            for siteId in siteIds:
                if self.shouldAbort:
                    break
                self.imageSite(siteId, cycleNum, experimentStart)
            if self.shouldAbort:
                break

    ## Go to the specified site and run our experiment on it.
    def imageSite(self, siteId, cycleNum, experimentStart):
        spec = self.spec.siteExperiment
        if self.spec.lightFrequencies:
            exposureSettings = filterExposureSettings(
                spec.exposureSettings, self.spec.lightFrequencies, cycleNum
            )
            if not exposureSettings:
                _logger.info(
                    "No lights in use on cycle %d; skipping site %s",
                    cycleNum,
                    siteId,
                )
                return
            spec = dataclasses.replace(spec, exposureSettings=exposureSettings)

        events.publish(
            events.UPDATE_STATUS_LIGHT,
            "device waiting",
            "Waiting for stage motion",
        )
        cockpit.interfaces.stageMover.waitForStop()
        cockpit.interfaces.stageMover.goToSite(siteId, shouldBlock=True)
        self.waitFor(self.spec.delayBeforeImaging)
        if self.shouldAbort:
            return

        # Zero-pad the site ID, when it is a number, so that filenames
        # sort properly.
        try:
            siteName = "%03d" % int(siteId)
        except ValueError:
            siteName = str(siteId)
        mappings = {
            "date": time.strftime("%Y%m%d", experimentStart),
            "time": time.strftime("%H%M", experimentStart),
            "cycle": "%03d" % cycleNum,
            "site": siteName,
        }
        _logger.info("Imaging site %s", siteId)
        start = time.time()
        self.currentExperiment = spec.run(self.confirm, mappings)
        if self.currentExperiment is None:
            _logger.warning(
                "Experiment at site %s, cycle %d was not started",
                siteId,
                cycleNum,
            )
            return
        self.currentExperiment.wait()
        _logger.info("Imaging took %.2f seconds", time.time() - start)

    ## Clean up after the experiment ends.
    def cleanup(self):
        if self.spec.powerDownWhenDone:
            for handler in depot.getHandlersOfType(depot.POWER_CONTROL):
                handler.disable()
        events.publish(events.UPDATE_STATUS_LIGHT, "device waiting", "")

    ## Wait for some time, allowing the user to abort the wait. Return True
    # if we were successful (i.e. handed a valid amount of time to wait for).
    def waitFor(self, seconds):
        if seconds <= 0:
            return False
        _logger.info("Waiting for %.2f seconds", seconds)
        endTime = time.time() + seconds
        curTime = time.time()
        while curTime < endTime and not self.shouldAbort:
            if int(curTime + 0.25) != int(curTime):
                # Advanced to a new second; update the status light.
                remaining = endTime - curTime
                displayMinutes = remaining // 60
                displaySeconds = (remaining - displayMinutes * 60) // 1
                events.publish(
                    events.UPDATE_STATUS_LIGHT,
                    "device waiting",
                    (
                        "Waiting for %02d:%02d"
                        % (displayMinutes, displaySeconds)
                    ),
                )
            time.sleep(0.25)
            curTime = time.time()
        return True
