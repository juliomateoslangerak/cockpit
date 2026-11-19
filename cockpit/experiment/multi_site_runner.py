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

import cockpit.interfaces.stage_mover
from cockpit import depot, events


_logger = logging.getLogger(__name__)


def site_visit_order(sites, frequencies=None, optimize=True):
    """Work out which sites to visit on each cycle.

    Site `sites[i]` is visited every `frequencies[i]`-th cycle. Returns a
    list of site lists; cycle `n` visits `result[n % len(result)]`.
    """
    if frequencies is None:
        frequencies = [1] * len(sites)
    # Number of unique sets of sites to visit. This results in some
    # redundancies, but that's not a huge deal.
    cycle_rate = 1
    for frequency in set(frequencies):
        cycle_rate *= frequency
    cycle_num_to_sites_list = []
    for i in range(cycle_rate):
        sites_list = [
            site_id
            for site_id, frequency in zip(sites, frequencies)
            if i % frequency == 0
        ]
        if optimize:
            sites_list = cockpit.interfaces.stage_mover.optimised_site_order(
                sites_list
            )
        cycle_num_to_sites_list.append(sites_list)
    return cycle_num_to_sites_list


def filter_exposure_settings(exposure_settings, light_frequencies_offsets, cycle_num):
    """Return exposure_settings without the lights that should not be used on
    the given cycle. Exposures left with no light at all are dropped.
    """
    def is_active(light):
        if light not in light_frequencies_offsets:
            return True
        frequency, offset = light_frequencies_offsets[light]
        return cycle_num % frequency == offset

    result = []
    for cameras, light_time_pairs in exposure_settings:
        active_pairs = [(l, t) for l, t in light_time_pairs if is_active(l)]
        if light_time_pairs and not active_pairs:
            continue
        result.append((cameras, active_pairs))
    return result


class MultiSiteRunner:
    """Runs a MultiSiteSpec. Single-use, like Experiment.
    """
    def __init__(self, multisite_spec):
        ## The MultiSiteSpec being run.
        self.multisite_spec = multisite_spec
        self.should_abort = False
        ## Experiment currently being run, for debugging.
        self.current_experiment = None
        self._thread = None

    def run(self):
        """
        Start the experiment in a background thread and return immediately.
        """
        if self._thread is not None:
            raise RuntimeError("This multi-site experiment was already run.")
        self.multisite_spec.sanity_check()
        self.should_abort = False
        self._thread = threading.Thread(
            target=self.execute, name="MultiSiteRunner", daemon=True
        )
        self._thread.start()

    def wait(self, timeout=None):
        """Block until the experiment has finished. Return True if it finished,
        False on timeout.
        """
        if self._thread is None:
            return True
        self._thread.join(timeout)
        return not self._thread.is_alive()

    def is_running(self):
        return self._thread is not None and self._thread.is_alive()

    ## User clicked the abort button.
    def on_abort(self, *args):
        self.should_abort = True

    ## Run all cycles. Runs in the thread started by run().
    def execute(self):
        events.subscribe(events.USER_ABORT, self.on_abort)
        try:
            self._execute()
        except Exception:
            _logger.exception("Multi-site experiment failed")
        finally:
            events.unsubscribe(events.USER_ABORT, self.on_abort)
            self.cleanup()

    def _execute(self):
        self.wait_for(self.multisite_spec.delay_before_starting)
        if self.should_abort:
            return

        experiment_start = time.localtime()
        cycle_num_to_sites_list = site_visit_order(
            self.multisite_spec.sites, self.multisite_spec.frequencies, self.multisite_spec.optimize_order
        )
        cycle_start_time = time.time()
        for cycle_num in range(self.multisite_spec.num_cycles):
            site_ids = cycle_num_to_sites_list[cycle_num % len(cycle_num_to_sites_list)]
            if cycle_num != 0:
                cycle_duration = self.multisite_spec.cycle_durations[
                    (cycle_num - 1) % len(self.multisite_spec.cycle_durations)
                ]
                # Move to the first site.
                if site_ids:
                    cockpit.interfaces.stage_mover.wait_for_stop()
                    cockpit.interfaces.stage_mover.go_to_site(
                        site_ids[0], should_block=True
                    )
                # Wait for when the next cycle should start.
                wait_time = cycle_start_time + cycle_duration - time.time()
                if not self.wait_for(wait_time) and cycle_duration > 0:
                    _logger.warning(
                        "Couldn't finish cycle in time; off by %.2f seconds",
                        -wait_time,
                    )
            if self.should_abort:
                break
            _logger.info(
                "Starting cycle %d of %d", cycle_num + 1, self.multisite_spec.num_cycles
            )
            cycle_start_time = time.time()
            for site_id in site_ids:
                if self.should_abort:
                    break
                _logger.info(f"Imaging site {site_id} at {time.time():.2f}")
                self.image_site(site_id, cycle_num, experiment_start)
            if self.should_abort:
                break

    def image_site(self, site_id, cycle_num, experiment_start):
        """Go to the specified site and run our experiment on it."""
        site_specs = self.multisite_spec.site_experiment
        if self.multisite_spec.light_frequencies_offsets:
            exposure_settings = filter_exposure_settings(
                site_specs.exposure_settings,
                self.multisite_spec.light_frequencies_offsets,
                cycle_num
            )
            if not exposure_settings:
                _logger.info(
                    "No lights in use on cycle %d; skipping site %s",
                    cycle_num,
                    site_id,
                )
                return
            # Don't modify the shared spec: lights filtered out on this cycle
            # must still be available on later cycles.
            site_specs = dataclasses.replace(
                site_specs, exposure_settings=exposure_settings
            )

        events.publish(
            events.UPDATE_STATUS_LIGHT,
            "device waiting",
            "Waiting for stage motion",
        )
        cockpit.interfaces.stage_mover.wait_for_stop()
        cockpit.interfaces.stage_mover.go_to_site(site_id, should_block=True)
        self.wait_for(self.multisite_spec.delay_before_imaging)
        if self.should_abort:
            return

        # Zero-pad the site ID, when it is a number, so that filenames
        # sort properly.
        try:
            site_name = "%03d" % int(site_id)
        except ValueError:
            site_name = str(site_id)
        mappings = {
            "date": time.strftime("%Y%m%d", experiment_start),
            "time": time.strftime("%H%M", experiment_start),
            "cycle": "%03d" % cycle_num,
            "site": site_name,
        }
        _logger.info("Imaging site %s", site_id)
        start = time.time()
        self.current_experiment = site_specs.run(mappings=mappings)
        if self.current_experiment is None:
            _logger.warning(
                "Experiment at site %s, cycle %d was not started",
                site_id,
                cycle_num,
            )
            return
        self.current_experiment.wait()
        _logger.info("Imaging took %.2f seconds", time.time() - start)

    ## Clean up after the experiment ends.
    def cleanup(self):
        if self.multisite_spec.power_down_when_done:
            for handler in depot.get_handlers_of_type(depot.POWER_CONTROL):
                handler.disable()
        events.publish(events.UPDATE_STATUS_LIGHT, "device waiting", "")

    def wait_for(self, seconds):
        """Wait for some time, allowing the user to abort the wait. Return True
        if we were successful (i.e. handed a valid amount of time to wait for).
        """
        if seconds <= 0:
            return False
        _logger.info("Waiting for %.2f seconds", seconds)
        end_time = time.time() + seconds
        cur_time = time.time()
        while cur_time < end_time and not self.should_abort:
            if int(cur_time + 0.25) != int(cur_time):
                # Advanced to a new second; update the status light.
                remaining = end_time - cur_time
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
