#!/usr/bin/python3
# Short example script of attaching to cockpit to find nuclei in DAPI images.
# Copyright Ian Dobbie, Julio Mateos Langerak 2021
import datetime
import time

import wx

# some imports to get at cockpit functions
from cockpit import depot, events
from cockpit.experiment import multi_site_runner, experiment_specs
import cockpit.util.threads
from cockpit.interfaces import stage_mover

mosaic_window = next(
    (
        win
        for win in wx.GetTopLevelWindows()
        if win.GetTitle() == "Mosaic view"
    ),
    None,
)
if mosaic_window is None:
    raise ValueError("No mosaic window open. Please open a mosaic window.")

CYCLE_PHASES = ["G1", "S", "G2/M"]
CYCLE_PHASES_TO_COLOR = [
    (0, 1, 0),
    (1, 0, 0),
    (0, 0, 1),
]


# find nuclei class
class NucleiFinder:
    def __init__(
            self,
            nuclei_finder,
            nuclei_classifier,
            max_sites=10,
            timeout_seconds=10,
            site_marker_size=50
    ):
        # useful values form cockpit objects
        self.finder = nuclei_finder
        self.classifier = nuclei_classifier
        self.stage_class = "S"
        self.max_sites = max_sites
        self.timeout_seconds = datetime.timedelta(seconds=timeout_seconds)
        self.specs = None
        self.site_marker_size = site_marker_size
        self.scan_start_time = None
        self.pixel_size = wx.GetApp().objectives.get_pixel_size()
        self.camera = depot.get_device_with_name("camera")
        self.scan_start_location = None

    def subscribe(self):
        # subscribe to new image event and call onImage
        self.scan_start_time = datetime.datetime.now()
        events.subscribe(events.NEW_IMAGE % self.camera.name, self.on_image)

    ## Receive a new image and process it to find nuclei
    def on_image(self, acquired_image, *args):
        curr_stage_pos = stage_mover.get_position()

        crops, centers = self.finder.find(acquired_image)

        # for each found object mark a positon
        if centers is not None:
            # loop over centers found in image
            for crop, center in zip(crops, centers):
                # covert pos to microns and add stage pos.
                # TODO: check axis logic
                nucleus_abs_pos = (
                    curr_stage_pos[0] + (acquired_image.shape[1] / 2 - center[1]) * self.pixel_size,
                    curr_stage_pos[1] + (acquired_image.shape[0] / 2 - center[0]) * self.pixel_size
                )

                class_id, prob = self.classifier.classify(crop)
                site_color =  CYCLE_PHASES_TO_COLOR[class_id]
                phase = CYCLE_PHASES[class_id]
                # append Z positon to get xyz pos.
                site_pos = [nucleus_abs_pos[0], nucleus_abs_pos[1], curr_stage_pos[2]]
                # save site to marked point list.
                stage_mover.save_site(
                    stage_mover.Site(
                        position=site_pos,
                        group=phase,
                        color=site_color,
                        size=self.site_marker_size
                    )
                )

        if (len(stage_mover.sites_in_group(self.stage_class)) >= self.max_sites or
                self.scan_start_time + self.timeout_seconds < datetime.datetime.now()):
            self.unsubscribe()
            if mosaic_window.mosaic_thread is not None and mosaic_window.mosaic_thread.is_alive():
                mosaic_window.toggle_mosaic()

    def unsubscribe(self):
        events.unsubscribe(events.NEW_IMAGE % self.camera.name, self.on_image)

    def run_scan(self, max_sites = None, timeout_seconds = None):
        if max_sites is not None:
            self.max_sites = max_sites
        if timeout_seconds is not None:
            self.timeout_seconds = timeout_seconds
        if self.scan_start_location is None:
            self.scan_start_location = stage_mover.get_position()
        stage_mover.go_to(self.scan_start_location)

        stage_mover.delete_all_sites()

        self.subscribe()
        mosaic_window.toggle_mosaic()

    def run_experiment(
        self,
        specs = None,
        num_cycles = 5,
        cycle_durations = [10],
        delay_before_starting = 0,
        delay_before_imaging = 0,
        optimize_order = False,
        light_frequencies = None,
    ):
        if self.scan_start_location is None:
            self.scan_start_location = stage_mover.get_position()
        if specs is None:
            if cockpit.gui.dialogs.experiment.single_site_experiment.dialog is None:
                raise Exception("Experiment dialog was not configured.")

            self.specs = cockpit.gui.dialogs.experiment.single_site_experiment.dialog.panel.get_experiment_spec()
        else:
            self.specs = specs
        all_sites = stage_mover.sites_in_group(self.stage_class)
        sites = list(all_sites[:min(len(all_sites), self.max_sites)])
        if sites:
            multi_site_spec = experiment_specs.MultiSiteSpec(
                site_experiment = self.specs,
                sites = sites,
                num_cycles = num_cycles,
                cycle_durations = cycle_durations,
                delay_before_starting = delay_before_starting,
                delay_before_imaging = delay_before_imaging,
                optimize_order = optimize_order,
                light_frequencies = light_frequencies,
            )

            print(f"Imaging sites: {sites}")
            multi_site_spec.sites = sites
            runner = multi_site_runner.MultiSiteRunner(multi_site_spec)
            runner.run(confirm=lambda x: True)
            runner.wait()

        stage_mover.delete_all_sites()
        stage_mover.go_to(self.scan_start_location)

    @cockpit.util.threads.call_in_new_thread
    def run_smart(
        self,
        nr_of_iterations=5
    ):
        for i in range(nr_of_iterations):
            print(f"Running scan iteration {i + 1} out of {nr_of_iterations}")
            self.run_scan()

            time.sleep(1)

            print("Starting experiment")
            self.run_experiment()
