#!/usr/bin/python3
# Short example script of attaching to cockpit to find nuclei in DAPI images.
# Copyright Ian Dobbie, Julio Mateos Langerak 2021
import datetime
import time

import wx

# some imports to get at cockpit functions
from cockpit import depot, events
from cockpit.gui.dialogs.experiment import multiSiteExperiment
from cockpit.experiment import multiSiteRunner, experimentSpecs
import cockpit.util.threads
from cockpit.interfaces import stageMover

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

# Nucleus crop size (maximum nucleus diameter in pixels) and model input resolution
DATA_SET_SIZE = 280
INPUT_SIZE = 128

CYCLE_PHASES = ["G1", "S", "G2/M"]
CYCLE_PHASES_TO_COLOR = [
    (0, 1, 0),
    (1, 0, 0),
    (0, 0, 1),
]


# find nuclei class
class NucleiFinder:
    def __init__(self, nuclei_finder, nuclei_classifier, *args, **kwargs):
        # useful values form cockpit objects
        self.finder = nuclei_finder
        self.classifier = nuclei_classifier
        self.pixel_size = wx.GetApp().Objectives.GetPixelSize()
        self.site_size = 50
        self.camera = depot.getDeviceWithName("camera")
        self.experiment_specs = None
        self.stage_class = "S"
        self.max_sites = 3
        self.scan_start_time = None
        self.scan_start_location = None
        self.timeout_seconds = datetime.timedelta(seconds=10)

    def subscribe(self):
        # subscribe to new image event and call onImage
        self.scan_start_time = datetime.datetime.now()
        events.subscribe(events.NEW_IMAGE % self.camera.name, self.on_image)

    ## Receive a new image and process it to find nuclei
    def on_image(self, acquired_image, *args):
        curr_stage_pos = stageMover.getPosition()

        crops, centers = self.finder.find(acquired_image)

        # for each found circle mark a positon
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
                stageMover.saveSite(
                    stageMover.Site(
                        position=site_pos,
                        group=phase,
                        color=site_color,
                        size=self.site_size
                    )
                )
        # print(f"Found {len(stageMover.sitesInGroup(self.stage_class))} sites")
        if (len(stageMover.sitesInGroup(self.stage_class)) >= self.max_sites or
                self.scan_start_time + self.timeout_seconds < datetime.datetime.now()):
            self.unsubscribe()
            if mosaic_window.mosaicThread is not None and mosaic_window.mosaicThread.is_alive():
                mosaic_window.toggleMosaic()

    def unsubscribe(self):
        events.unsubscribe(events.NEW_IMAGE % self.camera.name, self.on_image)

    def run_scan(self, max_sites = None, timeout_seconds = None):
        if max_sites is not None:
            self.max_sites = max_sites
        if timeout_seconds is not None:
            self.timeout_seconds = timeout_seconds
        if self.scan_start_location is None:
            self.scan_start_location = stageMover.getPosition()
        stageMover.goTo(self.scan_start_location)

        stageMover.deleteAllSites()

        self.subscribe()
        mosaic_window.toggleMosaic()

    def run_experiment(self):
        if self.scan_start_location is None:
            self.scan_start_location = stageMover.getPosition()
        if cockpit.gui.dialogs.experiment.singleSiteExperiment.dialog is None:
            raise Exception("Experiment dialog was not configured.")

        self.experiment_specs = cockpit.gui.dialogs.experiment.singleSiteExperiment.dialog.panel.getExperimentSpec()
        all_sites = stageMover.sitesInGroup(self.stage_class)
        sites = [s for s in all_sites[:min(len(all_sites), self.max_sites)]]
        if sites:
            multi_site_spec = experimentSpecs.MultiSiteSpec(
                siteExperiment = self.experiment_specs,
                sites = sites,
                numCycles = 5,
                cycleDurations = [10],
                delayBeforeStarting = 0,
                delayBeforeImaging = 0,
                optimizeOrder = False,
                lightFrequencies = None,
            )

            print(f"Imaging sites: {sites}")
            multi_site_spec.sites = sites
            runner = multiSiteRunner.MultiSiteRunner(multi_site_spec)
            runner.run(confirm=lambda x: True)
            runner.wait()

        stageMover.deleteAllSites()
        stageMover.goTo(self.scan_start_location)

    @cockpit.util.threads.callInNewThread
    def run_smart(self, nr_of_iterations=5):
        for i in range(nr_of_iterations):
            print(f"Running scan iteration {i + 1} out of {nr_of_iterations}")
            self.run_scan()

            time.sleep(1)

            print("Starting experiment")
            self.run_experiment()
