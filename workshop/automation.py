#!/usr/bin/python3
# Short example script of attaching to cockpit to find objects of interest.
# Julio Mateos Langerak 2026

import datetime

import wx


# some imports to get at cockpit functions
from cockpit import depot, events
from cockpit.experiment import multi_site_runner, experiment_specs
import cockpit.util.threads
from cockpit.interfaces import stage_mover

CAMERA_TIMEOUT = 1

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


class SmartFinder:
    """Find objects of interest in images, classify them and
    trigger experiments on them.
    """

    def __init__(
        self,
        object_finder,
        object_classifier,
        site_marker_size=50,
    ):
        self.finder = object_finder
        self.classifier = object_classifier
        self.stage_class = "S"
        self.max_sites = 0
        self.timeout_seconds = None
        self.site_marker_size = site_marker_size
        self.scan_start_time = None
        self.pixel_size = None
        self.camera = depot.get_device_with_name("camera")
        self.scan_start_location = None
        self.should_run = False

    def subscribe(self):
        """Subscribe to new image event and call onImage"""
        self.scan_start_time = datetime.datetime.now()
        events.subscribe(events.NEW_IMAGE % self.camera.name, self.on_image)

    def unsubscribe(self):
        events.unsubscribe(events.NEW_IMAGE % self.camera.name, self.on_image)

    def abort(self):
        print("aborting")
        self.should_run = False

    def on_image(self, acquired_image, *args):
        """Receive a new image and process it to find objects"""
        curr_stage_pos = stage_mover.get_position()

        crops, centers = self.finder.find(acquired_image)

        # for each found object mark a positon
        if centers is not None:
            # loop over centers found in image
            for crop, center in zip(crops, centers):
                # covert pos to microns and add stage pos.
                # TODO: check axis logic
                nucleus_abs_pos = (
                    curr_stage_pos[0]
                    + (acquired_image.shape[1] / 2 - center[1])
                    * self.pixel_size,
                    curr_stage_pos[1]
                    + (acquired_image.shape[0] / 2 - center[0])
                    * self.pixel_size,
                )

                class_id, prob = self.classifier.classify(crop)
                site_color = CYCLE_PHASES_TO_COLOR[class_id]
                phase = CYCLE_PHASES[class_id]
                # append Z positon to get xyz pos.
                site_pos = [
                    nucleus_abs_pos[0],
                    nucleus_abs_pos[1],
                    curr_stage_pos[2],
                ]
                # save site to marked point list.
                stage_mover.save_site(
                    stage_mover.Site(
                        position=site_pos,
                        group=phase,
                        color=site_color,
                        size=self.site_marker_size,
                    )
                )

    def _mosaic_stepper(self, overlap):
        """Generate spiral offsets (in tile units) with the given overlap percentage.
        Mirrors the algorithm in MosaicWindow.mosaic_stepper().
        """
        directions = [(0, -1), (-1, 0), (0, 1), (1, 0)]
        cur_spiral_size = 1
        last_x = last_y = 0
        i = 0
        while True:
            dx, dy = directions[i % 4]
            dx *= 1 - (overlap / 100.0)
            dy *= 1 - (overlap / 100.0)
            for j in range(1, cur_spiral_size + 1):
                yield (last_x + dx * j, last_y + dy * j)
            last_x += dx * cur_spiral_size
            last_y += dy * cur_spiral_size
            if i % 2:
                cur_spiral_size += 1
            i += 1

    def run_prescan(self, max_sites=None, timeout_seconds=None):
        """Run a spiral mosaic synchronously: move the stage, take images,
        add them to the mosaic canvas, and process each image to find objects.
        Stops when max_sites is reached or the timeout expires.

        This replaces the previous approach of delegating to the GUI mosaic
        thread (via toggle_mosaic), which caused race conditions with the
        multi-site experiment.
        """
        self.pixel_size = wx.GetApp().objectives.get_pixel_size()

        if max_sites is not None:
            self.max_sites = max_sites
        if timeout_seconds is not None:
            self.timeout_seconds = timeout_seconds
        if self.scan_start_location is None:
            self.scan_start_location = stage_mover.get_position()

        stage_mover.go_to(self.scan_start_location)

        stage_mover.delete_all_sites()

        # --- Set up camera and image size (same as mosaic_loop reconfigure) ---
        active = wx.GetApp().depot.get_active_cameras()
        if len(active) == 0:
            print("No active cameras for prescan.")
            return
        camera = self.camera
        if camera not in active:
            camera = active[0]

        width_px, height_px = camera.get_image_size()
        width = width_px * self.pixel_size
        height = height_px * self.pixel_size

        # --- Spiral parameters ---
        overlap = mosaic_window.overlap
        stepper = self._mosaic_stepper(overlap)

        position = stage_mover.get_position()

        self.scan_start_time = datetime.datetime.now()

        # --- Mosaic loop: spiral, take image, add to canvas, process ---
        while self.should_run:
            if (
                self.max_sites > 0
                and len(stage_mover.sites_in_group(self.stage_class))
                >= self.max_sites
            ):
                break
            if (
                self.timeout_seconds is not None
                and self.scan_start_time + self.timeout_seconds
                < datetime.datetime.now()
            ):
                break

            # Compute next target position
            try:
                dx, dy = next(stepper)
            except Exception as e:
                print(f"Mosaic stopping - problem in stepper: {e}")
                break

            target = (
                position[0] + dx * width,
                position[1] + dy * height,
            )

            # Move stage to target
            try:
                mosaic_window.go_to(target, should_block=True)
            except Exception as e:
                print(f"Mosaic stopping - problem moving stage: {e}")
                break

            # Take an image
            try:
                data, metadata = events.execute_and_wait_for_or_timeout(
                    events.NEW_IMAGE % camera.name,
                    wx.GetApp().imager.take_image,
                    camera.get_exposure_time() / 1000
                    + camera.get_time_between_exposures() / 1000
                    + CAMERA_TIMEOUT,
                    should_block=True,
                )
            except Exception as e:
                print(f"Mosaic stopping - problem taking image: {e}")
                break

            if data is None:
                print("Mosaic stopping - take_image returned None (timeout).")
                break

            # Add image to mosaic canvas
            position = stage_mover.get_position()
            mosaic_window.transfer_camera_image()

            # Process the image to find objects
            crops, centers = self.finder.find(data)
            if centers is not None:
                for crop, center in zip(crops, centers):
                    nucleus_abs_pos = (
                        position[0]
                        + (data.shape[1] / 2 - center[1]) * self.pixel_size,
                        position[1]
                        + (data.shape[0] / 2 - center[0]) * self.pixel_size,
                    )
                    class_id, prob = self.classifier.classify(crop)
                    site_color = CYCLE_PHASES_TO_COLOR[class_id]
                    phase = CYCLE_PHASES[class_id]
                    site_pos = [
                        nucleus_abs_pos[0],
                        nucleus_abs_pos[1],
                        position[2],
                    ]
                    stage_mover.save_site(
                        stage_mover.Site(
                            position=site_pos,
                            group=phase,
                            color=site_color,
                            size=self.site_marker_size,
                        )
                    )

    def run_multisite_experiment(
        self,
        multisite_experiment_specs,
        sites,
    ):
        if sites:
            print(f"Imaging sites: {sites}")
            multisite_experiment_specs.sites = sites
            runner = multi_site_runner.MultiSiteRunner(
                multisite_experiment_specs
            )
            runner.run(confirm=lambda x: True)
            runner.wait()
        else:
            print("No sites to image")

        stage_mover.delete_all_sites()
        stage_mover.go_to(self.scan_start_location)

    @cockpit.util.threads.call_in_new_thread
    def run_smart(
        self,
        nr_of_iterations=5,
        max_sites=10,
        timeout_seconds=10,
        single_experiment_specs=None,
        multisite_experiment_specs=None,
    ):
        self.should_run = True
        events.subscribe(events.USER_ABORT, self.abort)
        self.max_sites = max_sites
        self.timeout_seconds = datetime.timedelta(seconds=timeout_seconds)

        if self.scan_start_location is None:
            self.scan_start_location = stage_mover.get_position()

        if single_experiment_specs is None:
            if (
                cockpit.gui.dialogs.experiment.single_site_experiment.dialog
                is None
            ):
                raise Exception("Experiment dialog was not configured.")

            single_experiment_specs = (
                cockpit.gui.dialogs.experiment.single_site_experiment.dialog.panel.get_experiment_spec()
            )
        else:
            single_experiment_specs = single_experiment_specs

        if multisite_experiment_specs is None:
            multisite_experiment_specs = (
                cockpit.experiment.experiment_specs.MultiSiteSpec(
                    site_experiment=single_experiment_specs,
                    sites=[],
                    num_cycles=5,
                    cycle_durations=[10],
                    delay_before_starting=0,
                    delay_before_imaging=0,
                    optimize_order=False,
                    light_frequencies=None,
                )
            )

        for i in range(nr_of_iterations):
            if not self.should_run:
                return
            print(f"Running scan iteration {i + 1} out of {nr_of_iterations}")
            self.run_prescan()

            all_sites = stage_mover.sites_in_group(self.stage_class)
            sites = list(all_sites[: min(len(all_sites), self.max_sites)])

            print("Starting experiment")
            self.run_multisite_experiment(
                multisite_experiment_specs=multisite_experiment_specs,
                sites=sites,
            )
