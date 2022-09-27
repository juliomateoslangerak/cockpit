#!/usr/bin/env python

from microscope.devices import device
import microscope.testsuite.devices as testdevices
from microscope.simulators.stage_aware_camera import simulated_setup_from_image

DEVICE_SERVER_PORT = 8000
SIMULATION_IMAGE_FILEPATH = 'merged-zaber-rgb.jpg'


DEVICES = [
        device(
            simulated_setup_from_image,
            "127.0.0.1",
            8000,
            conf={"filepath": SIMULATION_IMAGE_FILEPATH},
        ),
    ]
