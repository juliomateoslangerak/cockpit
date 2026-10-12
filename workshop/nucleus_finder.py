#!/usr/bin/python3
"""Nucleus finding functionality for the cockpit workshop.

This module contains the logic for detecting nuclei in images using StarDist models.
"""

import numpy as np
from csbdeep.utils import normalize as csbdeep_normalize
from stardist.models import StarDist2D

# Constants for nucleus finding
EDGE = 10
SCALE = 0.5


def load_stardist_model(model_name: str = "2D_versatile_fluo"):
    return StarDist2D.from_pretrained(model_name)


class Finder:
    def __init__(self):
        self.stardist_model = load_stardist_model()
        self.edge = EDGE
        self.scale = SCALE
        self.min_x_crop = 100
        self.min_y_crop = 100
    def find(self, image: np.ndarray) -> tuple[list[np.ndarray], list[tuple[float, float]]]:
        """Segment nuclei in a field-of-view image and return their crops and centres.

        Parameters
        ----------
        image : np.ndarray
            Shape (X, Y), uint16 or float32.

        Returns
        -------
        crops : list[np.ndarray]
            Per-nucleus arrays of shape (X_crop, Y_crop).
        centers : list[tuple[float, float]]
            (x, Y) centroid coordinates for each returned nucleus.
        """
        _, details = self.stardist_model.predict_instances(csbdeep_normalize(image), scale=scale)

        x_shape, y_shape = image.shape
        crops = []
        centers = []
        for coord in details["coord"]:
            # Stardist swaps dimensions
            x0, x1 = int(np.floor(coord[0].min())) - self.edge, int(np.ceil(coord[0].max())) + self.edge
            y0, y1 = int(np.floor(coord[1].min())) - self.edge, int(np.ceil(coord[1].max())) + self.edge
            x_center = x0 + (x1 - x0) // 2
            y_center = y0 + (y1 - y0) // 2
            # skip nuclei touching the image border
            if x0 <= 0 or x1 >= x_shape or y0 <= 0 or y1 >= y_shape:
                continue
            # skip nuclei smaller than the min crop size
            if  x1 - x0 < self.min_x_crop or y1 - y0 < self.min_y_crop:
                continue
            crops.append(image[x0:x1, y0:y1])
            centers.append((x_center, y_center))

        return crops, centers