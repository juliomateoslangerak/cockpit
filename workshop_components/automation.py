#!/usr/bin/python3
# Short example script of attaching to cockpit to find nuclei in DAPI images.
# Copyright Ian Dobbie, Julio Mateos Langerak 2021

import wx

import numpy as np
from stardist.models import StarDist2D
from csbdeep.utils import normalize as csbdeep_normalize

# some imports to get at cockpit functions
from cockpit import depot, events
import cockpit.util.threads
from cockpit.gui.mosaic import window as mosaicWindow
from cockpit.interfaces import stageMover


def load_stardist_model(model_name: str = "2D_versatile_fluo"):
    return StarDist2D.from_pretrained(model_name)

def find_nuclei(
    image: np.ndarray,
    stardist_model: BaseModel,
    edge: int = 10,
    scale: float = 0.5,
    min_x_crop: int = 100,
    min_y_crop: int = 100,
) -> tuple[list[np.ndarray], list[tuple[float, float]]]:
    """Segment nuclei in a field-of-view image and return their crops and centres.

    Parameters
    ----------
    image : np.ndarray
        Shape (X, Y), uint16 or float32.
    stardist_model : StarDist2D
        StarDist model returned by :func:`load_stardist_model`.
    edge : int
        Margin in pixels added around each detected bounding box. Nuclei whose
        padded bounding box touches the image border are discarded.
    scale : float
        Rescaling factor passed to StarDist (default 0.5).
    min_x_crop : int
        Minimum nucleus X crop (default 100).
    min_y_crop : int
        Minimum nucleus Y crop (default 100).

    Returns
    -------
    crops : list[np.ndarray]
        Per-nucleus arrays of shape (X_crop, Y_crop).
    centers : list[tuple[float, float]]
        (x, Y) centroid coordinates for each returned nucleus.
    """
    _, details = stardist_model.predict_instances(csbdeep_normalize(image), scale=scale)

    x_shape, y_shape = image.shape
    crops = []
    centers = []
    for coord in details["coord"]:
        # Stardist swaps dimmensions
        x0, x1 = int(np.floor(coord[1].min())) - edge, int(np.ceil(coord[1].max())) + edge
        y0, y1 = int(np.floor(coord[0].min())) - edge, int(np.ceil(coord[0].max())) + edge
        x_center = x0 + (x1 - x0) // 2
        y_center = y0 + (y1 - y0) // 2
        # skip nuclei touching the image border
        if x0 <= 0 or x1 >= x_shape or y0 <= 0 or y1 >= y_shape:
            continue
        # skip nuclei smaller than the min crop size
        if  x1 - x0 < min_x_crop or y1 - y0 < min_y_crop:
            continue
        crops.append(image[x0:x1, y0:y1])
        centers.append((x_center, y_center))

    return crops, centers


# find nuclei class
class FindNuclei:
    def __init__(self, *args, **kwargs):
        # useful values form cockpit objects
        self.pixel_size = wx.GetApp().Objectives.GetPixelSize()
        self.site_color = mosaicWindow.window.siteColor
        self.site_size = 20
        self.camera = depot.getDeviceWithName("camera")
        self.stardist_model = load_stardist_model()

    # start finding Nuclei in images.
    def start(self):
        # subscribe to new image event and call onImage
        events.subscribe(events.NEW_IMAGE % self.camera.name, self.onImage)

    ## Receive a new image and process it to find nuclei
    @cockpit.util.threads.callInNewThread
    def onImage(self, acquired_image, *args):
        curr_stage_pos = stageMover.getPosition()

        crops, centers = find_nuclei(
            acquired_image,
            self.stardist_model,
        )

        # for each found circle mark a positon
        if centers is not None:
            # loop over centers found in image
            for center in centers:
                print(center)
                # covert pos to microns and add stage pos.
                nucleus_abs_pos = (
                    curr_stage_pos[0] + (acquired_image.shape[0] / 2 - center[0]) * self.pixel_size,
                    curr_stage_pos[1] + (acquired_image.shape[1] / 2 - center[1]) * self.pixel_size
                )
                print(nucleus_abs_pos)
                # append Z positon to get xyz pos.
                site_pos = [nucleus_abs_pos[0], nucleus_abs_pos[1], curr_stage_pos[2]]
                # save site to marked point list.
                stageMover.saveSite(
                    stageMover.Site(
                        site_pos, None, self.site_color, size=self.site_size
                    )
                )

    # stop finding Nuclei in images.
    def stop(self):
        events.unsubscribe(events.NEW_IMAGE % self.camera.name, self.onImage)
