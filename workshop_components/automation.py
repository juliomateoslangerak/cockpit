#!/usr/bin/python3
# Short example script of attaching to cockpit to find nuclei in DAPI images.
# Copyright Ian Dobbie, Julio Mateos Langerak 2021
import json
import pathlib

import wx

import numpy as np
from csbdeep.models import BaseModel
from stardist.models import StarDist2D
from csbdeep.utils import normalize as csbdeep_normalize
from huggingface_hub import snapshot_download
import torch
from torchvision.models import ResNet18_Weights, resnet18
import albumentations

# some imports to get at cockpit functions
from cockpit import depot, events
import cockpit.util.threads
from cockpit.interfaces import stageMover


HUGGINGFACE_REPO = "thomas-bonte/cell_cycle_classification"
MODEL_SUBFOLDER = "20241101-055937-4998324"
MODEL_FILENAME = "early_stopping_cycle_classification.pt"
MEAN_STD_FILENAME = "mean_std.json"

IN_CHANNELS = 1       # 1 DAPI channel × 5 z-slices
LATENT_DIM = 256
NB_CLASSES = 3
CYCLE_PHASES = ["G1", "S", "G2/M"]
CYCLE_PHASES_TO_COLOR = [
    (0, 1, 0),
    (1, 0, 0),
    (0, 0, 1),
]
EDGE = 10
SCALE = 0.5

# Nucleus crop size (maximum nucleus diameter in pixels) and model input resolution
DATA_SET_SIZE = 280
INPUT_SIZE = 128


def _redefine_first_layer(model, in_channels: int) -> None:
    """Replace ResNet's first conv to accept an arbitrary number of input channels.

    Extra channels beyond 3 are initialised with the mean of the ImageNet weights
    (same strategy as the original codebase).
    """
    orig = model.conv1.weight.data  # (64, 3, 7, 7)
    new_conv = torch.nn.Conv2d(in_channels, 64, kernel_size=7, stride=2, padding=3, bias=False)
    if in_channels >= 3:
        new_conv.weight.data[:, :3] = orig
        mean_weights = orig.mean(dim=1, keepdim=True)  # (64, 1, 7, 7)
        new_conv.weight.data[:, 3:] = mean_weights.expand(-1, in_channels - 3, -1, -1)
    else:
        mean_weights = orig.mean(dim=1, keepdim=True)
        new_conv.weight.data = mean_weights.expand(-1, in_channels, -1, -1)
    model.conv1 = new_conv

class FucciClassifier(torch.nn.Module):
    """Frozen ResNet18 encoder followed by a two-layer MLP classifier."""

    def __init__(
        self,
        in_channels: int = IN_CHANNELS,
        latent_dim: int = LATENT_DIM,
        nb_classes: int = NB_CLASSES,
    ):
        super().__init__()
        self.encoder = _ResnetEncoder(in_channels, latent_dim)
        self.fc1 = torch.nn.Linear(latent_dim, latent_dim)
        self.fc2 = torch.nn.Linear(latent_dim, nb_classes)

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        emb = self.encoder(x)             # (B, latent_dim)
        return self.fc2(self.fc1(emb))    # (B, nb_classes)


class _ResnetEncoder(torch.nn.Module):
    """ResNet18 backbone with VAE-style linear heads.

    The state-dict keys match the original ResnetEncoder so that pretrained
    weights can be loaded directly with load_state_dict().
    """

    def __init__(self, in_channels: int, latent_dim: int):
        super().__init__()
        backbone = resnet18(weights=ResNet18_Weights.DEFAULT)
        _redefine_first_layer(backbone, in_channels)
        feat_size = backbone.fc.in_features  # 512 for ResNet18
        backbone.fc = torch.nn.Identity()
        self.conv_layers = backbone

        # These names must match the saved checkpoint exactly
        self.embedding = torch.nn.Linear(feat_size, latent_dim)
        self.log_var = torch.nn.Linear(feat_size, latent_dim)
        self.fucci = torch.nn.Sequential(torch.nn.Linear(latent_dim, 2), torch.nn.Sigmoid())

    def forward(self, x: torch.Tensor) -> torch.Tensor:
        h = self.conv_layers(x)           # (B, 512)
        return self.embedding(h)          # (B, latent_dim)


def _ensure_weights(models_dir: str) -> pathlib.Path:
    """Download pretrained weights from HuggingFace if not already present."""
    model_dir = pathlib.Path(models_dir) / MODEL_SUBFOLDER
    if not model_dir.exists():
        print(f"Downloading pretrained weights from {HUGGINGFACE_REPO} ...")
        snapshot_download(HUGGINGFACE_REPO, local_dir=models_dir)
    return model_dir


def load_stardist_model(model_name: str = "2D_versatile_fluo"):
    return StarDist2D.from_pretrained(model_name)


def load_ccc_model(ccc_models_dir: str = "models"):
    ccc_model_dir = _ensure_weights(ccc_models_dir)
    ccc_model_path = ccc_model_dir / MODEL_FILENAME
    ccc_mean_std_path = ccc_model_dir / MEAN_STD_FILENAME

    with open(ccc_mean_std_path) as f:
        ccc_mean_std = json.load(f)

    ccc_model = FucciClassifier()
    state_dict = torch.load(ccc_model_path, map_location="cpu", weights_only=True)
    ccc_model.load_state_dict(state_dict)
    ccc_model.eval()

    return ccc_model, ccc_mean_std


def preprocess(array: np.ndarray, mean_std: dict) -> torch.Tensor:
    """Preprocess a z-stack numpy array and return a model-ready tensor.

    Parameters
    ----------
    array : np.ndarray
        Shape (Z, H, W). uint16 values are divided by 65 535; float32 arrays
        are assumed to already be in [0, 1].
    mean_std : dict
        Normalisation statistics with keys "mean" and "std" (lists of length Z).

    Returns
    -------
    torch.Tensor of shape (1, Z, INPUT_SIZE, INPUT_SIZE)
    """
    if array.dtype == np.uint16:
        img = array.astype(np.float32) / 65535.0
    else:
        img = array.astype(np.float32)

    img = np.moveaxis(img, 0, -1)  # (H, W, Z) — albumentations expects HWC

    transform = albumentations.Compose([
        albumentations.Normalize(
            mean=mean_std["mean"],
            std=mean_std["std"],
            max_pixel_value=1.0,
            p=1.0,
        ),
        albumentations.PadIfNeeded(
            min_height=DATA_SET_SIZE,
            min_width=DATA_SET_SIZE,
            border_mode=1,          # cv2.BORDER_REPLICATE
            p=1.0,
        ),
        albumentations.CenterCrop(height=DATA_SET_SIZE, width=DATA_SET_SIZE, p=1.0),
        albumentations.Resize(height=INPUT_SIZE, width=INPUT_SIZE, p=1.0),
    ])

    img = transform(image=img)["image"]  # (INPUT_SIZE, INPUT_SIZE, Z)
    img = np.clip(img, 0.0, 1.0)

    tensor = torch.from_numpy(img).permute(2, 0, 1).float()  # (Z, H, W)
    return tensor.unsqueeze(0)                                # (1, Z, H, W)


# ── Inference ──────────────────────────────────────────────────────────────────
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
        # Stardist swaps dimensions
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


def predict_nucleus(
    array: np.ndarray,
    ccc_model: FucciClassifier,
    ccc_mean_std: dict,
) -> tuple[tuple, str]:
    """Predict the cell cycle phase for a single nucleus crop.

    Parameters
    ----------
    array : np.ndarray
        Shape (Z, H, W). uint16 or float32 in [0, 1].
    ccc_model : FucciClassifier
        Model returned by :func:`load_model`.
    ccc_mean_std : dict
        Normalisation statistics returned by :func:`load_model`.

    Returns
    -------
    str
        One of "G1", "S", or "G2/M".
    """
    pre_processed = preprocess(array, ccc_mean_std)
    with torch.no_grad():
        logits = ccc_model(pre_processed)  # (1, 3)

    class_id = logits.argmax(dim=1).item()
    return CYCLE_PHASES_TO_COLOR[class_id], CYCLE_PHASES[class_id]


# find nuclei class
class NucleiStageFinder:
    def __init__(self, *args, **kwargs):
        # useful values form cockpit objects
        self.pixel_size = wx.GetApp().Objectives.GetPixelSize()
        self.site_size = 20
        self.camera = depot.getDeviceWithName("camera")
        self.stardist_model = load_stardist_model()
        self.ccc_model, self.ccc_mean_std = load_ccc_model()

    # start finding Nuclei in images.
    def start(self):
        # subscribe to new image event and call onImage
        events.subscribe(events.NEW_IMAGE % self.camera.name, self.on_image)

    ## Receive a new image and process it to find nuclei
    # @cockpit.util.threads.callInNewThread
    def on_image(self, acquired_image, *args):
        curr_stage_pos = stageMover.getPosition()

        crops, centers = find_nuclei(
            acquired_image,
            self.stardist_model,
        )

        # for each found circle mark a positon
        if centers is not None:
            # loop over centers found in image
            for crop, center in zip(crops, centers):
                # covert pos to microns and add stage pos.
                nucleus_abs_pos = (
                    curr_stage_pos[0] + (acquired_image.shape[0] / 2 - center[0]) * self.pixel_size,
                    curr_stage_pos[1] + (acquired_image.shape[1] / 2 - center[1]) * self.pixel_size
                )

                site_color, phase = predict_nucleus(crop, self.ccc_model, self.ccc_mean_std)
                print(f"at pos: {nucleus_abs_pos}, phase: {phase}")

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

    # stop finding Nuclei in images.
    def stop(self):
        events.unsubscribe(events.NEW_IMAGE % self.camera.name, self.on_image)
