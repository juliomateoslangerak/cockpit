#!/usr/bin/python3
"""Nucleus classification functionality for the cockpit workshop.

This module contains the logic for classifying nuclei using cell cycle phase models.
"""

import json
import pathlib

import numpy as np
from huggingface_hub import snapshot_download
import torch
from torchvision.models import ResNet18_Weights, resnet18
import albumentations


# Constants for nucleus classification
IN_CHANNELS = 1       # 1 DAPI channel × 5 z-slices
LATENT_DIM = 256
NB_CLASSES = 3
CYCLE_PHASES = ["G1", "S", "G2/M"]
CYCLE_PHASES_TO_COLOR = [
    (0, 1, 0),
    (1, 0, 0),
    (0, 0, 1),
]
DATA_SET_SIZE = 280
INPUT_SIZE = 128

# Model configuration
HUGGINGFACE_REPO = "thomas-bonte/cell_cycle_classification"
MODEL_SUBFOLDER = "20261001-112625-local"
MODEL_FILENAME = "early_stopping_fucci_vae.pt"
MEAN_STD_FILENAME = "mean_std.json"


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
    elif array.dtype == np.uint8:
        img = array.astype(np.float32) / 255.0
    else:
        img = array.astype(np.float32)

    img = np.moveaxis(img, 0, -1)  # (H, W, Z) — albumentations expects HWC

    transform = albumentations.Compose([
        # albumentations.Normalize(
        #     mean=mean_std["mean"],
        #     std=mean_std["std"],
        #     max_pixel_value=1.0,
        #     p=1.0,
        # ),
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
    # img = np.clip(img, 0.0, 1.0)
    img = np.repeat(img[:, :, np.newaxis], repeats=IN_CHANNELS, axis=-1)
    tensor = torch.from_numpy(img).permute(2, 0, 1).float()   # (Z, H, W)
    return tensor.unsqueeze(0)                                # (1, Z, H, W)


def load_ccc_model(ccc_models_dir: str = "models"):
    """Load the cell cycle classification model and its normalization statistics."""
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


class Classifier:
    def __init__(self):
        self.ccc_model, self.ccc_mean_std = load_ccc_model()

    def classify(self, array: np.ndarray) -> tuple[tuple, str]:
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
        pre_processed = preprocess(array, self.ccc_mean_std)
        with torch.no_grad():
            logits = self.ccc_model(pre_processed)  # (1, 3)

        class_id = logits.argmax(dim=1).item()
        class_prob = logits[class_id]

        return class_id, class_prob
