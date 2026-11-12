"""The learned tip localiser (train_tip_localiser.py) for eval_auto's REFINE step.

A ResNet18 regressing the detected tip minus the ring centre (px) from a CROP_PX crop centred on
the ring, drawn as the policy sees it. Same layout and parameter names as
probe_tip_localisation.Tuned, so its state_dict loads here.
"""

import json
from pathlib import Path

import numpy as np
import torch
import torchvision
from torch import nn
from torchvision.models._utils import IntermediateLayerGetter

from lerobot.robots.meca500.autocollect.overlay import TargetStyle, crop_at, draw_target

IMAGENET_MEAN, IMAGENET_STD = (0.485, 0.456, 0.406), (0.229, 0.224, 0.225)


class Head(nn.Module):
    def __init__(self, feat_hw):
        super().__init__()
        self.reduce = nn.Sequential(nn.Conv2d(512, 32, 1), nn.ReLU())
        self.mlp = nn.Sequential(nn.Flatten(), nn.Linear(32 * feat_hw[0] * feat_hw[1], 256), nn.ReLU(), nn.Linear(256, 2))

    def forward(self, f):
        return self.mlp(self.reduce(f))


class Tuned(nn.Module):
    def __init__(self, feat_hw):
        super().__init__()
        m = torchvision.models.resnet18(weights=None)
        self.backbone = IntermediateLayerGetter(m, return_layers={"layer4": "feature_map"})
        self.head = Head(feat_hw)
        self.register_buffer("mean", torch.tensor(IMAGENET_MEAN).view(1, 3, 1, 1))
        self.register_buffer("std", torch.tensor(IMAGENET_STD).view(1, 3, 1, 1))

    def forward(self, x_uint8):
        x = (x_uint8.float() / 255.0 - self.mean) / self.std
        return self.head(self.backbone(x)["feature_map"])


class TipLocaliser:
    def __init__(self, folder: str | Path, device: str = "cuda"):
        folder = Path(folder)
        cfg = json.loads((folder / "config.json").read_text())
        self.crop, self.scale, self.device = cfg["crop_px"], cfg["label_scale_px"], device
        self.style = TargetStyle(**{k: tuple(v) if isinstance(v, list) else v for k, v in cfg["target_style"].items()})
        self.model = Tuned(tuple(cfg["feat_hw"])).to(device)
        self.model.load_state_dict(torch.load(folder / "model.pt", map_location=device))
        self.model.eval()

    @torch.inference_mode()
    def offset_px(self, frame_rgb: np.ndarray, target_px) -> np.ndarray:
        """Tip minus target (px) in one raw RGB frame (no ring drawn yet)."""
        img = crop_at(draw_target(frame_rgb, target_px, self.style), target_px, self.crop)
        x = torch.from_numpy(np.ascontiguousarray(img)).permute(2, 0, 1)[None].to(self.device)
        return self.model(x)[0].float().cpu().numpy() * self.scale
