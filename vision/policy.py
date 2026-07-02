"""Sidewalk-keeping policy: front-camera image -> steering (linear, angular).

Behavior cloning. Two backbones:
  - "tiny"    : small 3-conv net, trains fast on CPU/MPS (used to prove the pipeline
                on the synthetic task, and fine as a light real-data model).
  - "resnet18": torchvision ResNet-18 (ImageNet init) for the real FrodoBots data.

action_dim=2 -> (linear, angular). Leave configurable so the same head fits a
waypoint-chunk action later (e.g. Berkeley-7K MBRA labels).
"""
import torch
import torch.nn as nn


class TinyBackbone(nn.Module):
    def __init__(self, out=64):
        super().__init__()
        # Pool over HEIGHT but keep WIDTH (8 columns): steering is a horizontal-position
        # task, so global pooling would throw away the one signal that matters.
        self.net = nn.Sequential(
            nn.Conv2d(3, 16, 5, stride=2, padding=2), nn.ReLU(),
            nn.Conv2d(16, 32, 3, stride=2, padding=1), nn.ReLU(),
            nn.Conv2d(32, 64, 3, stride=2, padding=1), nn.ReLU(),
            nn.AdaptiveAvgPool2d((1, 8)), nn.Flatten(),
        )
        self.out_dim = 64 * 8

    def forward(self, x):
        return self.net(x)


class SidewalkPolicy(nn.Module):
    def __init__(self, backbone="tiny", action_dim=2):
        super().__init__()
        if backbone == "tiny":
            self.backbone = TinyBackbone()
            feat = self.backbone.out_dim
        elif backbone == "resnet18":
            from torchvision.models import resnet18, ResNet18_Weights
            m = resnet18(weights=ResNet18_Weights.DEFAULT)
            feat = m.fc.in_features
            m.fc = nn.Identity()
            self.backbone = m
        else:
            raise ValueError(f"unknown backbone {backbone}")
        self.head = nn.Sequential(
            nn.Linear(feat, 64), nn.ReLU(), nn.Linear(64, action_dim)
        )

    def forward(self, x):
        return self.head(self.backbone(x))

    @torch.no_grad()
    def act(self, img_chw, device="cpu"):
        """img_chw: float tensor [3,H,W] in 0..1 -> (linear, angular) clamped."""
        self.eval()
        out = self(img_chw.unsqueeze(0).to(device))[0]
        linear = float(out[0].clamp(0, 1))
        angular = float(out[1].clamp(-1, 1))
        return linear, angular
