"""
ScreenshotTool — standardised multi-angle captures for STIRS-2025.

Four canonical angles captured per call:
  top_down   — straight overhead bird's-eye view
  isometric  — 45° diagonal overview
  street     — low-angle near-ground perspective
  drone_pov  — looking down from drone altitude

All angles write lossless PNG using the same pure-Python encoder
as scripts/phase3_showcase.py (no PIL dependency).

Usage:
    tool = ScreenshotTool(output_dir="screenshots/")
    paths = tool.capture_scenario(client_id, scenario="downtown")
    # → {"top_down": "screenshots/downtown_top_down.png", ...}

    # Or capture a single custom angle:
    tool.capture_single(client_id, "custom_view",
                        dist=15, pitch=-35, yaw=20, target=(0,0,2))
"""

import os
import struct
import zlib
import numpy as np


class ScreenshotTool:

    # Canonical camera presets: (dist, pitch, yaw, target_z_offset, label)
    ANGLES = [
        (20,  -90,   0,   0.0, "top_down"),
        (22,  -45,  35,   2.0, "isometric"),
        (14,  -20,  18,   0.5, "street"),
        (18,  -55,  20,   3.0, "drone_pov"),
    ]

    def __init__(self, output_dir: str = "screenshots/",
                 width: int = 1280, height: int = 720):
        self.output_dir = output_dir
        self.width      = width
        self.height     = height
        os.makedirs(output_dir, exist_ok=True)

    # ── Public API ─────────────────────────────────────────────────────────────

    def capture_scenario(self, client_id: int, scenario: str,
                         target: tuple = (0, 0, 0)) -> dict:
        """
        Capture all four canonical angles for a scenario.

        Returns a dict mapping angle label → file path.
        """
        paths = {}
        for dist, pitch, yaw, tz, label in self.ANGLES:
            tgt = (target[0], target[1], target[2] + tz)
            fname = f"{scenario}_{label}.png"
            path  = self.capture_single(client_id, fname,
                                         dist=dist, pitch=pitch,
                                         yaw=yaw, target=tgt)
            paths[label] = path
        return paths

    def capture_single(self, client_id: int, filename: str,
                       dist: float, pitch: float, yaw: float,
                       target: tuple = (0, 0, 2)) -> str:
        """
        Capture one frame and write it to output_dir/filename.

        Returns the full file path.
        """
        rgba = self._render(client_id, dist, pitch, yaw, target)
        if not filename.endswith(".png"):
            filename += ".png"
        path = os.path.join(self.output_dir, filename)
        self._write_png(path, rgba)
        return path

    def capture_flock_view(self, client_id: int, env,
                            scenario: str) -> str:
        """
        Capture a bird-flock centred view by querying boid_birds positions.
        Falls back to origin if no birds are present.
        """
        if env.boid_birds and len(env.boid_birds.positions):
            cx = float(np.mean(env.boid_birds.positions[:, 0]))
            cy = float(np.mean(env.boid_birds.positions[:, 1]))
            cz = float(np.mean(env.boid_birds.positions[:, 2]))
        else:
            cx, cy, cz = 0.0, 0.0, 5.0
        return self.capture_single(client_id, f"{scenario}_birds_flock.png",
                                    dist=18, pitch=-18, yaw=55,
                                    target=(cx, cy, cz))

    # ── Internal ───────────────────────────────────────────────────────────────

    def _render(self, client_id: int,
                dist: float, pitch: float, yaw: float,
                target: tuple) -> np.ndarray:
        import pybullet as p
        w, h = self.width, self.height
        vm = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=list(target),
            distance=dist, yaw=yaw, pitch=pitch, roll=0,
            upAxisIndex=2, physicsClientId=client_id)
        pm = p.computeProjectionMatrixFOV(
            fov=55, aspect=w / h, nearVal=0.2, farVal=120,
            physicsClientId=client_id)
        _, _, rgba, _, _ = p.getCameraImage(
            width=w, height=h,
            viewMatrix=vm, projectionMatrix=pm,
            renderer=p.ER_TINY_RENDERER,
            lightDirection=[1.5, 2.5, 5.0],
            lightColor=[1.0, 0.97, 0.92],
            lightDistance=120,
            lightAmbientCoeff=0.45,
            lightDiffuseCoeff=0.88,
            lightSpecularCoeff=0.3,
            physicsClientId=client_id)
        return np.array(rgba, dtype=np.uint8).reshape(h, w, 4)

    @staticmethod
    def _write_png(path: str, rgba_hwc: np.ndarray):
        h, w = rgba_hwc.shape[:2]
        rgb  = rgba_hwc[:, :, :3].astype(np.uint8)
        def chunk(tag, data):
            crc = zlib.crc32(tag + data) & 0xFFFFFFFF
            return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)
        raw  = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))
        out  = b"\x89PNG\r\n\x1a\n"
        out += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        out += chunk(b"IDAT", zlib.compress(raw, 6))
        out += chunk(b"IEND", b"")
        with open(path, "wb") as f:
            f.write(out)
