"""
VideoRecorder — capture PyBullet simulation frames and save as MP4.

Encoding priority:
  1. imageio + ffmpeg (best quality, requires: pip install imageio[ffmpeg])
  2. OpenCV VideoWriter (requires: pip install opencv-python)
  3. PNG frame dump fallback (always available — no extra deps)

Usage:
    recorder = VideoRecorder(output_dir="videos/", fps=30)
    # inside step loop:
    recorder.capture_frame(client_id, width=1280, height=720)
    # after episode:
    path = recorder.save("downtown_ppo_ep01")
    print(f"Saved: {path}")

Camera parameters default to a fixed overhead 3/4 view. Pass a custom
view_matrix and projection_matrix for animated or drone-follow cameras.
"""

import os
import struct
import zlib
import numpy as np


class VideoRecorder:

    def __init__(self, output_dir: str = "videos/",
                 fps: int = 30,
                 resolution: tuple = (1280, 720)):
        self.output_dir = output_dir
        self.fps        = fps
        self.width, self.height = resolution
        self._frames: list = []
        os.makedirs(output_dir, exist_ok=True)

    # ── Frame capture ──────────────────────────────────────────────────────────

    def capture_frame(self, client_id: int,
                      view_matrix=None, proj_matrix=None):
        """
        Render one frame from the PyBullet simulation.

        If view_matrix / proj_matrix are None a default 3/4 overhead view
        centred on the origin is used (matches Phase 3 showcase angles).
        """
        import pybullet as p
        w, h = self.width, self.height

        if view_matrix is None:
            view_matrix = p.computeViewMatrixFromYawPitchRoll(
                cameraTargetPosition=[0, 0, 2],
                distance=22, yaw=35, pitch=-42, roll=0,
                upAxisIndex=2, physicsClientId=client_id)
        if proj_matrix is None:
            proj_matrix = p.computeProjectionMatrixFOV(
                fov=55, aspect=w / h, nearVal=0.2, farVal=120,
                physicsClientId=client_id)

        _, _, rgba, _, _ = p.getCameraImage(
            width=w, height=h,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix,
            renderer=p.ER_TINY_RENDERER,
            lightDirection=[1.5, 2.5, 5.0],
            lightAmbientCoeff=0.45,
            lightDiffuseCoeff=0.88,
            physicsClientId=client_id)

        frame = np.array(rgba, dtype=np.uint8).reshape(h, w, 4)
        self._frames.append(frame[:, :, :3])   # drop alpha for video

    def capture_drone_follow(self, client_id: int, drone_pos: list,
                              follow_dist: float = 8.0, pitch: float = -30):
        """Capture a drone-follow camera frame centred on drone_pos."""
        import pybullet as p
        w, h = self.width, self.height
        vm = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=drone_pos,
            distance=follow_dist, yaw=0, pitch=pitch, roll=0,
            upAxisIndex=2, physicsClientId=client_id)
        pm = p.computeProjectionMatrixFOV(
            fov=60, aspect=w / h, nearVal=0.1, farVal=100,
            physicsClientId=client_id)
        _, _, rgba, _, _ = p.getCameraImage(
            width=w, height=h,
            viewMatrix=vm, projectionMatrix=pm,
            renderer=p.ER_TINY_RENDERER,
            physicsClientId=client_id)
        frame = np.array(rgba, dtype=np.uint8).reshape(h, w, 4)
        self._frames.append(frame[:, :, :3])

    # ── Saving ─────────────────────────────────────────────────────────────────

    def save(self, name: str) -> str:
        """
        Save all captured frames as MP4 (or PNG sequence fallback).

        Returns the path to the saved file.
        """
        if not self._frames:
            raise RuntimeError("No frames captured — call capture_frame() first.")

        mp4_path = os.path.join(self.output_dir, f"{name}.mp4")

        # Try imageio first
        if self._try_imageio(mp4_path):
            self._frames.clear()
            return mp4_path

        # Try OpenCV
        if self._try_opencv(mp4_path):
            self._frames.clear()
            return mp4_path

        # PNG fallback
        png_dir = os.path.join(self.output_dir, name + "_frames")
        self._save_pngs(png_dir)
        self._frames.clear()
        return png_dir

    def frame_count(self) -> int:
        return len(self._frames)

    def clear(self):
        self._frames.clear()

    # ── Encoding helpers ───────────────────────────────────────────────────────

    def _try_imageio(self, path: str) -> bool:
        try:
            import imageio
            writer = imageio.get_writer(path, fps=self.fps, codec='libx264',
                                         quality=8, macro_block_size=None)
            for f in self._frames:
                writer.append_data(f)
            writer.close()
            return True
        except Exception:
            return False

    def _try_opencv(self, path: str) -> bool:
        try:
            import cv2
            h, w = self._frames[0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            out = cv2.VideoWriter(path, fourcc, self.fps, (w, h))
            for f in self._frames:
                out.write(cv2.cvtColor(f, cv2.COLOR_RGB2BGR))
            out.release()
            return True
        except Exception:
            return False

    def _save_pngs(self, out_dir: str):
        os.makedirs(out_dir, exist_ok=True)
        for i, frame in enumerate(self._frames):
            self._write_png(os.path.join(out_dir, f"frame_{i:06d}.png"), frame)
        print(f"[VideoRecorder] MP4 encoding unavailable. "
              f"Frames saved to {out_dir}/ — "
              f"stitch with: ffmpeg -r {self.fps} -i frame_%06d.png out.mp4")

    @staticmethod
    def _write_png(path: str, rgb_hwc: np.ndarray):
        h, w = rgb_hwc.shape[:2]
        def chunk(tag, data):
            crc = zlib.crc32(tag + data) & 0xFFFFFFFF
            return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)
        raw = b"".join(b"\x00" + rgb_hwc[y].tobytes() for y in range(h))
        out  = b"\x89PNG\r\n\x1a\n"
        out += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        out += chunk(b"IDAT", zlib.compress(raw, 6))
        out += chunk(b"IEND", b"")
        with open(path, "wb") as f:
            f.write(out)
