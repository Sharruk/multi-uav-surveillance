"""
FigureGenerator — publication-ready figures and LaTeX tables for STIRS-2025.

Generates outputs needed for an IEEE paper:

  fig1_scenario_overview.png  — 2×3 grid of scenario screenshots (5 scenarios + legend)
  fig2_fps_comparison.png     — horizontal bar chart: 3-drone vs 5-drone FPS
  table1_scenario_specs.tex   — LaTeX table of scenario specifications
  table2_performance.tex      — LaTeX table of FPS benchmarks

All figures use only the Python standard library + numpy (no matplotlib).
If matplotlib is available, richer charts are produced automatically.

Usage:
    python utils/figure_generator.py --output-dir paper_figures/

    # Or from code:
    from utils.figure_generator import FigureGenerator
    gen = FigureGenerator(output_dir="paper_figures/")
    gen.generate_all()
"""

import os
import sys
import struct
import zlib
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

# ── Ground-truth data (from actual benchmark runs, Phase 3) ──────────────────
SCENARIO_SPECS = [
    # name, crowd, birds, buildings, city_m, block_m, road_m, height_range, arena_m
    ('Downtown',    15,  8, 'High',   '26×26', 6.0, 2.0, '6–18 m',  15),
    ('Event',       70,  8, 'Medium', '26×26', 7.0, 2.0, '4–12 m',  15),
    ('Residential',  8,  8, 'Medium', '24×24', 6.0, 2.0, '3–9 m',   14),
    ('Mixed',       35,  8, 'Medium', '24×24', 6.0, 2.0, '3–14 m',  14),
    ('Industrial',  10,  8, 'Low',    '26×26', 8.0, 3.0, '4–10 m',  15),
]

FPS_DATA = {
    #           3-drone   5-drone
    'Downtown':    (126.4, 108.9),
    'Event':       (132.7, 118.1),
    'Residential': (193.8, 134.7),
    'Mixed':       (153.8, 117.2),
    'Industrial':  (179.5, 133.5),
}


class FigureGenerator:

    def __init__(self, output_dir: str = "paper_figures/",
                 scenario_screenshot_dir: str = "screenshots/"):
        self.output_dir    = output_dir
        self.screenshot_dir = scenario_screenshot_dir
        os.makedirs(output_dir, exist_ok=True)

    # ── Public API ─────────────────────────────────────────────────────────────

    def generate_all(self):
        """Generate every figure and table."""
        print("[FigureGenerator] Generating all outputs...")
        self.generate_fps_chart()
        self.generate_latex_tables()
        self.generate_scenario_overview()
        print(f"[FigureGenerator] Done. Files in: {self.output_dir}")

    def generate_fps_chart(self):
        """
        Horizontal grouped bar chart comparing 3-drone vs 5-drone FPS.
        Uses matplotlib if available, otherwise writes a plain-text version.
        """
        path_png  = os.path.join(self.output_dir, "fig2_fps_comparison.png")
        path_txt  = os.path.join(self.output_dir, "fig2_fps_comparison.txt")

        scenarios = list(FPS_DATA.keys())
        fps3      = [FPS_DATA[s][0] for s in scenarios]
        fps5      = [FPS_DATA[s][1] for s in scenarios]

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt

            fig, ax = plt.subplots(figsize=(8, 4))
            y    = np.arange(len(scenarios))
            h    = 0.35
            b1   = ax.barh(y + h/2, fps3, h, label='3-drone', color='#2196F3', alpha=0.85)
            b2   = ax.barh(y - h/2, fps5, h, label='5-drone', color='#FF5722', alpha=0.85)
            ax.axvline(100, color='#4CAF50', linestyle='--', linewidth=1.2,
                       label='100 FPS target')
            ax.set_yticks(y)
            ax.set_yticklabels(scenarios, fontsize=10)
            ax.set_xlabel('Steps per Second (FPS)', fontsize=10)
            ax.set_title('STIRS-2025: Simulation Performance by Scenario', fontsize=11)
            ax.legend(fontsize=9)
            ax.set_xlim(0, 220)
            for bar in list(b1) + list(b2):
                w = bar.get_width()
                ax.text(w + 2, bar.get_y() + bar.get_height()/2,
                        f'{w:.0f}', va='center', fontsize=8)
            plt.tight_layout()
            plt.savefig(path_png, dpi=150, bbox_inches='tight')
            plt.close()
            print(f"  Saved: {path_png}")
        except ImportError:
            # Plain-text fallback
            lines = ["FPS Comparison (3-drone vs 5-drone load, headless)", "-"*54]
            for s in scenarios:
                f3, f5 = FPS_DATA[s]
                bar3 = '#' * int(f3 / 5)
                bar5 = '#' * int(f5 / 5)
                lines.append(f"{s:<14} 3-drone {f3:>6.1f} {bar3}")
                lines.append(f"{'':14} 5-drone {f5:>6.1f} {bar5}")
            with open(path_txt, 'w') as f:
                f.write('\n'.join(lines))
            print(f"  Saved (text fallback): {path_txt}")

    def generate_scenario_overview(self):
        """
        Assemble a 2×3 PNG grid from existing scenario screenshots.
        Looks for phase3_*_top_down.png or phase3_*_*.png in screenshot_dir.
        Falls back to a placeholder text file if screenshots are missing.
        """
        path_out = os.path.join(self.output_dir, "fig1_scenario_overview.png")

        # Canonical screenshot names (generated by phase3_showcase.py /
        # ScreenshotTool.capture_scenario)
        candidates = {
            'downtown':    ['downtown_top_down.png', 'phase3_downtown_full.png'],
            'event':       ['event_top_down.png', 'phase3_event_crowd_top.png'],
            'residential': ['residential_top_down.png', 'phase3_residential_park.png'],
            'mixed':       ['mixed_top_down.png'],
            'industrial':  ['industrial_top_down.png'],
        }

        panels = []
        for scn, names in candidates.items():
            img = None
            for name in names:
                p = os.path.join(self.screenshot_dir, name)
                if os.path.isfile(p):
                    img = self._load_png(p)
                    if img is not None:
                        break
            panels.append((scn, img))

        if all(img is None for _, img in panels):
            msg = ("fig1_scenario_overview: no screenshots found in "
                   f"{self.screenshot_dir}.\n"
                   "Run scripts/phase3_showcase.py first to generate them.")
            path_txt = path_out.replace('.png', '_missing.txt')
            with open(path_txt, 'w') as f:
                f.write(msg)
            print(f"  NOTE: {msg}")
            return

        # Resize all panels to a common size and stitch
        try:
            from PIL import Image as PILImage
            thumb_w, thumb_h = 640, 360
            imgs = []
            for scn, img in panels:
                if img is not None:
                    pil = PILImage.fromarray(img)
                    pil = pil.resize((thumb_w, thumb_h), PILImage.LANCZOS)
                else:
                    pil = PILImage.new('RGB', (thumb_w, thumb_h), (40, 40, 40))
                imgs.append((scn, pil))

            cols, rows = 3, 2
            grid_w = cols * thumb_w
            grid_h = rows * thumb_h
            grid   = PILImage.new('RGB', (grid_w, grid_h), (20, 20, 20))
            for i, (scn, pil) in enumerate(imgs[:cols * rows]):
                r, c = divmod(i, cols)
                grid.paste(pil, (c * thumb_w, r * thumb_h))
            grid.save(path_out)
            print(f"  Saved: {path_out}")
        except ImportError:
            # numpy-only stitch (best effort)
            self._stitch_numpy(panels, path_out)

    def generate_latex_tables(self):
        """Write LaTeX source for Tables 1 and 2."""
        self._write_table1()
        self._write_table2()

    # ── LaTeX table helpers ────────────────────────────────────────────────────

    def _write_table1(self):
        """Table 1: Scenario Specifications."""
        path = os.path.join(self.output_dir, "table1_scenario_specs.tex")
        rows = []
        for name, crowd, birds, density, city, block, road, hrange, arena in SCENARIO_SPECS:
            rows.append(
                f"  {name} & {city} m & {block} m & {hrange} & "
                f"{density} & {crowd} & {birds} & {arena} m \\\\"
            )
        tex = (
            r"\begin{table}[h]" + "\n"
            r"\centering" + "\n"
            r"\caption{STIRS-2025 Simulation Scenario Specifications}" + "\n"
            r"\label{tab:scenarios}" + "\n"
            r"\begin{tabular}{|l|c|c|c|c|c|c|c|}" + "\n"
            r"\hline" + "\n"
            r"\textbf{Scenario} & \textbf{City} & \textbf{Block} & "
            r"\textbf{Bldg. Height} & \textbf{Density} & "
            r"\textbf{Crowd} & \textbf{Birds} & \textbf{Arena} \\" + "\n"
            r"\hline" + "\n"
            + "\n".join(rows) + "\n"
            r"\hline" + "\n"
            r"\end{tabular}" + "\n"
            r"\end{table}"
        )
        with open(path, 'w') as f:
            f.write(tex)
        print(f"  Saved: {path}")

    def _write_table2(self):
        """Table 2: Performance Benchmarks."""
        path = os.path.join(self.output_dir, "table2_performance.tex")
        rows = []
        for name, crowd, birds, *_ in SCENARIO_SPECS:
            f3, f5 = FPS_DATA[name]
            flag3  = r"\checkmark" if f3 >= 100 else r"\times"
            flag5  = r"\checkmark" if f5 >= 100 else r"\times"
            rows.append(
                f"  {name} & {crowd} & {birds} & "
                f"{f3:.1f} {flag3} & {f5:.1f} {flag5} \\\\"
            )
        tex = (
            r"\begin{table}[h]" + "\n"
            r"\centering" + "\n"
            r"\caption{STIRS-2025 Simulation Performance (headless, fixed layout)}" + "\n"
            r"\label{tab:performance}" + "\n"
            r"\begin{tabular}{|l|c|c|c|c|}" + "\n"
            r"\hline" + "\n"
            r"\textbf{Scenario} & \textbf{Crowd} & \textbf{Birds} & "
            r"\textbf{3-Drone FPS} & \textbf{5-Drone FPS} \\" + "\n"
            r"\hline" + "\n"
            + "\n".join(rows) + "\n"
            r"\hline" + "\n"
            r"\multicolumn{5}{|l|}{\small Target: $\geq 100$ FPS. "
            r"All scenarios pass at both drone counts.} \\" + "\n"
            r"\hline" + "\n"
            r"\end{tabular}" + "\n"
            r"\end{table}"
        )
        with open(path, 'w') as f:
            f.write(tex)
        print(f"  Saved: {path}")

    # ── PNG utilities ──────────────────────────────────────────────────────────

    @staticmethod
    def _load_png(path: str):
        """Load PNG to numpy RGB array. Returns None on failure."""
        try:
            from PIL import Image
            return np.array(Image.open(path).convert('RGB'))
        except Exception:
            pass
        try:
            import png
            r = png.Reader(filename=path)
            w, h, rows, info = r.read_flat()
            arr = np.frombuffer(rows, dtype=np.uint8).reshape(h, w, -1)
            return arr[:, :, :3]
        except Exception:
            return None

    def _stitch_numpy(self, panels, path_out):
        """Last-resort numpy stitch: stack panels horizontally."""
        valid = [(s, img) for s, img in panels if img is not None]
        if not valid:
            return
        # Resize to same height
        h_target = min(img.shape[0] for _, img in valid)
        resized  = []
        for s, img in valid:
            scale = h_target / img.shape[0]
            new_w = int(img.shape[1] * scale)
            # nearest-neighbour resize
            ys = (np.arange(h_target) / scale).astype(int)
            xs = (np.arange(new_w)    / scale).astype(int)
            resized.append(img[np.ix_(ys, xs)])
        row   = np.concatenate(resized, axis=1)
        _write_png_rgb(path_out, row)
        print(f"  Saved (numpy stitch): {path_out}")


def _write_png_rgb(path: str, rgb_hwc: np.ndarray):
    h, w = rgb_hwc.shape[:2]
    def chunk(tag, data):
        crc = zlib.crc32(tag + data) & 0xFFFFFFFF
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", crc)
    raw  = b"".join(b"\x00" + rgb_hwc[y].tobytes() for y in range(h))
    out  = b"\x89PNG\r\n\x1a\n"
    out += chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
    out += chunk(b"IDAT", zlib.compress(raw, 6))
    out += chunk(b"IEND", b"")
    with open(path, "wb") as f:
        f.write(out)


# ── CLI ────────────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    import argparse
    p = argparse.ArgumentParser(description='STIRS-2025 Figure Generator')
    p.add_argument('--output-dir', default='paper_figures/',
                   help='Output directory (default: paper_figures/)')
    p.add_argument('--screenshot-dir', default='screenshots/',
                   help='Source screenshot directory (default: screenshots/)')
    p.add_argument('--fps-chart', action='store_true',
                   help='Generate only FPS comparison chart')
    p.add_argument('--tables', action='store_true',
                   help='Generate only LaTeX tables')
    p.add_argument('--overview', action='store_true',
                   help='Generate only scenario overview panel')
    args = p.parse_args()

    gen = FigureGenerator(output_dir=args.output_dir,
                          scenario_screenshot_dir=args.screenshot_dir)

    if args.fps_chart:
        gen.generate_fps_chart()
    elif args.tables:
        gen.generate_latex_tables()
    elif args.overview:
        gen.generate_scenario_overview()
    else:
        gen.generate_all()
