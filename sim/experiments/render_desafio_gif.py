"""Renderiza o GIF e o painel de etapas do Desafio Pick-and-Drop (pegar a bolinha e soltar no chão em loop).

Gera:
  - docs/images/desafio_bolinha.gif
  - docs/images/painel_desafio_bolinha.png

Uso:
  python sim/experiments/render_desafio_gif.py
"""
import os
import sys
import ctypes

HERE = os.path.dirname(os.path.abspath(__file__))
SIM_DIR = os.path.abspath(os.path.join(HERE, ".."))
X64 = os.path.join(SIM_DIR, "x64")
if os.path.exists(X64):
    os.environ["PATH"] = X64 + os.pathsep + os.environ.get("PATH", "")
    for dll in ["dxil.dll", "libglapi.dll", "libgallium_wgl.dll", "opengl32.dll"]:
        p = os.path.join(X64, dll)
        if os.path.exists(p):
            try:
                ctypes.CDLL(p)
            except Exception:
                pass

sys.path.insert(0, SIM_DIR)
import numpy as np
import mujoco
from PIL import Image, ImageDraw, ImageFont
from simulador import PickAndDropLoopController, XML_DEFAULT, clamp_safe_workspace

OUT_DIR = os.path.abspath(os.path.join(HERE, "..", "..", "docs", "images"))
os.makedirs(OUT_DIR, exist_ok=True)
GIF_PATH = os.path.join(OUT_DIR, "desafio_bolinha.gif")
PANEL_PATH = os.path.join(OUT_DIR, "painel_desafio_bolinha.png")

try:
    FONT = ImageFont.truetype("segoeui.ttf", 14)
except Exception:
    try:
        FONT = ImageFont.truetype("arial.ttf", 14)
    except Exception:
        FONT = ImageFont.load_default()

W, H = 520, 400
FPS = 20

model = mujoco.MjModel.from_xml_path(XML_DEFAULT)
data = mujoco.MjData(model)
for _ in range(40):
    mujoco.mj_step(model, data)

ctrl = PickAndDropLoopController(model, data)
ctrl.start()

renderer = mujoco.Renderer(model, H, W)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [0.025, 0.100, 0.068]
cam.distance = 0.25
cam.azimuth = 140
cam.elevation = -24

dt = model.opt.timestep
steps_per_frame = int(round((1.0 / FPS) / dt))

# Simula 2 ciclos completos (~10.8 s) para o GIF
total_duration = 10.8
n_frames = int(total_duration * FPS)
frames = []
stage_snapshots = {}

for f in range(n_frames):
    for _ in range(steps_per_frame):
        cmd_deg = clamp_safe_workspace(*ctrl.step(dt))
        data.ctrl[:4] = np.radians(cmd_deg)
        mujoco.mj_step(model, data)

    cam.azimuth = 138 + 12 * np.sin(2 * np.pi * f / n_frames)
    renderer.update_scene(data, camera=cam)
    img = Image.fromarray(renderer.render())
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 0, W, 28], fill=(17, 24, 39))
    draw.text((10, 5), ctrl.status_text, fill=(250, 204, 21), font=FONT)

    # Captura snapshots das 6 fases do Ciclo #1 para o painel estático
    if ctrl.cycle_count == 1 and ctrl.phase_idx in (1, 2, 3, 4, 5, 6):
        if ctrl.phase_time >= ctrl.phase_duration * 0.65 and ctrl.phase_idx not in stage_snapshots:
            stage_snapshots[ctrl.phase_idx] = img.copy()

    frames.append(
        img.quantize(colors=128, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE)
    )

frames[0].save(
    GIF_PATH,
    save_all=True,
    append_images=frames[1:],
    duration=int(1000 / FPS),
    loop=0,
    optimize=True,
    disposal=2,
)
print(f"GIF salvo em: {GIF_PATH} ({os.path.getsize(GIF_PATH)/1e6:.2f} MB, {len(frames)} frames)")

if len(stage_snapshots) == 6:
    grid = Image.new("RGB", (W * 3, H * 2))
    for idx, ph in enumerate([1, 2, 3, 4, 5, 6]):
        col = idx % 3
        row = idx // 3
        grid.paste(stage_snapshots[ph], (col * W, row * H))
    grid.save(PANEL_PATH)
    print(f"Painel salvo em: {PANEL_PATH}")
