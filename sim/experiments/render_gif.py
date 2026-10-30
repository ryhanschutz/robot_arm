"""Gera docs/images/simulacao.gif (loop de 6 s) renderizando models/arm.xml offscreen.

Uso: python sim/experiments/render_gif.py
Usa o Mesa de sim/x64 quando presente (mesmo mecanismo de animar_robo.py).
"""
import os
import ctypes
import math

HERE = os.path.dirname(os.path.abspath(__file__))
X64 = os.path.join(HERE, "..", "x64")
os.environ["PATH"] = X64 + os.pathsep + os.environ.get("PATH", "")
for dll in ["dxil.dll", "libglapi.dll", "libgallium_wgl.dll", "opengl32.dll"]:
    p = os.path.join(X64, dll)
    if os.path.exists(p):
        try:
            ctypes.CDLL(p)
        except Exception:
            pass

import numpy as np
import mujoco
from PIL import Image

XML = os.path.abspath(os.path.join(HERE, "..", "..", "models", "arm.xml"))
OUT = os.path.abspath(os.path.join(HERE, "..", "..", "docs", "images", "simulacao.gif"))
os.makedirs(os.path.dirname(OUT), exist_ok=True)

W, H = 560, 420
FPS, DURATION = 20, 6.0
N = int(FPS * DURATION)

m = mujoco.MjModel.from_xml_path(XML)
d = mujoco.MjData(m)
r = mujoco.Renderer(m, H, W)

frames = []
for i in range(N):
    ph = 2 * math.pi * i / N  # fase 0..2pi => loop perfeito
    # mesmas amplitudes do animar_robo.py, com frequencias inteiras para fechar o loop
    j1 = math.radians(45.0 * math.sin(ph))
    j2 = math.radians(25.0 * math.sin(2 * ph))
    j3 = math.radians(25.0 * math.cos(2 * ph))
    j4 = math.radians(15.0 + 15.0 * math.sin(3 * ph))
    d.qpos[:5] = [j1, j2, j3, j4, j4]
    mujoco.mj_forward(m, d)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = [0.025, 0.065, 0.09]
    cam.distance = 0.20
    cam.azimuth = 135 + 25 * math.sin(ph)  # leve orbita da camera
    cam.elevation = -22
    r.update_scene(d, camera=cam)
    frames.append(Image.fromarray(r.render()).quantize(colors=128, method=Image.Quantize.MEDIANCUT, dither=Image.Dither.NONE))

frames[0].save(OUT, save_all=True, append_images=frames[1:], duration=int(1000 / FPS), loop=0, optimize=True, disposal=2)
print("Saved:", OUT, f"({os.path.getsize(OUT) / 1e6:.1f} MB, {N} frames)")
