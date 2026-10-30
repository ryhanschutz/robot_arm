"""Renderiza o robo_completo.xml exercitando todas as juntas (J1, J2, J3 e Garra)."""
import os
import ctypes

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
from PIL import Image, ImageDraw

xml_path = os.path.join(HERE, "..", "..", "models", "arm.xml")
m = mujoco.MjModel.from_xml_path(os.path.abspath(xml_path))
d = mujoco.MjData(m)

out_dir = os.path.join(HERE, "render_completo")
os.makedirs(out_dir, exist_ok=True)

W, H = 420, 360
r = mujoco.Renderer(m, H, W)

def render_pose(q_deg, azimuth=135, elevation=-22, distance=0.26, lookat=(0.025, 0.065, 0.085), label=""):
    mujoco.mj_resetData(m, d)
    # qpos: [j1_base, j2_ombro, j3_cotovelo, j4_esq, j4_dir]
    for i, val in enumerate(q_deg):
        d.qpos[i] = np.radians(val)
    mujoco.mj_forward(m, d)

    cam = mujoco.MjvCamera()
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.lookat[:] = lookat
    cam.distance = distance
    cam.azimuth = azimuth
    cam.elevation = elevation

    r.update_scene(d, camera=cam)
    img = Image.fromarray(r.render())
    draw = ImageDraw.Draw(img)
    draw.rectangle([0, 0, W, 26], fill=(25, 25, 25))
    draw.text((10, 6), label, fill=(255, 255, 255))
    return img

# 1. Pose neutra (0, 0, 0, 0, 0)
p1 = render_pose([0, 0, 0, 0, 0], azimuth=140, elevation=-20, label="1. Pose Neutra (0, 0, 0, Garra Fechada)")

# 2. J2 Ombro movendo (-25 graus e +25 graus) - vista lateral (azimuth=0 olha ao longo de X)
p2 = render_pose([0, -25, 0, 0, 0], azimuth=15, elevation=-15, label="2. J2 Ombro (-25 deg)")
p3 = render_pose([0, 25, 0, 0, 0], azimuth=15, elevation=-15, label="3. J2 Ombro (+25 deg)")

# 3. J3 Cotovelo movendo (-30 graus e +25 graus)
p4 = render_pose([0, 0, -30, 0, 0], azimuth=15, elevation=-15, label="4. J3 Cotovelo (-30 deg)")
p5 = render_pose([0, 15, 25, 0, 0], azimuth=15, elevation=-15, label="5. J2 (+15) + J3 Cotovelo (+25 deg)")

# 4. Garra aberta + Giro J1
p6 = render_pose([40, -15, 20, 28, 28], azimuth=130, elevation=-35, label="6. Combinado: J1(40) J2(-15) J3(20) Garra Aberta")

grid = Image.new("RGB", (W * 3, H * 2))
for idx, im in enumerate([p1, p2, p3, p4, p5, p6]):
    col = idx % 3
    row = idx // 3
    grid.paste(im, (col * W, row * H))

grid_path = os.path.join(out_dir, "painel_juntas.png")
grid.save(grid_path)
print("Saved:", grid_path)
