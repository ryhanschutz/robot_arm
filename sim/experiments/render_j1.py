"""Renderiza o teste_junta1.xml offscreen em 0, 45 e 90 graus (PNG), usando o Mesa de simulation/x64."""
import os, ctypes, sys
HERE = os.path.dirname(os.path.abspath(__file__))
X64 = os.path.join(HERE, "..", "x64")
os.environ["PATH"] = X64 + os.pathsep + os.environ.get("PATH", "")
for dll in ["dxil.dll", "libglapi.dll", "libgallium_wgl.dll", "opengl32.dll"]:
    p = os.path.join(X64, dll)
    if os.path.exists(p):
        try: ctypes.CDLL(p)
        except Exception as e: print("aviso:", dll, e)

import numpy as np, mujoco
from PIL import Image

nome = sys.argv[1] if len(sys.argv) > 1 else "teste_junta1.xml"
xml = os.path.join(HERE, "..", "..", "models", "experiments", nome)
m = mujoco.MjModel.from_xml_path(os.path.abspath(xml))
d = mujoco.MjData(m)
out = os.path.join(HERE, "render_j1_" + os.path.splitext(nome)[0])
os.makedirs(out, exist_ok=True)

r = mujoco.Renderer(m, 480, 480)
cam = mujoco.MjvCamera()
cam.type = mujoco.mjtCamera.mjCAMERA_FREE
cam.lookat[:] = [0.02, 0.05, 0.08]
cam.distance = 0.45
cam.azimuth, cam.elevation = 130, -90  # vista de cima
for ang in (0, 45, 90):
    d.qpos[0] = np.radians(ang)
    mujoco.mj_forward(m, d)
    r.update_scene(d, camera=cam)
    Image.fromarray(r.render()).save(os.path.join(out, f"j1_{ang:02d}.png"))
    print("ok", ang)

