"""Abre o viewer interativo do MuJoCo animando todas as juntas automaticamente em tempo real."""
import os
import sys
import time
import math
import ctypes

HERE = os.path.dirname(os.path.abspath(__file__))
X64 = os.path.join(HERE, "x64")
os.environ["PATH"] = X64 + os.pathsep + os.environ.get("PATH", "")
for dll in ["dxil.dll", "libglapi.dll", "libgallium_wgl.dll", "opengl32.dll"]:
    p = os.path.join(X64, dll)
    if os.path.exists(p):
        try:
            ctypes.CDLL(p)
        except Exception:
            pass

import mujoco
import mujoco.viewer

XML_DEFAULT = os.path.join(HERE, "..", "models", "arm.xml")
xml_path = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.abspath(XML_DEFAULT)

model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

print(f"Simulando: {xml_path}")
print("Juntas ativas:", [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i) for i in range(model.njnt)])
print("Feche a janela do viewer para encerrar.")

with mujoco.viewer.launch_passive(model, data) as viewer:
    viewer.cam.lookat[:] = [0.025, 0.065, 0.085]
    viewer.cam.distance = 0.28
    viewer.cam.azimuth = 135
    viewer.cam.elevation = -22

    t0 = time.time()
    while viewer.is_running():
        t = time.time() - t0

        # Movimentos senoidais suaves em radianos para demonstrar todas as juntas
        j1 = math.radians(45.0 * math.sin(1.2 * t))          # Giro da base (-45 a +45 deg)
        j2 = math.radians(25.0 * math.sin(1.8 * t))          # Ombro (-25 a +25 deg)
        j3 = math.radians(25.0 * math.cos(1.8 * t))          # Cotovelo (-25 a +25 deg)
        j4 = math.radians(15.0 + 15.0 * math.sin(3.0 * t))   # Garra (0 fechada a 30 deg aberta)

        data.qpos[0] = j1
        data.qpos[1] = j2
        data.qpos[2] = j3
        data.qpos[3] = j4
        data.qpos[4] = j4

        mujoco.mj_forward(model, data)
        viewer.sync()
        time.sleep(0.016)

