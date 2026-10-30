"""Abre o teste_montagem.xml usando o OpenGL por software (Mesa) de simulation/x64."""
import os
import ctypes

HERE = os.path.dirname(os.path.abspath(__file__))
X64 = os.path.join(HERE, "x64")
XML = os.path.join(HERE, "..", "models", "experiments", "teste_montagem.xml")

# Mesma ativação do main.py: precisa rodar ANTES de importar mujoco.
os.environ["PATH"] = X64 + os.pathsep + os.environ.get("PATH", "")
for dll in ["dxil.dll", "libglapi.dll", "libgallium_wgl.dll", "opengl32.dll"]:
    p = os.path.join(X64, dll)
    if os.path.exists(p):
        try:
            ctypes.CDLL(p)
        except Exception as e:
            print("aviso:", dll, e)

import mujoco
import mujoco.viewer

import sys
# Uso: python view_mockup.py [caminho_do_xml]  (padrão: teste_montagem.xml)
xml_path = os.path.abspath(sys.argv[1]) if len(sys.argv) > 1 else os.path.abspath(XML)
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)
mujoco.viewer.launch(model, data)

