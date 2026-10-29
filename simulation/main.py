import os
import sys
import time
import math
import ctypes

# Ativação do renderizador OpenGL por software (Mesa3D D3D12/LLVMpipe)
def enable_software_opengl():
    sim_dir = os.path.dirname(os.path.abspath(__file__))
    x64_dir = os.path.join(sim_dir, "x64")
    if os.path.exists(x64_dir):
        os.environ["PATH"] = x64_dir + os.pathsep + os.environ.get("PATH", "")
        for dll in ["dxil.dll", "libglapi.dll", "libgallium_wgl.dll", "opengl32.dll"]:
            p = os.path.join(x64_dir, dll)
            if os.path.exists(p):
                try:
                    ctypes.CDLL(p)
                except Exception:
                    pass

enable_software_opengl()

import mujoco
import mujoco.viewer

def main():
    xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
    
    print(f"Carregando modelo: {xml_path}")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print("=== Robô Articulado Inicializado no MuJoCo ===")
    print(f"Número de Juntas / DoF: {model.nv}")
    print(f"Número de Atuadores (Servos MG90S): {model.nu}")
    print("Iniciando visualizador 3D interativo...")
    print("Controles no visualizador:")
    print("  - Botão esquerdo do mouse: Rotacionar câmera")
    print("  - Botão direito do mouse: Pan / Deslocar câmera")
    print("  - Scroll do mouse: Zoom")
    print("  - Tecla ESPAÇO: Pausar / Despausar simulação")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        
        while viewer.is_running():
            step_start = time.time()
            t = time.time() - start_time

            # Movimentação harmônica suave dos servos para validação cinemática
            # 1. Base / Tampa rotativa (-45° a +45°)
            data.ctrl[0] = math.radians(45.0 * math.sin(t * 1.0))
            
            # 2. Ombro (-15° a +35°)
            data.ctrl[1] = math.radians(25.0 * math.sin(t * 1.2))
            
            # 3. Cotovelo (-45° a +15°)
            data.ctrl[2] = math.radians(30.0 * math.sin(t * 1.2))
            
            # 4. Garra (0° a 30° - abre e fecha ambas as mandíbulas por engrenagem)
            #data.ctrl[3] = math.radians(15.0 * (1.0 - math.cos(t * 2.0)))

            # Executa passo de física
            mujoco.mj_step(model, data)

            # Sincroniza visualização
            viewer.sync()

            # Mantém taxa de tempo real (~500 Hz físico)
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

if __name__ == "__main__":
    main()
