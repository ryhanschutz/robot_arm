"""Simulador interativo realista do braço robótico 4-DOF (Servos MG90 + PCA9685 + ESP32) no MuJoCo.

Funcionalidades:
1. Calibração em Zero Graus (idêntica ao sim/view_mockup.py):
   - Em 0.0° (pulso de 1500 us / tick 307 no PCA9685 a 50 Hz), todas as 4 juntas ficam
     exatamente na pose de montagem do `view_mockup.py`.
2. Modo Manual com sensibilidade reduzida e suavização de velocidade dos servos MG90:
   - Exibe em tempo real para cada junta: Ângulo (°), Pulso (us) e Tick PWM 12-bit (0..4095) do PCA9685.
   - Ponte Serial USB em tempo real (115200 baud) para enviar os sinais diretamente ao ESP32.
3. Modo Desafio (Loop infinito Pick-and-Drop):
   - Localiza a bolinha no chão em tempo real, calcula a Cinemática Inversa 3D exata,
   - Abre a garra, desce até a bolinha, fecha a garra com física real de contato,
   - Levanta a bolinha, gira a base até a próxima zona e solta a bolinha no chão em loop contínuo.

Uso:
  python sim/simulador.py                   # Abre em Modo Manual + Painel MG90/PCA9685
  python sim/simulador.py --desafio         # Já inicia executando o Loop de pegar e soltar a bolinha
  python sim/simulador.py --port COM3       # Conecta ao ESP32 na porta COM3 em tempo real
  python sim/simulador.py --no-gui          # Usa apenas a janela 3D do MuJoCo (sem painel Tkinter)
"""
import os
import sys
import time
import math
import ctypes
import argparse

HERE = os.path.dirname(os.path.abspath(__file__))
X64 = os.path.join(HERE, "x64")
if os.path.exists(X64):
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
import mujoco.viewer

import socket

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

XML_DEFAULT = os.path.abspath(os.path.join(HERE, "..", "models", "arm.xml"))

# Coordenadas exatas dos pivôs no frame neutro (extraídas do CAD / STLs em metros)
P_J1 = np.array([0.022828, 0.051405, 0.054312])
P_J2 = np.array([0.034824, 0.057804, 0.077586])
P_J3 = np.array([0.022953, 0.012061, 0.123375])
P_GRIP0 = np.array([0.023550, 0.126490, 0.106960])
Z_FLOOR = 0.030236
BALL_SPAWN_POS = np.array([0.02355, 0.1558, 0.0410])

# Limites operacionais seguros de cada junta (evitam colisão mecânica entre elos e contra a base/chão)
J1_MIN_DEG, J1_MAX_DEG = -90.0, 90.0   # Base: 544 us .. 2400 us (tick 111 .. 492)
J2_MIN_DEG, J2_MAX_DEG = -72.0, 60.0   # Ombro: 735 us .. 2100 us (tick 151 .. 430)
J3_MIN_DEG, J3_MAX_DEG = -35.0, 65.0   # Cotovelo: 1128 us .. 2150 us (tick 231 .. 440)
GARRA_MIN_DEG = 0.0                    # Garra fechada (0.0° = 1500 us / tick 307 = view_mockup.py)
GARRA_MAX_DEG = 38.0                   # Garra aberta (+38.0° = 1880 us / tick 385)

JOINT_MIN_DEGS = (J1_MIN_DEG, J2_MIN_DEG, J3_MIN_DEG, GARRA_MIN_DEG)
JOINT_MAX_DEGS = (J1_MAX_DEG, J2_MAX_DEG, J3_MAX_DEG, GARRA_MAX_DEG)

# Constantes seguras do Servo MG90 + PCA9685 (padrão oficial Arduino Servo.h / Wokwi)
PCA9685_FREQ_HZ = 50.0
MG90_MIN_US = 544.0      # -90.0° (tick ~111)
MG90_CENTER_US = 1500.0  #   0.0° (tick 307 — Pose do view_mockup.py)
MG90_MAX_US = 2400.0     # +90.0° (tick ~492)


def j3_min_safe_for_j2(j2_deg: float) -> float:
    """Retorna o ângulo mínimo permitido para o cotovelo J3 em função do ombro J2
    para garantir que a garra e o antebraço nunca colidam com o chão (folga >= 3.5 mm).
    """
    val = 0.0051 * (j2_deg * j2_deg) - 0.520 * j2_deg - 35.2
    return max(J3_MIN_DEG, min(J3_MAX_DEG, val))


def clamp_safe_workspace(j1_deg: float, j2_deg: float, j3_deg: float, j4_deg: float):
    """Aplica os limites individuais das 4 juntas e a restrição cinemática acoplada (J2, J3)
    contra colisão com o solo.
    """
    j1 = max(J1_MIN_DEG, min(J1_MAX_DEG, float(j1_deg)))
    j2 = max(J2_MIN_DEG, min(J2_MAX_DEG, float(j2_deg)))
    j3_floor_min = j3_min_safe_for_j2(j2)
    j3 = max(j3_floor_min, min(J3_MAX_DEG, float(j3_deg)))
    j4 = max(GARRA_MIN_DEG, min(GARRA_MAX_DEG, float(j4_deg)))
    return j1, j2, j3, j4


def deg_to_pca9685(deg: float, freq_hz: float = PCA9685_FREQ_HZ):
    """Converte o ângulo da junta (em graus, 0° = view_mockup.py) em (pulso_us, tick_12bit) do PCA9685."""
    clamped = max(-90.0, min(90.0, float(deg)))
    if clamped >= 0.0:
        pulse_us = MG90_CENTER_US + (clamped / 90.0) * (MG90_MAX_US - MG90_CENTER_US)
    else:
        pulse_us = MG90_CENTER_US + (clamped / 90.0) * (MG90_CENTER_US - MG90_MIN_US)
    period_us = 1_000_000.0 / freq_hz
    tick = int(round((pulse_us / period_us) * 4096.0))
    tick = max(0, min(4095, tick))
    return pulse_us, tick


def solve_ik_3d(x: float, y: float, z: float):
    """Cinemática Inversa 3D analítica para posicionar o centro da garra em (x, y, z).

    Retorna (j1_deg, j2_deg, j3_deg) dentro dos limites operacionais seguros e sem colisão com o solo,
    ou None se o ponto estiver fora do alcance seguro.
    """
    z_safe = max(float(z), Z_FLOOR + 0.0135)
    dx = x - P_J1[0]
    dy = y - P_J1[1]
    r_xy = math.hypot(dx, dy)
    dx0 = P_GRIP0[0] - P_J1[0]
    if r_xy < abs(dx0) + 0.045:
        return None

    y_local = math.sqrt(max(1e-8, r_xy * r_xy - dx0 * dx0))
    j1 = math.atan2(-dx, dy) + math.atan2(dx0, y_local)
    y_eq = P_J1[1] + y_local

    v1_y, v1_z = P_J3[1] - P_J2[1], P_J3[2] - P_J2[2]
    v2_y, v2_z = P_GRIP0[1] - P_J3[1], P_GRIP0[2] - P_J3[2]
    L1 = math.hypot(v1_y, v1_z)
    L2 = math.hypot(v2_y, v2_z)
    phi1 = math.atan2(v1_z, v1_y)
    phi2 = math.atan2(v2_z, v2_y)

    Dy = y_eq - P_J2[1]
    Dz = z_safe - P_J2[2]
    D = math.hypot(Dy, Dz)
    cos_alpha = (D * D - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if abs(cos_alpha) > 1.0:
        return None

    best = None
    for sgn in (-1.0, 1.0):
        alpha = sgn * math.acos(cos_alpha)
        j3 = alpha - (phi2 - phi1)
        theta_D = math.atan2(Dz, Dy)
        beta = math.atan2(L2 * math.sin(alpha), L1 + L2 * math.cos(alpha))
        j2 = theta_D - phi1 - beta
        j1_d, j2_d, j3_d = math.degrees(j1), math.degrees(j2), math.degrees(j3)
        if J1_MIN_DEG <= j1_d <= J1_MAX_DEG and J2_MIN_DEG <= j2_d <= J2_MAX_DEG and J3_MIN_DEG <= j3_d <= J3_MAX_DEG:
            _, cj2, cj3, _ = clamp_safe_workspace(j1_d, j2_d, j3_d, 0.0)
            score = j2_d
            if best is None or score < best[0]:
                best = (score, j1_d, cj2, cj3)
    return best[1:] if best else None


def reset_ball(model: mujoco.MjModel, data: mujoco.MjData, pos=BALL_SPAWN_POS):
    """Reposiciona a bolinha suavemente no chão com velocidade zero."""
    ball_jnt_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "bolinha_free")
    if ball_jnt_id >= 0:
        qpos_adr = model.jnt_qposadr[ball_jnt_id]
        qvel_adr = model.jnt_dofadr[ball_jnt_id]
        data.qpos[qpos_adr : qpos_adr + 3] = pos
        data.qpos[qpos_adr + 3 : qpos_adr + 7] = [1.0, 0.0, 0.0, 0.0]
        data.qvel[qvel_adr : qvel_adr + 6] = 0.0
        mujoco.mj_forward(model, data)


class ESP32SerialBridge:
    """Ponte híbrida (TCP Socket para Wokwi localhost:4000 / PySerial para COMx USB) entre MuJoCo e ESP32."""

    def __init__(self, port: str = "", baud: int = 115200):
        self.port = port
        self.baud = baud
        self.ser = None
        self.sock = None
        self.last_send_time = 0.0
        self.last_auto_probe = 0.0
        self.auto_wokwi = True
        self.rx_buffer = ""
        self.last_rx_line = ""
        self.status_msg = "Aguardando Wokwi (localhost:4000)..."
        if port:
            self.connect(port)
        else:
            self.try_auto_connect_wokwi()

    @property
    def is_connected(self) -> bool:
        if self.sock is not None:
            return True
        return self.ser is not None and self.ser.is_open

    def try_auto_connect_wokwi(self) -> bool:
        """Se desconectado e o Wokwi estiver rodando no VS Code (porta TCP 4000 em IPv6 ::1 ou IPv4), conecta direto!"""
        if self.is_connected or not self.auto_wokwi:
            return self.is_connected
        now = time.time()
        if (now - self.last_auto_probe) < 1.0:
            return False
        self.last_auto_probe = now
        return self.connect("wokwi", quiet=True)

    def connect(self, port: str, quiet: bool = False) -> bool:
        self.disconnect()
        target = port.strip()
        if not target or target.lower() in ("wokwi", "localhost:4000", "4000", "rfc2217://localhost:4000"):
            try:
                sock = socket.create_connection(("localhost", 4000), timeout=0.15)
                sock.sendall(b"\n")
                sock.setblocking(False)
                self.sock = sock
                self.rx_buffer = ""
                self.port = "wokwi"
                self.status_msg = "Conectado: Wokwi (localhost:4000)"
                print("[ESP32 Wokwi] Conectado com sucesso em localhost:4000!", flush=True)
                return True
            except Exception as e:
                self.sock = None
                self.status_msg = "Wokwi (porta 4000) aguardando Play..."
                if not quiet:
                    print(f"[ESP32 Wokwi] Porta localhost:4000 ainda fechada ({e})", flush=True)
                return False

        if serial is None:
            self.status_msg = "pyserial não instalado"
            return False
        try:
            self.ser = serial.Serial(target, baudrate=self.baud, timeout=0.0, write_timeout=0.02)
            self.port = target
            self.status_msg = f"Conectado: {target}"
            print(f"[ESP32 Serial] Conectado em {target} ({self.baud} baud)", flush=True)
            return True
        except Exception as e:
            self.ser = None
            self.status_msg = f"Sem conexão ({target})"
            if not quiet:
                print(f"[ESP32 Serial] Falha ao abrir {target}: {e}", flush=True)
            return False

    def disconnect(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.status_msg = "Desconectado"

    def poll_incoming(self):
        """Lê respostas enviadas pelo ESP32 via Socket (Wokwi) ou Serial USB e imprime no terminal."""
        if not self.is_connected:
            return
        try:
            raw = ""
            if self.sock is not None:
                try:
                    data = self.sock.recv(4096)
                    if data == b"":
                        # Conexão fechada pelo Wokwi (ex: parou a simulação)
                        self.disconnect()
                        return
                    raw = data.decode("utf-8", errors="replace")
                except BlockingIOError:
                    return
            elif self.ser is not None:
                waiting = self.ser.in_waiting
                if waiting > 0:
                    raw = self.ser.read(waiting).decode("utf-8", errors="replace")
            if not raw:
                return
            self.rx_buffer += raw
            while "\n" in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split("\n", 1)
                line_s = line.strip()
                if line_s:
                    self.last_rx_line = line_s
                    print(f"[ESP32 -> PC] {line_s}", flush=True)
        except Exception:
            self.disconnect()

    def send_command(self, line: str):
        if not self.is_connected:
            self.try_auto_connect_wokwi()
        if not self.is_connected:
            return
        try:
            if not line.endswith("\n"):
                line += "\n"
            payload = line.encode("ascii", errors="ignore")
            if self.sock is not None:
                self.sock.sendall(payload)
            elif self.ser is not None:
                self.ser.write(payload)
        except Exception as e:
            self.status_msg = f"Erro envio: {e}"
            self.disconnect()

    def stream_joints_deg(self, degs):
        """Envia S:j1,j2,j3,j4 em até 30 Hz (a cada 33 ms) para o ESP32 / Wokwi."""
        if not self.is_connected:
            self.try_auto_connect_wokwi()
            if not self.is_connected:
                return
        self.poll_incoming()
        now = time.time()
        if (now - self.last_send_time) < 0.033:
            return
        self.last_send_time = now
        cmd = f"S:{degs[0]:.1f},{degs[1]:.1f},{degs[2]:.1f},{degs[3]:.1f}\n"
        self.send_command(cmd)


class PickAndDropLoopController:
    """Máquina de estados para o desafio de pegar a bolinha e soltar no chão em loop."""

    DROP_ZONES_J1 = [40.0, -40.0, 0.0]  # Esquerda (+40°), Direita (-40°), Centro (0°)

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData):
        self.model = model
        self.data = data
        self.ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bolinha")
        self.cycle_count = 0
        self.zone_idx = 0
        self.phase_idx = 0
        self.phase_time = 0.0
        self.phase_duration = 0.8
        self.start_ctrl_deg = np.zeros(4)
        self.target_ctrl_deg = np.zeros(4)
        self.sol_hover = (0.0, -52.0, 21.0)
        self.sol_grasp = (0.0, -73.0, 27.0)
        self.drop_j1 = 40.0
        self.status_text = "Pronto"

    def start(self):
        self.phase_idx = 0
        self._prepare_next_cycle()

    def _current_ctrl_deg(self) -> np.ndarray:
        return np.degrees(self.data.ctrl[:4].copy())

    def _ensure_ball_reachable(self):
        bx, by, bz = self.data.xpos[self.ball_id]
        sol_h = solve_ik_3d(bx, by, 0.072)
        sol_g = solve_ik_3d(bx, by, 0.0442)
        if sol_h is None or sol_g is None or bz < Z_FLOOR - 0.01:
            reset_ball(self.model, self.data, BALL_SPAWN_POS)
            bx, by, bz = self.data.xpos[self.ball_id]
            sol_h = solve_ik_3d(bx, by, 0.072)
            sol_g = solve_ik_3d(bx, by, 0.0442)
        self.sol_hover = sol_h
        self.sol_grasp = sol_g

    def _prepare_next_cycle(self):
        self.cycle_count += 1
        self._ensure_ball_reachable()
        cur_j1 = self.sol_grasp[0]
        candidate = self.DROP_ZONES_J1[self.zone_idx % len(self.DROP_ZONES_J1)]
        if abs(candidate - cur_j1) < 15.0:
            self.zone_idx += 1
            candidate = self.DROP_ZONES_J1[self.zone_idx % len(self.DROP_ZONES_J1)]
        self.drop_j1 = candidate
        self.zone_idx += 1
        self._enter_phase(1)

    def _enter_phase(self, phase: int):
        self.phase_idx = phase
        self.phase_time = 0.0
        self.start_ctrl_deg = self._current_ctrl_deg()

        if phase == 1:
            self._ensure_ball_reachable()
            j1, j2, j3 = self.sol_hover
            self.target_ctrl_deg = np.array([j1, j2, j3, 32.0])
            self.phase_duration = 0.90
            self.status_text = f"Ciclo #{self.cycle_count} · 1/6 Aproximando por cima (garra aberta)"
        elif phase == 2:
            self._ensure_ball_reachable()
            j1, j2, j3 = self.sol_grasp
            self.target_ctrl_deg = np.array([j1, j2, j3, 32.0])
            self.phase_duration = 0.75
            self.status_text = f"Ciclo #{self.cycle_count} · 2/6 Descendo até a bolinha"
        elif phase == 3:
            j1, j2, j3 = self.sol_grasp
            self.target_ctrl_deg = np.array([j1, j2, j3, 1.0])
            self.phase_duration = 0.65
            self.status_text = f"Ciclo #{self.cycle_count} · 3/6 Fechando a garra (pegando bolinha)"
        elif phase == 4:
            j1 = self.sol_grasp[0]
            self.target_ctrl_deg = np.array([j1, -15.0, 10.0, 1.0])
            self.phase_duration = 0.95
            self.status_text = f"Ciclo #{self.cycle_count} · 4/6 Levantando a bolinha"
        elif phase == 5:
            self.target_ctrl_deg = np.array([self.drop_j1, -45.0, 22.0, 1.0])
            self.phase_duration = 1.05
            self.status_text = f"Ciclo #{self.cycle_count} · 5/6 Levando para zona ({self.drop_j1:+.0f}°)"
        elif phase == 6:
            self.target_ctrl_deg = np.array([self.drop_j1, -45.0, 22.0, 32.0])
            self.phase_duration = 0.50
            self.status_text = f"Ciclo #{self.cycle_count} · 6/6 Soltando a bolinha no chão!"
        elif phase == 7:
            self.target_ctrl_deg = np.array([self.drop_j1, -30.0, 15.0, 32.0])
            self.phase_duration = 0.65
            self.status_text = f"Ciclo #{self.cycle_count} · Bolinha quicando no chão (próximo ciclo...)"

    def step(self, dt: float) -> np.ndarray:
        if self.phase_idx == 0:
            self.start()

        self.phase_time += dt
        u = min(1.0, self.phase_time / max(1e-4, self.phase_duration))
        alpha = 0.5 * (1.0 - math.cos(math.pi * u))
        cmd_deg = self.start_ctrl_deg + alpha * (self.target_ctrl_deg - self.start_ctrl_deg)

        if u >= 1.0:
            if self.phase_idx < 7:
                self._enter_phase(self.phase_idx + 1)
            else:
                self._prepare_next_cycle()

        return cmd_deg


class SimuladorState:
    """Estado compartilhado entre o loop físico MuJoCo, ponte Serial ESP32 e painel GUI."""

    def __init__(self, model: mujoco.MjModel, data: mujoco.MjData, start_desafio: bool = False, serial_port: str = ""):
        self.model = model
        self.data = data
        self.ball_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "bolinha")
        self.auto_loop = start_desafio
        self.max_deg_per_sec = 42.0
        self.pwm_freq_hz = PCA9685_FREQ_HZ
        # Começa na Pose Zero exata do view_mockup.py: [0°, 0°, 0°, 0°] (todos os servos em 1500 us / tick 307)
        self.manual_target_deg = np.array([0.0, 0.0, 0.0, 0.0], dtype=float)
        self.smoothed_deg = self.manual_target_deg.copy()
        self.last_written_ctrl = np.radians(self.smoothed_deg)
        self.data.ctrl[:4] = self.last_written_ctrl.copy()
        self.loop_ctrl = PickAndDropLoopController(model, data)
        self.esp32 = ESP32SerialBridge(port=serial_port)
        if start_desafio:
            self.loop_ctrl.start()
        self.running = True
        self.gui_dirty = True

    def set_mode(self, auto_loop: bool):
        if self.auto_loop == auto_loop:
            return
        self.auto_loop = auto_loop
        if auto_loop:
            self.loop_ctrl.start()
        else:
            cur_deg = np.degrees(self.data.ctrl[:4].copy())
            self.manual_target_deg[:] = cur_deg
            self.smoothed_deg[:] = cur_deg
            self.gui_dirty = True

    def move_to_ball_ik(self):
        """Posiciona os alvos manuais logo acima da bolinha usando Cinemática Inversa."""
        self.set_mode(False)
        bx, by, _ = self.data.xpos[self.ball_id]
        sol = solve_ik_3d(bx, by, 0.0415)
        if sol is None:
            reset_ball(self.model, self.data)
            bx, by, _ = self.data.xpos[self.ball_id]
            sol = solve_ik_3d(bx, by, 0.0415)
        if sol is not None:
            self.manual_target_deg[:] = [sol[0], sol[1], sol[2], 30.0]
            self.gui_dirty = True

    def reset_zero_mockup(self):
        """Coloca todas as 4 juntas exatamente em 0.0° (1500 us / tick 307) = Pose do view_mockup.py."""
        self.set_mode(False)
        self.manual_target_deg[:] = [0.0, 0.0, 0.0, 0.0]
        self.esp32.send_command("ZERO")
        self.gui_dirty = True

    def step_controller(self, dt: float):
        if self.auto_loop:
            cmd_deg = np.array(clamp_safe_workspace(*self.loop_ctrl.step(dt)), dtype=float)
            self.manual_target_deg[:] = cmd_deg
            self.smoothed_deg[:] = cmd_deg
            self.last_written_ctrl = np.radians(cmd_deg)
            self.data.ctrl[:4] = self.last_written_ctrl
            self.gui_dirty = True
        else:
            diff_viewer = np.abs(self.data.ctrl[:4] - self.last_written_ctrl)
            if np.any(diff_viewer > 1e-4):
                viewer_deg = np.degrees(self.data.ctrl[:4])
                for i in range(4):
                    if diff_viewer[i] > 1e-4:
                        self.manual_target_deg[i] = viewer_deg[i]
                self.gui_dirty = True

            clamped_target = np.array(clamp_safe_workspace(*self.manual_target_deg), dtype=float)
            if np.any(np.abs(clamped_target - self.manual_target_deg) > 1e-3):
                self.gui_dirty = True
            self.manual_target_deg[:] = clamped_target

            max_step = self.max_deg_per_sec * dt
            for i in range(4):
                err = self.manual_target_deg[i] - self.smoothed_deg[i]
                if abs(err) <= max_step:
                    self.smoothed_deg[i] = self.manual_target_deg[i]
                else:
                    self.smoothed_deg[i] += math.copysign(max_step, err)

            # Garante também na trajetória suavizada a proteção acoplada J3 >= j3_min_safe_for_j2(J2)
            self.smoothed_deg[:] = clamp_safe_workspace(*self.smoothed_deg)
            self.last_written_ctrl = np.radians(self.smoothed_deg)
            self.data.ctrl[:4] = self.last_written_ctrl

        # Envia os ângulos atuais (em graus, já limitados contra colisão) para o ESP32 via Serial/Wokwi
        self.esp32.stream_joints_deg(self.smoothed_deg)


def create_tk_panel(state: SimuladorState):
    """Cria o painel gráfico Tkinter com sliders, conversão PCA9685 (us/tick) e ponte Serial ESP32."""
    import tkinter as tk
    from tkinter import ttk

    root = tk.Tk()
    root.title("Gêmeo Digital MG90 + PCA9685 · robot_arm")
    root.geometry("500x660")
    root.configure(bg="#111827")
    root.resizable(False, False)

    style = ttk.Style(root)
    style.theme_use("clam")

    # Cabeçalho
    hdr = tk.Frame(root, bg="#1f2937", padx=14, pady=10)
    hdr.pack(fill="x")
    tk.Label(
        hdr,
        text="🦾 Gêmeo Digital 4-DOF (ESP32 + PCA9685 + MG90)",
        font=("Segoe UI", 11, "bold"),
        fg="#38bdf8",
        bg="#1f2937",
    ).pack(anchor="w")
    status_lbl = tk.Label(
        hdr,
        text="Pose Zero (0.0° = 1500 us / tick 307 · idêntica ao view_mockup.py)",
        font=("Segoe UI", 9),
        fg="#e5e7eb",
        bg="#1f2937",
    )
    status_lbl.pack(anchor="w", pady=(2, 0))

    # Barra de conexão Serial ESP32 / Wokwi
    ser_frame = tk.Frame(root, bg="#1e293b", padx=14, pady=8)
    ser_frame.pack(fill="x", pady=(2, 0))
    tk.Label(ser_frame, text="🔌 Porta ESP32:", font=("Segoe UI", 9, "bold"), fg="#93c5fd", bg="#1e293b").pack(
        side="left"
    )
    port_var = tk.StringVar(value="wokwi" if not state.esp32.port or "4000" in state.esp32.port else state.esp32.port)
    port_entry = tk.Entry(
        ser_frame, textvariable=port_var, width=8, font=("Consolas", 9, "bold"), bg="#0f172a", fg="white", insertbackground="white"
    )
    port_entry.pack(side="left", padx=6)

    ser_status_lbl = tk.Label(
        ser_frame, text=state.esp32.status_msg, font=("Consolas", 8), fg="#9ca3af", bg="#1e293b"
    )

    def toggle_serial():
        if state.esp32.is_connected:
            state.esp32.auto_wokwi = False
            state.esp32.disconnect()
        else:
            state.esp32.auto_wokwi = True
            state.esp32.connect(port_var.get())
        ser_status_lbl.config(
            text=state.esp32.status_msg,
            fg="#4ade80" if state.esp32.is_connected else "#f87171",
        )

    tk.Button(
        ser_frame,
        text="Conectar / Desconectar",
        font=("Segoe UI", 8, "bold"),
        bg="#2563eb",
        fg="white",
        relief="flat",
        padx=8,
        pady=2,
        command=toggle_serial,
    ).pack(side="left", padx=4)
    ser_status_lbl.pack(side="left", padx=6)

    # Barra de envio de comando Serial manual (substitui o Serial Monitor quando rfc2217 está ativo)
    cmd_frame = tk.Frame(root, bg="#0f172a", padx=14, pady=6)
    cmd_frame.pack(fill="x")
    tk.Label(cmd_frame, text="⌨️ Cmd Serial:", font=("Segoe UI", 8, "bold"), fg="#38bdf8", bg="#0f172a").pack(
        side="left"
    )
    cmd_var = tk.StringVar(value="STATUS")
    cmd_entry = tk.Entry(
        cmd_frame, textvariable=cmd_var, width=22, font=("Consolas", 9), bg="#1e293b", fg="white", insertbackground="white"
    )
    cmd_entry.pack(side="left", padx=6)

    def send_manual_serial_cmd(event=None):
        txt = cmd_var.get().strip()
        if not txt:
            return
        state.esp32.send_command(txt)
        # Se digitou 4 ângulos ex: 0,-45,20,25, também move o MuJoCo!
        parts = [p.strip() for p in txt.replace(";", ",").split(",") if p.strip()]
        if len(parts) == 4:
            try:
                vals = [float(p) for p in parts]
                state.set_mode(False)
                state.manual_target_deg[:] = vals
                state.gui_dirty = True
            except Exception:
                pass
        elif txt.upper() in ("ZERO", "0", "RESET"):
            state.reset_zero_mockup()

    cmd_entry.bind("<Return>", send_manual_serial_cmd)
    tk.Button(
        cmd_frame,
        text="Enviar",
        font=("Segoe UI", 8, "bold"),
        bg="#0284c7",
        fg="white",
        relief="flat",
        padx=8,
        pady=1,
        command=send_manual_serial_cmd,
    ).pack(side="left", padx=2)
    tk.Button(
        cmd_frame,
        text="STATUS",
        font=("Consolas", 8),
        bg="#334155",
        fg="white",
        relief="flat",
        padx=6,
        pady=1,
        command=lambda: state.esp32.send_command("STATUS"),
    ).pack(side="left", padx=2)

    # Botão principal de alternância Manual <-> Desafio Loop Bolinha
    btn_frame = tk.Frame(root, bg="#111827", padx=14, pady=8)
    btn_frame.pack(fill="x")

    mode_btn = tk.Button(
        btn_frame,
        text="🔄 Iniciar Desafio: Pegar e Soltar Bolinha (Loop)",
        font=("Segoe UI", 10, "bold"),
        bg="#0284c7",
        fg="white",
        activebackground="#0369a1",
        activeforeground="white",
        relief="flat",
        pady=7,
        cursor="hand2",
        command=lambda: state.set_mode(not state.auto_loop),
    )
    mode_btn.pack(fill="x")

    sub_btns = tk.Frame(btn_frame, bg="#111827", pady=6)
    sub_btns.pack(fill="x")
    tk.Button(
        sub_btns,
        text="🎯 Calibrar ZERO (0° = view_mockup)",
        font=("Segoe UI", 8, "bold"),
        bg="#059669",
        fg="white",
        relief="flat",
        padx=6,
        pady=4,
        command=state.reset_zero_mockup,
    ).pack(side="left", expand=True, fill="x", padx=(0, 3))
    tk.Button(
        sub_btns,
        text="🏀 Reset Bolinha (R)",
        font=("Segoe UI", 8),
        bg="#374151",
        fg="white",
        relief="flat",
        padx=6,
        pady=4,
        command=lambda: reset_ball(state.model, state.data),
    ).pack(side="left", expand=True, fill="x", padx=3)
    tk.Button(
        sub_btns,
        text="📍 Ir até Bolinha (IK)",
        font=("Segoe UI", 8),
        bg="#374151",
        fg="white",
        relief="flat",
        padx=6,
        pady=4,
        command=state.move_to_ball_ik,
    ).pack(side="left", expand=True, fill="x", padx=(3, 0))

    # Sliders dos 4 servos MG90 + Telemetria PCA9685 (us e tick)
    sliders_frame = tk.Frame(root, bg="#111827", padx=14, pady=2)
    sliders_frame.pack(fill="x")

    joint_specs = [
        (0, "CH0 · J1 Base (Z) [-90°..+90°]", J1_MIN_DEG, J1_MAX_DEG, "#38bdf8"),
        (1, "CH1 · J2 Ombro (X) [-72°..+60°]", J2_MIN_DEG, J2_MAX_DEG, "#22d3ee"),
        (2, "CH2 · J3 Cotovelo (X) [-35°..+65°]", J3_MIN_DEG, J3_MAX_DEG, "#818cf8"),
        (3, "CH3 · J4 Garra [0°=Fechada..+38°]", GARRA_MIN_DEG, GARRA_MAX_DEG, "#34d399"),
    ]

    slider_vars = []
    val_labels = []
    updating_gui = {"lock": False}

    def on_slider_change(idx, val_str):
        if updating_gui["lock"]:
            return
        val = float(val_str)
        if state.auto_loop:
            state.set_mode(False)
        state.manual_target_deg[idx] = val
        state.manual_target_deg[:] = clamp_safe_workspace(*state.manual_target_deg)
        state.gui_dirty = True

    def nudge_joint(idx, delta_deg):
        if state.auto_loop:
            state.set_mode(False)
        lo = JOINT_MIN_DEGS[idx]
        hi = JOINT_MAX_DEGS[idx]
        new_deg = max(lo, min(hi, state.manual_target_deg[idx] + delta_deg))
        state.manual_target_deg[idx] = new_deg
        state.manual_target_deg[:] = clamp_safe_workspace(*state.manual_target_deg)
        state.gui_dirty = True

    for idx, title, lo, hi, color in joint_specs:
        row = tk.Frame(sliders_frame, bg="#1f2937", padx=10, pady=5)
        row.pack(fill="x", pady=3)

        top = tk.Frame(row, bg="#1f2937")
        top.pack(fill="x")
        tk.Label(top, text=title, font=("Segoe UI", 9, "bold"), fg=color, bg="#1f2937").pack(side="left")
        lbl_val = tk.Label(
            top, text="0.0° | 1500 us | tick 307", font=("Consolas", 8, "bold"), fg="#f9fafb", bg="#1f2937"
        )
        lbl_val.pack(side="right")
        val_labels.append(lbl_val)

        ctrl_row = tk.Frame(row, bg="#1f2937")
        ctrl_row.pack(fill="x", pady=(2, 0))

        tk.Button(
            ctrl_row,
            text="-2°",
            font=("Consolas", 8, "bold"),
            bg="#374151",
            fg="white",
            relief="flat",
            width=4,
            command=lambda i=idx: nudge_joint(i, -2.0),
        ).pack(side="left", padx=(0, 6))

        var = tk.DoubleVar(value=0.0)
        slider_vars.append(var)
        sc = tk.Scale(
            ctrl_row,
            from_=lo,
            to=hi,
            resolution=0.5,
            orient="horizontal",
            variable=var,
            showvalue=False,
            bg="#1f2937",
            fg="white",
            troughcolor="#374151",
            highlightthickness=0,
            bd=0,
            command=lambda v, i=idx: on_slider_change(i, v),
        )
        sc.pack(side="left", fill="x", expand=True)

        tk.Button(
            ctrl_row,
            text="+2°",
            font=("Consolas", 8, "bold"),
            bg="#374151",
            fg="white",
            relief="flat",
            width=4,
            command=lambda i=idx: nudge_joint(i, +2.0),
        ).pack(side="left", padx=(6, 0))

    # Controle de Sensibilidade / Velocidade dos Servos
    sens_frame = tk.Frame(root, bg="#1f2937", padx=10, pady=7)
    sens_frame.pack(fill="x", padx=14, pady=(6, 4))
    sens_top = tk.Frame(sens_frame, bg="#1f2937")
    sens_top.pack(fill="x")
    tk.Label(
        sens_top,
        text="⚙️ Sensibilidade / Velocidade Manual dos Servos MG90",
        font=("Segoe UI", 9, "bold"),
        fg="#fbbf24",
        bg="#1f2937",
    ).pack(side="left")
    sens_lbl = tk.Label(
        sens_top,
        text=f"{state.max_deg_per_sec:.0f} °/s (Suave)",
        font=("Consolas", 9),
        fg="#f9fafb",
        bg="#1f2937",
    )
    sens_lbl.pack(side="right")

    def on_sens_change(val_str):
        v = float(val_str)
        state.max_deg_per_sec = v
        tag = "Ultra Fino" if v < 25 else ("Suave" if v <= 55 else "Rápido")
        sens_lbl.config(text=f"{v:.0f} °/s ({tag})")

    sens_scale = tk.Scale(
        sens_frame,
        from_=10.0,
        to=120.0,
        resolution=2.0,
        orient="horizontal",
        showvalue=False,
        bg="#1f2937",
        fg="white",
        troughcolor="#374151",
        highlightthickness=0,
        bd=0,
        command=on_sens_change,
    )
    sens_scale.set(state.max_deg_per_sec)
    sens_scale.pack(fill="x", pady=(3, 0))

    def sync_gui():
        if not state.running:
            return
        updating_gui["lock"] = True
        try:
            if state.auto_loop:
                mode_btn.config(
                    text="⏸️ Pausar Desafio e Assumir Controle Manual",
                    bg="#dc2626",
                    activebackground="#b91c1c",
                )
                status_lbl.config(text=state.loop_ctrl.status_text, fg="#fde047")
            else:
                mode_btn.config(
                    text="🔄 Iniciar Desafio: Pegar e Soltar Bolinha (Loop)",
                    bg="#0284c7",
                    activebackground="#0369a1",
                )
                bx, by, bz = state.data.xpos[state.ball_id]
                status_lbl.config(
                    text=f"Modo Manual · PCA9685 @ {state.pwm_freq_hz:.0f}Hz · Bolinha X={bx*1000:.0f} Y={by*1000:.0f} mm",
                    fg="#e5e7eb",
                )

            ser_status_lbl.config(
                text=state.esp32.status_msg,
                fg="#4ade80" if state.esp32.is_connected else "#fbbf24",
            )
            for i in range(4):
                deg = state.smoothed_deg[i]
                pulse_us, tick = deg_to_pca9685(deg, state.pwm_freq_hz)
                if state.gui_dirty or state.auto_loop:
                    slider_vars[i].set(round(state.manual_target_deg[i], 1))
                val_labels[i].config(text=f"{deg:+5.1f}° | {pulse_us:4.0f} us | tick {tick:3d}")
            state.gui_dirty = False
        finally:
            updating_gui["lock"] = False

    def on_close():
        state.running = False

    root.protocol("WM_DELETE_WINDOW", on_close)
    return root, sync_gui


def main():
    parser = argparse.ArgumentParser(description="Simulador e Gêmeo Digital do Braço Robótico 4-DOF (MG90 + PCA9685)")
    parser.add_argument("--desafio", action="store_true", help="Inicia diretamente no modo Loop de pegar e soltar a bolinha")
    parser.add_argument("--port", type=str, default="", help="Porta Serial do ESP32 (ex: COM3)")
    parser.add_argument("--no-gui", action="store_true", help="Executa sem o painel lateral Tkinter (apenas MuJoCo Viewer)")
    parser.add_argument("--duration", type=float, default=0.0, help="Tempo máximo em segundos (0 = ilimitado)")
    parser.add_argument("xml", nargs="?", default=XML_DEFAULT, help="Caminho para o modelo MJCF (padrão: models/arm.xml)")
    args = parser.parse_args()

    xml_path = os.path.abspath(args.xml)
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)

    state = SimuladorState(model, data, start_desafio=args.desafio, serial_port=args.port)

    STEP_KEY_DEG = 2.5

    def key_callback(keycode: int):
        if keycode in (ord("L"), ord("l")):
            state.set_mode(not state.auto_loop)
            print(f"[Simulador] Modo Desafio Loop: {'ATIVADO' if state.auto_loop else 'MANUAL'}")
        elif keycode in (ord("R"), ord("r")):
            reset_ball(model, data)
            print("[Simulador] Bolinha reposicionada na zona central.")
        elif keycode in (ord("G"), ord("g")):
            state.move_to_ball_ik()
            print("[Simulador] Braco posicionado sobre a bolinha (Cinematica Inversa).")
        elif keycode in (ord("0"), ord("Z"), ord("z"), ord("N"), ord("n")):
            state.reset_zero_mockup()
            print("[Simulador] Pose Zero de Calibracao (0.0 deg = 1500 us / tick 307 = view_mockup.py).")
        elif keycode == ord("1"):
            state.set_mode(False)
            state.manual_target_deg[0] -= STEP_KEY_DEG
            state.gui_dirty = True
        elif keycode == ord("2"):
            state.set_mode(False)
            state.manual_target_deg[0] += STEP_KEY_DEG
            state.gui_dirty = True
        elif keycode == ord("3"):
            state.set_mode(False)
            state.manual_target_deg[1] -= STEP_KEY_DEG
            state.gui_dirty = True
        elif keycode == ord("4"):
            state.set_mode(False)
            state.manual_target_deg[1] += STEP_KEY_DEG
            state.gui_dirty = True
        elif keycode == ord("5"):
            state.set_mode(False)
            state.manual_target_deg[2] -= STEP_KEY_DEG
            state.gui_dirty = True
        elif keycode == ord("6"):
            state.set_mode(False)
            state.manual_target_deg[2] += STEP_KEY_DEG
            state.gui_dirty = True
        elif keycode == ord("7"):
            state.set_mode(False)
            state.manual_target_deg[3] -= 2.0
            state.gui_dirty = True
        elif keycode == ord("8"):
            state.set_mode(False)
            state.manual_target_deg[3] += 2.0
            state.gui_dirty = True

    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:
        pass

    print("=" * 72)
    print("GEMEO DIGITAL BRACO ROBOTICO 4-DOF (ESP32 + PCA9685 + Servos MG90)")
    print("=" * 72)
    print("Pose Zero (0.0 graus = 1500 us = tick 307 @ 50Hz): Identica ao view_mockup.py")
    print("Atalhos na janela 3D:")
    print("  [Z / 0] Calibrar todas as juntas em 0.0 graus (Pose view_mockup.py)")
    print("  [L]     Alternar Modo Desafio (Loop Pegar/Soltar Bolinha) <-> Modo Manual")
    print("  [R]     Reposicionar a bolinha no centro")
    print("  [G]     Ir ate a bolinha automaticamente (Cinematica Inversa)")
    print("  [1/2]   J1 Base | [3/4] J2 Ombro | [5/6] J3 Cotovelo | [7/8] J4 Garra")
    print("=" * 72, flush=True)

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        viewer.cam.lookat[:] = [0.025, 0.095, 0.075]
        viewer.cam.distance = 0.29
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -24
        viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = 0
        viewer.opt.geomgroup[4] = 0
        viewer.opt.sitegroup[4] = 0

        tk_root = None
        sync_gui = None
        if not args.no_gui:
            try:
                tk_root, sync_gui = create_tk_panel(state)
            except Exception as e:
                print(f"[Aviso] Painel Tkinter indisponivel ({e}); usando apenas a janela 3D do MuJoCo.")

        t_start = time.time()
        last_wall = t_start
        dt_phys = model.opt.timestep

        while viewer.is_running() and state.running:
            now = time.time()
            if args.duration > 0 and (now - t_start) >= args.duration:
                break

            elapsed = min(0.05, now - last_wall)
            last_wall = now
            steps = max(1, int(round(elapsed / dt_phys)))

            for _ in range(steps):
                state.step_controller(dt_phys)
                mujoco.mj_step(model, data)

            viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_INERTIA] = 0
            viewer.opt.geomgroup[4] = 0
            viewer.opt.sitegroup[4] = 0
            viewer.sync()

            if tk_root is not None and sync_gui is not None:
                try:
                    sync_gui()
                    tk_root.update_idletasks()
                    tk_root.update()
                except Exception:
                    state.running = False
                    break

            time.sleep(0.012)

        state.esp32.disconnect()
        if tk_root is not None:
            try:
                tk_root.destroy()
            except Exception:
                pass
        sys.stdout.flush()
        os._exit(0)


if __name__ == "__main__":
    main()
