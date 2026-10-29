import os
import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import mujoco

"""
Script de Treinamento de Rede Neural para Cinemática Inversa (IK)
- Entrada: Posição cartesiana do efetuador (x, y, z)
- Saída: Ângulos das juntas (theta_base, theta_ombro, theta_cotovelo)
"""

class InverseKinematicsMLP(nn.Module):
    def __init__(self):
        super(InverseKinematicsMLP, self).__init__()
        self.net = nn.Sequential(
            nn.Linear(3, 128),
            nn.ReLU(),
            nn.Linear(128, 128),
            nn.ReLU(),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, 3)
        )

    def forward(self, x):
        return self.net(x)

def generate_dataset(model, data, num_samples=10000):
    print(f"Gerando {num_samples} amostras sintéticas de cinemática direta no MuJoCo...")
    X = [] # Posição 3D (x, y, z)
    Y = [] # Ângulos (graus)

    for _ in range(num_samples):
        # Sorteia ângulos dentro dos limites físicos dos servos
        q_base = np.random.uniform(-80, 80)
        q_ombro = np.random.uniform(-30, 75)
        q_cotovelo = np.random.uniform(-70, 45)

        data.qpos[model.joint('joint_base').qposadr] = np.deg2rad(q_base)
        data.qpos[model.joint('joint_ombro').qposadr] = np.deg2rad(q_ombro)
        data.qpos[model.joint('joint_cotovelo').qposadr] = np.deg2rad(q_cotovelo)

        # Atualiza a cinemática direta do robô
        mujoco.mj_forward(model, data)

        tcp_pos = data.site('gripper_tcp').xpos.copy()

        X.append(tcp_pos)
        Y.append([q_base, q_ombro, q_cotovelo])

    return np.array(X, dtype=np.float32), np.array(Y, dtype=np.float32)

def train():
    xml_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "model.xml")
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    X_train, Y_train = generate_dataset(model, data, num_samples=15000)

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    print(f"Dispositivo de treinamento: {device}")

    ik_model = InverseKinematicsMLP().to(device)
    optimizer = optim.Adam(ik_model.parameters(), lr=1e-3)
    criterion = nn.MSELoss()

    inputs = torch.from_numpy(X_train).to(device)
    targets = torch.from_numpy(Y_train).to(device)

    print("Iniciando treinamento da Rede Neural...")
    for epoch in range(1, 301):
        optimizer.zero_grad()
        predictions = ik_model(inputs)
        loss = criterion(predictions, targets)
        loss.backward()
        optimizer.step()

        if epoch % 50 == 0:
            print(f"Epoch {epoch}/300 - Perda (MSE): {loss.item():.4f}")

    # Salva pesos treinados
    save_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ik_model.pth")
    torch.save(ik_model.state_dict(), save_path)
    print(f"Modelo de IA salvo em: {save_path}")

if __name__ == "__main__":
    train()
