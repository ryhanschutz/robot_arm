# robot_arm — Gêmeo digital (MuJoCo)

Braço robótico 4-DOF (base, ombro, cotovelo, garra) com simulação em MuJoCo,
preparada para receber dados reais do robô físico (ESP32).

## Instalação
```powershell
pip install -r requirements.txt
```

## Rodar
```powershell
python sim/animar_robo.py                 # viewer animando todas as juntas (modelo principal)
python sim/view_mockup.py                 # montagem estática (teste_montagem.xml)
python sim/view_mockup.py models/arm.xml  # modelo principal com painel de sliders dos atuadores
```
Ambos usam o OpenGL por software (Mesa) de `sim/x64/` quando presente.

## Estrutura
| Pasta | Conteúdo |
|---|---|
| `models/arm.xml` | **Modelo principal** (MJCF): 4 juntas, atuadores e engrenagem da garra. É o que `animar_robo.py` carrega. |
| `models/experiments/` | XMLs de teste (montagem estática, calibração da junta 1) |
| `meshes/` | Somente os 7 STLs referenciados por `arm.xml` (peças com servo, mm → m) |
| `sim/` | `animar_robo.py`, `view_mockup.py`, `x64/` (Mesa) |
| `sim/experiments/` | Scripts auxiliares (render de poses, inspeção de STL) |
| `prints/robot/`, `prints/case/` | STLs para impressão 3D (sem servo) e caixa da eletrônica |
| `cad/` | Modelos sólidos SolidWorks (somente leitura) |
| `firmware/` | Firmware ESP32 (PlatformIO) |
| `docs/` | Documentação, auditoria e imagens de referência |
| `_trash/` | Itens removidos (fora do git), com `MANIFEST.md` |

O simulador antigo (`main.py`, `model.xml`) foi descartado e está em `_trash/`.
