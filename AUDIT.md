# Relatório de Auditoria — Gêmeo Digital do Braço Robótico

**Repositório:** `robot_arm`  
**Branch de Trabalho:** `cleanup-digital-twin` (criada a partir do estado inicial)  
**Data da Auditoria:** 29/10/2026  
**Status:** Etapa 1 Concluída — Aguardando aprovação para Etapa 2  

---

## 1. Resumo Executivo e Mapeamento da Árvore

O repositório contém os elementos para um braço robótico de 4 Graus de Liberdade (4-DOF), acionado por servomotores (estilo micro-servo SG90/MG90S), controlado via ESP32 (PlatformIO) com modelo preliminar de simulação em MuJoCo.

### Categorização dos Arquivos
| Categoria | Extensões / Pastas | Descrição | Volume |
| :--- | :--- | :--- | :--- |
| **CAD SolidWorks (Original)** | `.SLDPRT`, `.SLDASM`, `.err` | Montagens (`Mont1.SLDASM`, `Mont3.SLDASM`) e peças sólidas | 17 arquivos |
| **STLs — Versão A (com servos)** | `3d_models/referencia/pecas_novas/`, `simulation/assets/` | Malhas com a carcaça/geometria de servomotores acoplada | 14 arquivos |
| **STLs — Versão B (sem servos)** | `3d_models/braço_robótico/`, `original_backup/` | Malhas ocas/impressão 3D das peças plásticas mecânicas | 20 arquivos |
| **STLs de Acessórios (Case)** | `3d_models/case/` | Caixa e tampa do gabinete da eletrônica WEG | 2 arquivos |
| **Modelos de Montagem Unificada** | `robo_montado_glb.glb`, `robo_montado_stl.stl` | Robô inteiro fundido em uma malha estática | 2 arquivos |
| **Simulação MuJoCo** | `simulation/model.xml`, `main.py`, `train_ik.py` | Modelo MJCF e scripts de visualização/treinamento | 3 arquivos |
| **Firmware ESP32** | `src/main.cpp`, `platformio.ini`, `include/`, `lib/` | Código C++ PlatformIO para acionamento dos servos | 3 arquivos base |
| **Binários de Renderização (DLLs)** | `simulation/x64/` (Mesa/Gallium OpenGL) | Drivers de software OpenGL para computadores sem GPU | 4 arquivos (62 MB) |
| **Builds e Caches** | `.pio/`, `.vscode/` | Artefatos de compilação locais gerados automaticamente | ~5 arquivos |
| **Arquivos Temporários / Testes** | `.ps1`, `.png`, `~$*.SLD*`, `.db` | Scripts auxiliares, prints de tela e travas do SolidWorks | ~25 arquivos |

---

## 2. Auditoria Detalhada das Peças STL

Todas as peças foram inspecionadas individualmente com a biblioteca geométrica `trimesh`:

| Arquivo Original | Faces | Bounding Box (mm) | Watertight? | Escala | Classificação / Elo |
| :--- | :---: | :---: | :---: | :---: | :--- |
| `base.STL` *(pecas_novas)* | 5.172 | 45.0 x 45.0 x 24.0 | Sim | mm | **Versão A (com servo)** — Base |
| `Mont3 - base_Servo-1.STL` | 5.252 | 45.0 x 45.0 x 29.5 | Sim | mm | **Versão A (com servo)** — Base Servo |
| `tampa.STL` *(pecas_novas)* | 3.422 | 38.0 x 38.0 x 29.5 | Não | mm | **Versão A (com servo)** — Base Giratória |
| `braco_1.STL` *(pecas_novas)* | 1.190 | 26.5 x 63.3 x 63.3 | Não | mm | **Versão A (com servo)** — Elo 1 (Ombro) |
| `braco_2.STL` *(pecas_novas)* | 1.380 | 18.5 x 80.0 x 26.5 | Não | mm | **Versão A (com servo)** — Elo 2 (Cotovelo) |
| `base_garra.STL` *(pecas_novas)* | 2.660 | 29.0 x 44.0 x 10.0 | Sim | mm | **Versão A (com servo)** — Suporte Garra |
| `garra_direita.STL` *(pecas_novas)* | 878 | 14.0 x 58.4 x 9.0 | Sim | mm | **Versão A (com servo)** — Dedo Direito |
| `garra_esquerda.STL` *(pecas_novas)* | 900 | 14.5 x 58.4 x 9.0 | Sim | mm | **Versão A (com servo)** — Dedo Esquerdo |
| `base_2_weg_v2.stl` | 6.996 | 45.0 x 45.0 x 24.0 | Sim | mm | **Versão B (impressão 3D)** — Base WEG |
| `braco_1_v1.stl` | 870 | 16.5 x 80.0 x 5.0 | Sim | mm | **Versão B (impressão 3D)** — Elo 1 Ombro |
| `braco_2_v1.stl` | 1.078 | 16.5 x 80.0 x 12.0 | Sim | mm | **Versão B (impressão 3D)** — Elo 2 Cotovelo |
| `braco_2_weg_v1.stl` | 978 | 18.5 x 80.0 x 12.0 | Sim | mm | **Versão B (impressão 3D)** — Elo 2 WEG |
| `suporte_garra_v1.stl` | 1.170 | 29.0 x 44.0 x 4.0 | Sim | mm | **Versão B (impressão 3D)** — Suporte Garra |
| `garra_direita_v1.stl` | 940 | 58.4 x 14.0 x 9.0 | Sim | mm | **Versão B (impressão 3D)** — Dedo Direito |
| `garra_esquerda_v1.stl` | 954 | 58.4 x 14.5 x 9.0 | Sim | mm | **Versão B (impressão 3D)** — Dedo Esquerdo |
| `engrenagem_servo_v1.stl` | 1.548 | 20.0 x 20.0 x 5.0 | Sim | mm | **Versão B (impressão 3D)** — Pinhão Servo |
| `engrenagem_garra_1_v1.stl` | 2.342 | 28.0 x 28.0 x 7.0 | Sim | mm | **Versão B (impressão 3D)** — Pinhão Garra |
| `tampa_v2.stl` | 5.618 | 38.0 x 38.0 x 29.5 | Sim | mm | **Versão B (impressão 3D)** — Tampa Base |
| `caixa_weg.stl` *(case)* | 29.976 | 189.0 x 115.9 x 58.0 | Não | mm | **Gabinete Externo** — Case Eletrônica |
| `tampa_weg.stl` *(case)* | 7.634 | 190.0 x 116.8 x 6.0 | Sim | mm | **Gabinete Externo** — Tampa Eletrônica |

> [!NOTE]
> **Escala das Malhas:** Todos os arquivos STL originais estão expressos em **milímetros (mm)**. Para a física correta no MuJoCo, o MJCF precisa importar aplicando a escala `scale="0.001 0.001 0.001"` (ou as malhas de simulação devem ser convertidas diretamente para metros).

---

## 3. Identificação de Duplicatas (Hashes SHA-256 Idênticos)

Foram identificados múltiplos conjuntos de arquivos idênticos espalhados em pastas redundantes:
1. **Base WEG (Hash `782983a6cd`):**
   - `3d_models/braço_robótico/base_2_weg_v2.stl`
   - `3d_models/braço_robótico/original_backup/Base 2 WEG by Schutz.stl`
   - `simulation/assets/base_2_weg_v2.stl`
2. **Braço 1 v1 (Hash `4defb5ec19`):**
   - `3d_models/braço_robótico/braco_1_v1.stl`
   - `3d_models/braço_robótico/original_backup/Brazo 1 v1.stl`
   - `simulation/assets/braco_1_v1.stl`
3. **Braço 2 v1 (Hash `dbd8af98e4`):**
   - `3d_models/braço_robótico/braco_2_v1.stl`
   - `3d_models/braço_robótico/original_backup/Brazo 2 v1.stl`
   - `simulation/assets/braco_2_v1.stl`
4. **Garras, Engrenagens e Suportes:**
   - As peças em espanhol em `original_backup/` são bit a bit idênticas às peças renomeadas em `braço_robótico/` e às cópias em `simulation/assets/`.
5. **Peças da Simulação:**
   - Os arquivos em `simulation/assets/` (`base.STL`, `base_garra.STL`, `braco_1.STL`, etc.) são duplicatas dos arquivos em `3d_models/referencia/pecas_novas/`.

---

## 4. Mapeamento da Cadeia Cinemática e dos Servos

O braço robótico tem topologia de cadeia serial com efetuador acoplado por engrenagens:

```
[Mundo / Chão]
      │
      ▼ (Fixo)
[link_00_base] ─── Motor Servo 1 (Giro Base - Eixo Z)
      │
      ▼ Junta revoluta J1 (Hinge Z)
[link_01_shoulder] (Tampa giratória) ─── Motor Servo 2 (Elevação Ombro - Eixo X)
      │
      ▼ Junta revoluta J2 (Hinge X)
[link_02_upper_arm] (Braço 1) ─── Motor Servo 3 (Extensão Cotovelo - Eixo X)
      │
      ▼ Junta revoluta J3 (Hinge X)
[link_03_forearm] (Braço 2) ─── Micro-servo 4 (Acionamento Garra)
      │
      ▼ Suporte fixo / Punho
[link_04_gripper_base]
      ├──▶ Junta J4_esq (Dedo esquerdo)
      └──▶ Junta J4_dir (Dedo direito acoplado via engrenagem)
```

### Por que a simulação atual (`simulation/model.xml`) apresentava limitações?
1. **Deslocamento de Origem das Peças:** As peças importadas mantinham a origem local do SolidWorks sem zerar o centro das juntas. O arquivo `model.xml` tentou compensar com `pos="0.0132 0.0300 0.0477"` com valores aproximados (*"ESTIMATIVA"* anotada no próprio XML).
2. **Garra Travada como Rígida:** O XML marcava a garra como fixa com o comentário: *"Garra rígida até medirmos o pino de giro dos dedos"*.
3. **Renderizador e DLLs:** A pasta `x64/` continha 60 MB de DLLs de OpenGL de software que poluem o repositório.

---

## 5. Plano de Reorganização (Proposta para a Etapa 2)

Conforme as regras estabelecidas, a reorganização seguirá a estrutura padronizada sem apagar nada:

```
/cad_source/          # Modelos originais SolidWorks (.SLDPRT, .SLDASM, .err)
/meshes/sim/          # STLs da Versão A (com servos), padronizados em snake_case
/meshes/print/        # STLs da Versão B (sem servos / impressão 3D)
/models/              # Arquivo MJCF do MuJoCo (.xml) limpo e calibrado
/sim/                 # Scripts Python de simulação (visualização, IK)
/hardware/            # Firmware PlatformIO (ESP32: main.cpp, platformio.ini)
/docs/                # Documentação técnica, manual original (PDF), JOINTS.md
/_archive/            # Duplicatas, arquivos temporários, DLLs pesadas, com MANIFEST.md
```

### Detalhamento das Movimentações Propostas:

#### A. Para `/cad_source/` (Somente Leitura)
- `3d_models/referencia/pecas_novas/*.SLDPRT`
- `3d_models/referencia/pecas_novas/*.SLDASM`
- `3d_models/referencia/pecas_novas/*.err`

#### B. Para `/meshes/sim/` (STLs com Servos — Simulação MuJoCo)
- `base_Servo-1.STL` ➔ `link_00_base.stl`
- `tampa-1.STL` ➔ `link_01_shoulder.stl`
- `braco_1-1.STL` ➔ `link_02_upper_arm.stl`
- `braco_2_correto-1.STL` ➔ `link_03_forearm.stl`
- `base_garra-1.STL` ➔ `link_04_gripper_base.stl`
- `garra_esquerda-1.STL` ➔ `gripper_finger_left.stl`
- `garra_direita-1.STL` ➔ `gripper_finger_right.stl`

#### C. Para `/meshes/print/` (STLs sem Servos — Impressão 3D)
- `base_2_weg_v2.stl` ➔ `print_base_weg.stl`
- `tampa_v2.stl` ➔ `print_base_cap.stl`
- `braco_1_v1.stl` ➔ `print_arm_lower.stl`
- `braco_2_weg_v1.stl` ➔ `print_arm_upper_weg.stl`
- `braco_2_v1.stl` ➔ `print_arm_upper_plain.stl`
- `suporte_garra_v1.stl` ➔ `print_gripper_mount.stl`
- `garra_esquerda_v1.stl` ➔ `print_finger_left.stl`
- `garra_direita_v1.stl` ➔ `print_finger_right.stl`
- `engrenagem_servo_v1.stl` ➔ `print_gear_servo.stl`
- `engrenagem_garra_1_v1.stl` ➔ `print_gear_finger.stl`

#### D. Para `/docs/`
- Manual PDF: `449747-brazo-robotico-robotic-arm-...pdf`
- Documento de mapeamento: `README_MODELOS.md`
- Imagens de referência de montagem de `3d_models/referencia/imagens_ref/`

#### E. Para `/hardware/`
- `src/main.cpp`
- `platformio.ini`
- `include/` e `lib/`

#### F. Para `/sim/`
- `simulation/main.py`
- `simulation/train_ik.py`

#### G. Para `/models/`
- `simulation/model.xml` (a ser revisado na etapa do gêmeo digital)

#### H. Para `/_archive/` (com rastreamento em `_archive/MANIFEST.md`)
- `3d_models/braço_robótico/original_backup/` (duplicatas em espanhol)
- `simulation/assets/` (duplicatas já transferidas para `/meshes/sim/`)
- `simulation/x64/` (DLLs libgallium, libglapi de 60 MB não essenciais ao versionamento)
- Arquivos de teste temporários criados (`*.ps1`, `chrome_history_copy.db`, prints de teste)
- Travas temporárias do SolidWorks (`~$*.SLD*`)
- Arquivo `3d_models/case/` (gabinete da eletrônica separado da cinemática do braço)

---

## 6. Pontos Ambíguos para Sua Confirmação

Antes de executar qualquer movimentação física de arquivos, preciso que confirme:
1. **Peça do Braço 2 para impressão:** No repositório existem duas opções para o elo superior: a versão com relevo WEG (`braco_2_weg_v1.stl`) e a versão lisa original (`braco_2_v1.stl`). Deseja manter ambas em `/meshes/print/`?
2. **Case da Eletrônica (`caixa_weg.stl` e `tampa_weg.stl`):** Movemos para `/hardware/case_3d/` ou para `/_archive/case/`?
3. **DLLs da pasta `simulation/x64/`:** As DLLs (Mesa/Gallium de 60MB) só são necessárias se você rodar o MuJoCo em um ambiente sem aceleração de GPU DirectX/OpenGL. Deseja arquivá-las em `/_archive/x64_opengl_fallback/` para despoluir o repositório principal?

