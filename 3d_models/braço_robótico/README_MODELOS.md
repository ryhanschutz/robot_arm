# Especificação e Mapeamento dos Modelos 3D (Braço Robótico)

Este documento relaciona os arquivos STL do braço robótico com a documentação original ([449747-brazo-robotico-robotic-arm-33f3bc05-b693-46d1-aa83-425e5f54e855.pdf](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/449747-brazo-robotico-robotic-arm-33f3bc05-b693-46d1-aa83-425e5f54e855.pdf)) e as versões customizadas com a logo/personalização **WEG by Schutz**.

---

## 1. Verificação com o PDF Original

O documento original lista um **Mini braço robótico de 4 Graus de Liberdade (4-DOF)** acionado por micro servos **MG90 / SG90**:
1. **Base** (1º Grau de Liberdade - Giro)
2. **Ombro** (2º Grau de Liberdade - Elevação)
3. **Cotovelo** (3º Grau de Liberdade - Extensão)
4. **Garra** (4º Grau de Liberdade - Abertura/Fechamento)

### Tabela de Comparação e Tradução

| Arquivo Original no PDF | Nome Original na Pasta | Nome Traduzido (PT-BR) | Status / Observações |
| :--- | :--- | :--- | :--- |
| `base-2-v2.stl` | `Base 2 WEG by Schutz.stl` | [`base_2_weg_v2.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/base_2_weg_v2.stl) | ✅ Versão customizada WEG da base giratória |
| `brazo-1-v1.stl` | `Brazo 1 v1.stl` | [`braco_1_v1.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/braco_1_v1.stl) | ✅ Braço inferior (ombro) |
| `brazo-2-v1.stl` | `Brazo 2 v1.stl` | [`braco_2_v1.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/braco_2_v1.stl) | ✅ Braço superior original (cotovelo) |
| *(Variação customizada)* | `Arm 2 WEG by Schutz.stl` | [`braco_2_weg_v1.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/braco_2_weg_v1.stl) | ⚙️ Versão alternativa personalizada WEG |
| `soporte-pinza-v1.stl`| `Soporte Pinza v1.stl` | [`suporte_garra_v1.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/suporte_garra_v1.stl) | ✅ Suporte de fixação da garra/efetuador |
| `pinza-derecha-v1.stl` | `Pinza derecha v1.stl` | [`garra_direita_v1.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/garra_direita_v1.stl) | ✅ Dedo direito da garra |
| `pinza-izquierda-v1.stl`| `Pinza izquierda v1.stl` | [`garra_esquerda_v1.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/garra_esquerda_v1.stl) | ✅ Dedo esquerdo da garra |
| `pinon-servo-v1.stl` | `Piñon Servo v1.stl` | [`engrenagem_servo_v1.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/engrenagem_servo_v1.stl) | ✅ Engrenagem (pinhão) montada no servo |
| `pinon-pinza-1-v1.stl` | `Piñon pinza 1 v1.stl` | [`engrenagem_garra_1_v1.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/engrenagem_garra_1_v1.stl) | ✅ Engrenagem de acoplamento da garra |
| `tapa-v2.stl` | `Tapa v2.stl` | [`tampa_v2.stl`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/tampa_v2.stl) | ✅ Tampa de fechamento/acabamento |

---

## 2. Backup dos Arquivos Originais

Os arquivos originais e com a nomenclatura anterior foram preservados com integridade na subpasta:
- [`original_backup/`](file:///c:/Users/ryhan_Schutz.EDU_FIESC/Documents/PlatformIO/Projects/robot_arm/3d_models/braço_robótico/original_backup)
