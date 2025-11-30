# Albaricoque — Perimeter Human-Only Node (Arduino Nano 33 BLE Sense Rev2 + Edge Impulse)

> **Detección de personas sin cámaras**, preservando la privacidad, en puertas, accesos y cercas.  
> Cobertura objetivo: **~0–3 m** en un sector de **~90°**.  
> **Inferencia en el borde** con Edge Impulse sobre **Arduino Nano 33 BLE Sense Rev2**.  
> **Prototipo** con carcasa impresa en 3D (PETG/TPU). LoRa está **diseñado** pero **no implementado** en esta fase.

---

## 1) Descripción general

**Albaricoque** es un nodo perimetral de bajo consumo que **detecta exclusivamente personas** sin recurrir a cámaras. Combina un arreglo **multiaxial de 4 sensores PIR (HC-SR501)** —para dirección y cinemática a partir del **orden** y los **Δt** de activación— con **3 sensores ultrasónicos (HC-SR04)** inclinados a **15°/45°/75°**, cuyos ecos se procesan en **ráfagas** (mediana y varianza) para estimar **distancia** y la **altura aparente** del objetivo. Estas señales se **fusionan** y alimentan un **modelo TinyML** (Edge Impulse) que ejecuta la inferencia **on-device** en un **Arduino Nano 33 BLE Sense Rev2**. El resultado final es una decisión rápida y confiable **humano vs. no humano** (animales/ambiente).

**Estado del prototipo:**  
- LoRa está planificado pero **no implementado** (se diseñó la carcasa para alojar antena).  
- Carcasa **sin sellado IP** (prototipo): impresa en 3D **PETG (estructura)** y **TPU (juntas)**.  
- Montaje final validado: **1.5 m de altura**, **90°** respecto al suelo (apuntando al frente).

---

## 2) Características clave

- **Privacy-by-design**: no usa cámaras, no guarda imágenes.  
- **Fusión de sensores**: 4× PIR + 3× ultrasónico (15°/45°/75°).  
- **Inferencia local**: Edge Impulse (SDK C++) en Arduino Nano 33 BLE Sense Rev2.  
- **Baja latencia**: pipeline por eventos (trigger PIR → ráfagas ultrasónicas → features → inferencia).  
- **Prototipado rápido**: carcasa 3D PETG/TPU preparada para futura antena LoRa.

---

## 3) Arquitectura técnica (alto nivel)

```
[ PIR x4 ]           [ Ultrasónico x3 ]
 (110° FOV)          (15° | 45° | 75°)
    |                      |
    |  orden/Δt PIR        | ráfagas -> mediana/varianza -> r (m)
    |                      | geometría -> h = H − r·sin(θ)
    |                      |
    +----------[ Fusión de features ]----------+
                                               |
                                     [ Modelo EI (on-device) ]
                                               |
                                        [ Decisión Persona ]
                                               |
                                       [ Señal / Log serie ]
```

**Geometría clave:**  
- Altura de montaje **H = 1.5 m**  
- Ángulos ultrasónicos: **θ ∈ {15°, 45°, 75°}**  
- **Altura aparente**: \( h = H - r \cdot \sin(\theta) \)

---

## 4) Hardware

### 4.1 Bill of Materials (BOM)

| Componente | Cant. | Notas |
|---|---:|---|
| Arduino Nano 33 BLE Sense Rev2 | 1 | nRF52840, 3.3 V lógico |
| PIR HC-SR501 | 4 | Ajustar sensibilidad/tiempo; FOV ~110° |
| Ultrasonido HC-SR04 | 3 | **ECHO es 5 V** → usar **level shifting/divisor** a 3.3 V |
| Impresiones 3D (PETG/TPU) | — | Carcasa prototipo, juntas blandas |
| Tornillería, cableado, protoboard/PCB | — | Según diseño |

> **Seguridad eléctrica:** el **Nano 33 BLE Sense Rev2 es 3.3 V**. Los **HC-SR04** entregan **ECHO a 5 V**: usa **divisor resistivo** o **level shifter** antes de entrar a los pines del Arduino.

### 4.2 Notas mecánicas (prototipo)
- Cuerpo/tapa en **PETG**, juntas en **TPU 95A** (prototipo sin IP).  
- Alojamiento y **reserva para antena LoRa** (fase futura).  
- Viseras/laberintos para PIR y **hood** para transductores ultrasónicos.

---

## 5) Metodología y datos

- **Captura en campo** con variación sistemática de **distancia (0–3 m)**, **azimut (−45°…+45°)**, **velocidad** (lento/normal/rápido) y **postura** (de pie, agachado, gateo).  
- **Negativos duros**: **perros**, **ramas con viento**, **lluvia**.  
- **Features**: orden/Δt PIR, duración y % de solape; mediana/varianza de ráfagas ultrasónicas; **h** (altura aparente); estimación de velocidad; compensación térmica (velocidad del sonido).  
- **Selección de umbral** con **curvas ROC**; evaluación por **matrices de confusión** segmentadas por rango y azimut.

---

## 6) Flujo de funcionamiento

1. **Trigger por PIR** (cualquier canal activa el evento).  
2. **Secuencia ultrasónica** (para evitar crosstalk): 75° → 45° → 15°, **ráfagas** con **mediana** y control de outliers.  
3. **Cálculo de features** (r, varianza, **h** por sensor, fusión robusta).  
4. **Inferencia** (modelo Edge Impulse on-device).  
5. **Decisión** y **salida** (log serie; en el futuro, **LoRa**).

---

## 7) Integración con Edge Impulse

1. Crear proyecto en **Edge Impulse** (público o privado).  
2. Definir **clases**: `human` / `nonhuman` (animal/ambiente).  
3. Cargar dataset etiquetado (o adquirir con SDK).  
4. Diseñar **bloque de features**: señales PIR + ultrasónico (mediana/varianza) + `h`.  
5. Entrenar y **cuantizar (INT8)**.  
6. **Export** SDK C++ e incluir en firmware (p. ej. `Albaricoque_inferencing.h`).  
7. Medir **latencia**, **memoria** y **exactitud** on-device.

**Pseudocódigo de inferencia:**
```cpp
#include "Albaricoque_inferencing.h"  // generado por EI

// features[] = {pir_order, pir_dt1, pir_dt2, r15, r45, r75, h15, h45, h75, var15, var45, var75, ...}
EI_IMPULSE_ERROR res = run_classifier(&signal, &result, false);
if (result.classification[HUMAN].value >= THRESH) {
  // Persona detectada
}
```

---

## 8) Desarrollo de firmware

**Entorno recomendado:** [PlatformIO](https://platformio.org/)  
`platformio.ini` (ejemplo):

```ini
[env:nano33ble]
platform = nordicnrf52
board = nano33ble
framework = arduino
monitor_speed = 115200
build_flags =
  -D SERIAL_BAUD=115200
  ; agrega defines necesarios para EI
lib_deps =
  ; (no requerido para PIR/HC-SR04 básicos)
```

**Comandos básicos:**
```bash
# Compilar
pio run

# Subir firmware (ajusta puerto)
pio run -t upload --upload-port /dev/ttyACM0

# Monitor serie
pio device monitor -b 115200
```

---

## 9) Métricas objetivo

- **Detección:** alta **sensibilidad** (TPR) en personas; **baja FPR** (animales/ambiente).  
- **Latencia:** pipeline < **300 ms** (objetivo).  
- **Cobertura:** **0–3 m**, sector ~**90°**.  
- **Privacidad:** sin video, sin imágenes.

> **Nota:** reportar métricas con **matrices de confusión** por bandas de distancia y azimut, además de **FAR** (falsas alarmas por día) en escenarios reales.

---

## 10) Roadmap (alto nivel)

- [x] Arquitectura y geometría (15°/45°/75°, H=1.5 m).  
- [x] Prototipo mecánico (PETG/TPU) y electrificación.  
- [x] Captura inicial y baseline de features.  
- [ ] Entrenamiento EI + integración SDK en firmware.  
- [ ] Validación en campo (puerta/acceso/cerca) y ajuste de umbral.  
- [ ] LoRa P2P (futuro) + receptor y dashboard.

---

## 11) Estructura sugerida del repositorio

```
.
├── firmware/                 # Código Arduino/PlatformIO
│   ├── src/
│   ├── include/
│   └── platformio.ini
├── docs/
│   ├── ARCHITECTURE.md       # Decisiones, geometría, fórmulas
│   └── DATASET.md            # Protocolo de captura y etiquetado
├── mechanical/               # STL/STEP de la carcasa (prototipo)
├── edge-impulse/             # Notas del proyecto EI / enlaces
├── hardware/                 # BOM y esquemas simples
└── README.md
```

---

## 12) Limitaciones y trabajo futuro

- **LoRa**: diseñado pero **no** operativo (requiere integración de radio + protocolo).  
- **Protección ambiental**: la carcasa **no es IP** en esta fase (prototipo).  
- **Transductores**: para producción, considerar **ultrasónicos sellados** y **vent ePTFE**.  
- **Auto-calibración**: incorporar IMU para compensar ángulo real y baseline de suelo nocturno.

---

## 13) Licencia

Este proyecto se distribuye bajo **MIT License** (ver `LICENSE`).

---

## 14) Agradecimientos

- Comunidad Edge Impulse y makers que inspiran el desarrollo **edge-native**.  
- Contribuciones y *feedback* sobre escenarios de prueba en exteriores.

## 15) Final Logs & Analysis

**Raw samples (excerpt):**
```text
TS(ms): 1430071 | sample: 0 | PIR: 1,0,0,1 | US(cm): 38,42,-1 | Delta(cm): 156.00,153.00,0.00
TS(ms): 1430348 | sample: 1 | PIR: 1,0,0,0 | US(cm): 32,-1,33 | Delta(cm): 162.00,0.00,164.00
TS(ms): 1430651 | sample: 2 | PIR: 1,0,0,0 | US(cm): 30,-1,-1 | Delta(cm): 164.00,0.00,0.00
TS(ms): 1430918 | sample: 3 | PIR: 0,0,0,0 | US(cm): 27,304,-1 | Delta(cm): 167.00,109.00,0.00
TS(ms): 1431222 | sample: 4 | PIR: 0,1,0,0 | US(cm): -1,303,-1 | Delta(cm): 0.00,108.00,0.00
TS(ms): 1431457 | sample: 5 | PIR: 0,1,0,0 | US(cm): 25,-1,23 | Delta(cm): 169.00,0.00,174.00
TS(ms): 1431761 | sample: 6 | PIR: 0,0,0,0 | US(cm): 24,-1,-1 | Delta(cm): 170.00,0.00,0.00
TS(ms): 1432038 | sample: 7 | PIR: 0,0,0,0 | US(cm): 25,-1,-1 | Delta(cm): 169.00,0.00,0.00
TS(ms): 1432290 | sample: 8 | PIR: 0,0,0,0 | US(cm): 23,28,-1 | Delta(cm): 171.00,167.00,0.00
TS(ms): 1432584 | sample: 9 | PIR: 0,0,0,0 | US(cm): 46,291,-1 | Delta(cm): 148.00,96.00,0.00
TS(ms): 1432871 | sample: 10 | PIR: 0,0,1,1 | US(cm): -1,-1,23 | Delta(cm): 0.00,0.00,174.00
TS(ms): 1433122 | sample: 11 | PIR: 0,0,1,1 | US(cm): 23,-1,26 | Delta(cm): 171.00,0.00,171.00
TS(ms): 1433425 | sample: 12 | PIR: 0,0,0,1 | US(cm): 17,-1,-1 | Delta(cm): 177.00,0.00,0.00
TS(ms): 1433703 | sample: 13 | PIR: 0,0,0,1 | US(cm): 18,-1,-1 | Delta(cm): 176.00,0.00,0.00
TS(ms): 1433928 | sample: 14 | PIR: 0,0,0,1 | US(cm): 18,20,21 | Delta(cm): 176.00,175.00,176.00
TS(ms): 1434285 | sample: 15 | PIR: 0,0,0,1 | US(cm): -1,-1,-1 | Delta(cm): 0.00,0.00,0.00
TS(ms): 1434536 | sample: 16 | PIR: 0,0,0,1 | US(cm): 17,-1,-1 | Delta(cm): 177.00,0.00,0.00
TS(ms): 1434813 | sample: 17 | PIR: 0,1,0,1 | US(cm): 16,-1,-1 | Delta(cm): 178.00,0.00,0.00
TS(ms): 1435091 | sample: 18 | PIR: 0,1,0,1 | US(cm): 18,-1,-1 | Delta(cm): 176.00,0.00,0.00
TS(ms): 1435315 | sample: 19 | PIR: 0,0,0,0 | US(cm): 13,17,13 | Delta(cm): 181.00,178.00,184.00
TS(ms): 1435593 | sample: 20 | PIR: 1,0,0,0 | US(cm): 12,20,12 | Delta(cm): 182.00,175.00,185.00
TS(ms): 1435879 | sample: 21 | PIR: 1,0,1,0 | US(cm): 17,182,15 | Delta(cm): 177.00,13.00,182.00
TS(ms): 1436181 | sample: 22 | PIR: 1,0,1,0 | US(cm): 25,116,-1 | Delta(cm): 169.00,79.00,0.00
TS(ms): 1436438 | sample: 23 | PIR: 0,0,0,0 | US(cm): 113,115,27 | Delta(cm): 81.00,80.00,170.00
TS(ms): 1436720 | sample: 24 | PIR: 0,0,0,0 | US(cm): 114,107,135 | Delta(cm): 80.00,88.00,62.00
```
- **Samples:** 25 from **TS 1430071 ms** to **1436720 ms** (span **6649 ms**); average sampling period ≈ **277.0 ms**.
- **Missing echoes** are marked as `-1`. Outlier long echoes (e.g., ~291–304 cm) indicate multi‑path or spurious reflections under outdoor conditions.
- **Ultrasonic CH0** → valid: 22, invalid: 3, range: 12–114 cm, mean: 31.41 cm.
- **Ultrasonic CH1** → valid: 12, invalid: 13, range: 17–304 cm, mean: 128.75 cm.
- **Ultrasonic CH2** → valid: 10, invalid: 15, range: 12–135 cm, mean: 32.8 cm.
- **PIR activation totals** (over 25 samples): CH0=6, CH1=4, CH2=4, CH3=10.
- **Pattern observation:** bursts of short ranges (13–32 cm) appear when PIRs 2–3 fire, consistent with a subject traversing close to the sensor plane; mid‑range returns (80–135 cm) co‑occur with no PIR activity (cool‑down/background).
- **Delta(cm)** fields are derived features (e.g., apparent height or geometry transforms) and align with missing echoes when zeroed.

**Edge Impulse inference (sample output):**
```text
Predictions (DSP: 0 ms., Classification: 1 ms., Anomaly: 0 ms.):
Animal: 0.00391
Solo: 0.00391
Walk1: 0.69922
Walk2: 0.21484
Walk3: 0.03516
Walk4: 0.03906
anomaly score: -0.867
Clase detectada: WALK1

Starting inferencing in 2 seconds...
Sampling at TS(ms): 1410121 ...
```
- **On‑device latency:** ~**1 ms** classification (reported), confirming real‑time feasibility.
- **Predicted class:** `WALK1` (≈0.70) with a low anomaly score (close to the learned manifold).
- **Next steps:** broaden dataset for `WALK2–WALK4`, add hard negatives under similar lighting/weather, and tune the operating threshold (ROC) to target **TPR ≥95%** with **<1 false alarm/day**.
