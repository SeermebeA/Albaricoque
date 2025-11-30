/* Edge Impulse ingestion SDK
 * Adaptado para 4 PIR + 3 HC-SR05 (Nano 33 BLE)
 */

#include <Albaricoque_inferencing.h>   // Cambia al nombre de tu modelo si es distinto

/*
   Lectura de 4 sensores PIR HC-SR501 y 3 sensores ultrasónicos HC-SR05
   Arduino Nano 33 BLE (3.3 V en pines digitales)

   NOTA DE HARDWARE IMPORTANTE:
   - Nano 33 BLE NO es tolerante a 5V en las entradas.
   - PIR HC-SR501: aliméntalos idealmente con 5V pero revisa que la salida vaya a 3.3V
     o usa un divisor de tensión / level shifter si la salida es 5V.
   - HC-SR05: el pin ECHO suele ir a 5V -> OBLIGATORIO usar divisor de tensión
     (por ejemplo: 10k / 15k) hacia el pin ECHO del Nano 33 BLE.
*/

// --- Configuración de sensores PIR ---
const int NUM_PIR = 4;
const int PIR_PINS[NUM_PIR]  = {2, 3, 4, 5};  // PIR1..PIR4

// --- Configuración de sensores ultrasónicos HC-SR05 ---
const int NUM_US = 3;
const int TRIG_PINS[NUM_US] = {6, 8, 10};     // Ultrasónicos 1..3 - TRIG
const int ECHO_PINS[NUM_US] = {7, 9, 11};     // Ultrasónicos 1..3 - ECHO

const unsigned long ECHO_TIMEOUT_US = 30000UL; // ~5 m máx aprox.

// Parámetros de muestreo/calibración ultrasónicos
const int NUM_MUESTRAS_INICIAL = 20;
const int MUESTRA = 10;   // ms entre muestras de un mismo US durante calibración

// Número total de características por muestra para el modelo
const int FEATURES_PER_SAMPLE = NUM_PIR + NUM_US;  // 4 PIR + 3 US = 7

// Distancia base (baseline) por sensor ultrasónico
long baselineUS[NUM_US];

static bool debug_nn = false; // Poner en true para ver detalles de features, etc.

/**
 * @brief Mide la distancia en cm con un HC-SR05
 */
long medirDistanciaCm(int trigPin, int echoPin) {
    // Aseguramos nivel bajo antes del pulso
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);

    // Pulso de 10 us en TRIG
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Leemos el ancho de pulso en ECHO (HIGH)
    unsigned long duracion = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);

    if (duracion == 0) {
        // No se recibió eco dentro del tiempo -> sin lectura válida
        return -1;
    }

    // Conversión a centímetros (aprox. para HC-SR04/05)
    float distancia = duracion / 58.2f;
    return (long)distancia;
}

/**
 * @brief Calibración inicial: baseline de cada ultrasónico
 */
void calibrarUltrasonicos() {
    ei_printf("Calibrando distancias iniciales de ultrasónicos...\r\n");

    for (int i = 0; i < NUM_US; i++) {
        long suma = 0;
        int lecturasValidas = 0;

        for (int n = 0; n < NUM_MUESTRAS_INICIAL; n++) {
            long d = medirDistanciaCm(TRIG_PINS[i], ECHO_PINS[i]);

            if (d >= 0) {  // Solo contamos lecturas válidas
                suma += d;
                lecturasValidas++;
            }

            // Pausa entre lecturas del mismo sensor
            delay(MUESTRA);
        }

        if (lecturasValidas > 0) {
            baselineUS[i] = suma / lecturasValidas;
        } else {
            baselineUS[i] = -1;   // Sin baseline válida
        }

        ei_printf("Baseline US%d = %ld cm\r\n", i + 1, baselineUS[i]);
        delay(200);
    }

    ei_printf("Calibracion de ultrasónicos finalizada.\r\n\r\n");
}

/**
 * @brief Arduino setup: inicialización de Serial y sensores
 */
void setup()
{
    Serial.begin(115200);
    // Esperar a que el puerto serie esté listo (útil en Nano 33 BLE)
    while (!Serial);

    ei_printf("Edge Impulse Inferencing Demo - 4 PIR + 3 US\r\n");

    // Configuramos PIR como entradas
    for (int i = 0; i < NUM_PIR; i++) {
        pinMode(PIR_PINS[i], INPUT);
    }

    // Configuramos ultrasónicos
    for (int i = 0; i < NUM_US; i++) {
        pinMode(TRIG_PINS[i], OUTPUT);
        pinMode(ECHO_PINS[i], INPUT);
        digitalWrite(TRIG_PINS[i], LOW);  // TRIG en LOW al inicio
    }

    // Calibración inicial de ultrasónicos
    calibrarUltrasonicos();

    // Verificamos que el modelo de Edge Impulse tenga 7 features por muestra
    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != FEATURES_PER_SAMPLE) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME debe ser %d (4 PIR + 3 US).\r\n",
                  FEATURES_PER_SAMPLE);
        ei_printf("Valor actual: %d\r\n", EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME);
        return;
    }
}

/**
 * @brief Bucle principal: muestreo + inferencia
 */
void loop()
{
    ei_printf("\nStarting inferencing in 2 seconds...\n");
    delay(2000);

    unsigned long ts = millis();
    ei_printf("Sampling at TS(ms): %lu ...\r\n", ts);

    // Buffer donde se almacenan las muestras para el modelo
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    // Cada iteración del for llena un "frame" de 7 features:
    // 4 PIR + 3 deltas de ultrasónicos
    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += FEATURES_PER_SAMPLE) {
        // Próximo tick según intervalo del modelo
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        // --- Lectura PIR ---
        int pirValues[NUM_PIR];
        for (int i = 0; i < NUM_PIR; i++) {
            pirValues[i] = digitalRead(PIR_PINS[i]);  // 0 = sin movimiento, 1 = movimiento
        }

        // --- Lectura Ultrasónicos con baseline y delta ---
        long distancias[NUM_US];
        float deltas[NUM_US];

        for (int i = 0; i < NUM_US; i++) {
            distancias[i] = medirDistanciaCm(TRIG_PINS[i], ECHO_PINS[i]);

            if (baselineUS[i] >= 0 && distancias[i] >= 0) {
                long deltaBruto = distancias[i] - baselineUS[i];

                // Delta en valor absoluto (siempre positivo)
                long deltaAbs = labs(deltaBruto);

                // Filtrado: solo cambios mayores a 10 cm, el resto se fuerza a 0
                if (deltaAbs > 10) {
                    deltas[i] = (float)deltaAbs;
                } else {
                    deltas[i] = 0.0f;
                }
            } else {
                deltas[i] = 0.0f;   // error o sin baseline → 0
            }

            // Separación entre sensores ultrasónicos para evitar interferencia
            if (i < NUM_US - 1) {
                delay(MUESTRA);
            }
        }

        // === LOG EN CONSOLA: PIR + DISTANCIAS + DELTAS ===
        size_t sample_index = ix / FEATURES_PER_SAMPLE;
        unsigned long ts_sample = millis();
        ei_printf(
            "TS(ms): %lu | sample: %u | PIR: %d,%d,%d,%d | US(cm): %ld,%ld,%ld | Delta(cm): %.2f,%.2f,%.2f\r\n",
            ts_sample,
            (unsigned int)sample_index,
            pirValues[0], pirValues[1], pirValues[2], pirValues[3],
            distancias[0], distancias[1], distancias[2],
            deltas[0], deltas[1], deltas[2]
        );

        // --- Volcado de features al buffer de Edge Impulse ---
        // Orden fijo: PIR1..PIR4, DeltaUS1..DeltaUS3
        buffer[ix + 0] = (float)pirValues[0];
        buffer[ix + 1] = (float)pirValues[1];
        buffer[ix + 2] = (float)pirValues[2];
        buffer[ix + 3] = (float)pirValues[3];
        buffer[ix + 4] = deltas[0];
        buffer[ix + 5] = deltas[1];
        buffer[ix + 6] = deltas[2];

        // Respetar el intervalo de muestreo del modelo
        uint64_t now = micros();
        if (next_tick > now) {
            delayMicroseconds(next_tick - now);
        }
    }

    // Convertir el buffer en una señal para el clasificador
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Ejecutar el clasificador
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // Imprimir resultados
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(":\n");

    float maxVal = 0.0f;
    const char* detectedClass = "";

    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n",
                  result.classification[ix].label,
                  result.classification[ix].value);

        if (result.classification[ix].value > maxVal) {
            maxVal = result.classification[ix].value;
            detectedClass = result.classification[ix].label;
        }
    }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);
#endif

    // Si quieres filtrar por anomalía alta:
    if (result.anomaly > 0.0f) {
        ei_printf("Anomalía elevada, descartando inferencia. DATOS INVALIDOS\n");
        delay(2000);
        return;
    }

    // Convertir etiqueta detectada a mayúsculas (opcional)
    char upperClass[32];
    strncpy(upperClass, detectedClass, sizeof(upperClass));
    upperClass[sizeof(upperClass) - 1] = '\0';
    for (int i = 0; upperClass[i]; i++) {
        upperClass[i] = toupper(upperClass[i]);
    }

    ei_printf("Clase detectada: %s\n", upperClass);
}

/*
 * Verificación de tipo de sensor del modelo:
 * Debe estar configurado como "Other / Generic" en Edge Impulse.
 */
#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_GENERIC
#undef EI_CLASSIFIER_SENSOR
#define EI_CLASSIFIER_SENSOR EI_CLASSIFIER_SENSOR_GENERIC
#endif

