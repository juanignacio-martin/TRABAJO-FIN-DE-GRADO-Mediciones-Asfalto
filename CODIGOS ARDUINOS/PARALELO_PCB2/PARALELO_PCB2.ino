// ==========================================
// ARDUINO 3 — FIRMWARE PARALELO, VERSIÓN PCB2
// ==========================================
//
// Proyecto: Digital-PAVE (TED2021-131474B-I00)
// Autor:    Juan Ignacio Martín Moreno
// Tutor:    Giuseppe Conti (ETSIDI-UPM)
// Cotutor:  Federico Gulisano (ETSICCP-UPM)
//
// Descripción:
//   Firmware del tercer Arduino Nano ESP32 del sistema, dedicado
//   exclusivamente a la configuración en paralelo de la PCB2. Gestiona
//   los cuatro convertidores ADS1115 U9-U12, repartidos en dos buses
//   I2C independientes (Wire y Wire1), adquiere los ocho voltajes crudos
//   de los nodos de medida y los transmite a la Raspberry Pi 5 por SPI.
//
//   A diferencia de los Arduinos 1 y 2 (configuración serie), aquí cada
//   canal corresponde a un divisor de tensión completamente independiente:
//   una resistencia de referencia integrada en la PCB2 en serie con el
//   tramo de mezcla asfáltica correspondiente. No existe corriente común
//   entre canales: la resistencia de cada tramo se calcula de forma
//   autónoma en la Raspberry Pi a partir de la tensión medida en el nodo
//   intermedio de su propio divisor.
//
// Arquitectura de doble núcleo:
//   Core 0 (tarea FreeRTOS): gestiona Wire1 → U11 (0x48) y U12 (0x49)
//                             → voltajes[4..7] (RP4 a RP7)
//   Core 1 (loop principal): gestiona Wire  → U9  (0x48) y U10 (0x49)
//                             → voltajes[0..3] (JP7, RP1, RP2, RP3)
//                             + comunicación SPI con la Raspberry Pi
//
// Mapeo de voltajes → nodos de los divisores de tensión en paralelo:
//   V[0] = AIN0(U9)  → nodo del jumper JP7:
//                       posición 1: mide V_REF real en tiempo real
//                       posición 2: mide el nodo del canal RP8 (borne 8)
//   V[1] = AIN1(U9)  → nodo del canal RP1 (entre R_ref1 y R_asfalto1)
//   V[2] = AIN0(U10) → nodo del canal RP2
//   V[3] = AIN1(U10) → nodo del canal RP3
//   V[4] = AIN0(U11) → nodo del canal RP4
//   V[5] = AIN1(U11) → nodo del canal RP5
//   V[6] = AIN0(U12) → nodo del canal RP6
//   V[7] = AIN1(U12) → nodo del canal RP7
//
//   La Raspberry Pi aplica la fórmula del divisor de tensión a cada canal:
//   R_asfalto = R_ref × V_nodo / (V_ref − V_nodo)
//   donde V_ref es la tensión de alimentación de los divisores, tomada
//   en tiempo real de V[0] si JP7 está en posición 1, o como constante
//   configurada por el usuario si JP7 está en posición 2.
//
// Protocolo SPI:
//   Idéntico a los Arduinos 1 y 2: esclavo SPI2 a 1 MHz, modo 0,
//   payload de 36 bytes (firma 0xABCD1234 + 8 floats de voltaje),
//   señal DRDY en GPIO 2.
// ==========================================

#include <Wire.h>
#include <ESP32SPISlave.h>

// ==========================================
// CONFIGURACIÓN DE HARDWARE
// ==========================================

// Pines del primer bus I2C
constexpr uint8_t I2C0_SDA_PIN = A4;
constexpr uint8_t I2C0_SCL_PIN = A5;

// Pines del segundo bus I2C
constexpr uint8_t I2C1_SDA_PIN = D4;
constexpr uint8_t I2C1_SCL_PIN = D5;

// Direcciones I2C de los ADS1115
// Wire:  U9(0x48)  U10(0x49)
// Wire1: U11(0x48) U12(0x49)
constexpr uint8_t ADDR_U9_U11  = 0x48;
constexpr uint8_t ADDR_U10_U12 = 0x49;

// Ganancia / fondo de escala: configurado a ±4,096V.
// Da 0,000125V por bit — resolución adecuada para el rango 0-3,44V
constexpr float VOLTS_PER_BIT = 0.000125f;

constexpr uint32_t ADC_CONVERSION_TIME_US   = 1350;
constexpr uint32_t SERIAL_PRINT_INTERVAL_US = 500000UL;

// Configuración pines SPI
const int GPIO_MOSI = 38;
const int GPIO_MISO = 47;
const int GPIO_SCK  = 48;
const int GPIO_CS   = 21;
const int PIN_DRDY  = 2;  // señal de dato listo hacia la Raspberry Pi

ESP32SPISlave slave;

// ==========================================
// ENVÍO DE DATOS SPI
// Sin relleno para que el struct se desempaquete correctamente
// en la Raspberry Pi con struct.unpack('<I8f')
// ==========================================
#pragma pack(push, 1)
struct SpiPayload {
    uint32_t firma;       // 4 bytes — la RPi comprueba este valor para
                           // validar que la transacción SPI es correcta
    float voltajes[8];    // 32 bytes — 8 voltajes crudos de los nodos
};                         // 36 bytes en total, sin relleno
#pragma pack(pop)

SpiPayload datosSalida;
uint8_t datosEntrada[sizeof(SpiPayload)]; // buffer de recepción exigido
    // por el protocolo SPI full-duplex; en esta versión no se recibe
    // contenido útil de la Raspberry Pi

// ==========================================
// VOLTAJES COMPARTIDOS ENTRE NÚCLEOS
// ==========================================

// Core 0 escribe V[4..7] (U11 y U12, canales RP4-RP7)
// Core 1 escribe V[0..3] (U9 y U10, JP7 y RP1-RP3)
// volatile garantiza visibilidad cruzada entre núcleos sin caché de registros
volatile float shared_voltages[8];

// Flag de sincronización: Core 0 lo pone a true al terminar sus lecturas;
// Core 1 lo pone a false para liberar a Core 0 y arrancar un nuevo ciclo
volatile bool core0_ready = false;

TaskHandle_t TaskCore0;
uint32_t measurementCount = 0;
uint32_t lastPrintTime = 0;

// ==========================================
// FUNCIONES I2C
// ==========================================

// Inicia una conversión single-shot en un ADS1115.
// canal=0 → AIN0 vs GND (msb=0xC3)
// canal=1 → AIN1 vs GND (msb=0xD3)
// Ambos valores configuran ganancia ±4,096V y 860 SPS
void triggerConversion(TwoWire& wire, uint8_t addr, uint8_t channel) {
    uint8_t msb = (channel == 0) ? 0xC3 : 0xD3;
    wire.beginTransmission(addr);
    wire.write(0x01);   // puntero al registro CONFIG del ADS1115
    wire.write(msb);    // canal + ganancia + modo + inicio de conversión
    wire.write(0xE3);   // 860 SPS → tiempo de conversión ≈ 1,16 ms
    wire.endTransmission(); // aquí arranca la conversión
}

// Lee el registro de conversión del ADS1115 y devuelve el resultado en voltios.
// Devuelve 0.0 si el ADS no responde con ACK o no entrega los 2 bytes esperados
float readAdcVolts(TwoWire& wire, uint8_t addr) {
    wire.beginTransmission(addr);
    wire.write(0x00);   // puntero al registro de conversión (resultado de 16 bits)
    if (wire.endTransmission() != 0) return 0.0f;
    if (wire.requestFrom(addr, (uint8_t)2) == 2) {
        int16_t raw = (int16_t)((wire.read() << 8) | wire.read());
            // MSB desplazado 8 bits a la izquierda y combinado con LSB;
            // cast a int16_t para interpretar correctamente valores negativos
        return raw * VOLTS_PER_BIT;
    }
    return 0.0f;
}

// ==========================================
// TAREA DEL NÚCLEO 0 (Wire1: U11 y U12)
// V[4]=AIN0(U11)=RP4  V[5]=AIN1(U11)=RP5
// V[6]=AIN0(U12)=RP6  V[7]=AIN1(U12)=RP7
// ==========================================

// Se ejecuta permanentemente en Core 0 en paralelo con el loop() de Core 1.
// No gestiona SPI: solo adquiere los cuatro voltajes de U11 y U12
// y señala su disponibilidad mediante el flag core0_ready
void core0Code(void * pvParameters) {
    for (;;) {
        // Canal 0: dispara U11 y U12 simultáneamente y espera la conversión
        triggerConversion(Wire1, ADDR_U9_U11, 0);
        triggerConversion(Wire1, ADDR_U10_U12, 0);

        uint32_t timer = micros();
        while (micros() - timer < ADC_CONVERSION_TIME_US) { yield(); }
            // yield() cede tiempo al scheduler de FreeRTOS,
            // evitando el reset por timeout del watchdog

        shared_voltages[4] = readAdcVolts(Wire1, ADDR_U9_U11);  // AIN0(U11)=RP4
        shared_voltages[6] = readAdcVolts(Wire1, ADDR_U10_U12); // AIN0(U12)=RP6

        // Canal 1: dispara y espera
        triggerConversion(Wire1, ADDR_U9_U11, 1);
        triggerConversion(Wire1, ADDR_U10_U12, 1);

        timer = micros();
        while (micros() - timer < ADC_CONVERSION_TIME_US) { yield(); }

        shared_voltages[5] = readAdcVolts(Wire1, ADDR_U9_U11);  // AIN1(U11)=RP5
        shared_voltages[7] = readAdcVolts(Wire1, ADDR_U10_U12); // AIN1(U12)=RP7

        core0_ready = true;   // indica a Core 1 que los voltajes están listos
        while (core0_ready) { yield(); } // espera a que Core 1 libere el flag
    }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(921600); // solo para verificación visual en el IDE

    // Inicializa los dos buses I2C en Fast Mode (400 kHz)
    Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN);
    Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);
    Wire.setClock(400000);
    Wire1.setClock(400000);

    pinMode(PIN_DRDY, OUTPUT);
    digitalWrite(PIN_DRDY, LOW); // reposo; se activará en HIGH antes de cada SPI

    // Configura el periférico SPI2 como esclavo en modo 0 (CPOL=0, CPHA=0)
    slave.setDataMode(SPI_MODE0);
    slave.begin(SPI2_HOST, GPIO_SCK, GPIO_MISO, GPIO_MOSI, GPIO_CS);

    // Inicializa el payload con la firma correcta y buffers a cero
    memset(&datosSalida, 0, sizeof(SpiPayload));
    datosSalida.firma = 0xABCD1234;
    memset(datosEntrada, 0, sizeof(datosEntrada));

    // Crea la tarea de Core 0 con 10 KB de stack, prioridad 1,
    // anclada al núcleo físico 0 del ESP32-S3
    xTaskCreatePinnedToCore(core0Code, "TaskCore0", 10000, NULL, 1, &TaskCore0, 0);

    lastPrintTime = micros();
    Serial.printf("\n[+] Arduino 3 — Voltajes crudos. Iniciado.\n");
}

// ==========================================
// LOOP — NÚCLEO 1 (Wire: U9 y U10)
// V[0]=AIN0(U9) ← JP7 (V_REF o RP8 según modo)
// V[1]=AIN1(U9) ← RP1
// V[2]=AIN0(U10) ← RP2
// V[3]=AIN1(U10) ← RP3
// ==========================================

// Core 1 gestiona U9 y U10, la transacción SPI y la sincronización con Core 0.
// La transacción SPI se solapa con la conversión del canal 0 del ADC
// para no añadir latencia al ciclo de adquisición
void loop() {

    // 1. DISPARAR CANAL 0 — U9 y U10
    // AIN0(U9)=JP7 y AIN0(U10)=RP2
    triggerConversion(Wire, ADDR_U9_U11, 0);
    triggerConversion(Wire, ADDR_U10_U12, 0);
    uint32_t timer = micros();

    // 2. TRANSACCIÓN SPI CON LA RASPBERRY PI (solapada con la conversión ADC)
    slave.queue((uint8_t*)&datosSalida, datosEntrada, sizeof(SpiPayload));
        // envía los voltajes del ciclo ANTERIOR; cast a uint8_t* porque
        // la librería SPI trabaja con bytes, no con structs
    digitalWrite(PIN_DRDY, HIGH); // avisa a la RPi que hay datos disponibles
    slave.wait();                  // a 1 MHz, 36 bytes tardan ≈ 0,288 ms,
                                   // muy por debajo de los 1350 µs de conversión
    digitalWrite(PIN_DRDY, LOW);
    memset(datosEntrada, 0, sizeof(datosEntrada));

    // 3. ESPERAR LO QUE RESTE DE CONVERSIÓN DEL CANAL 0
    while (micros() - timer < ADC_CONVERSION_TIME_US) { yield(); }

    // 4. LEER CANAL 0 — U9 y U10
    shared_voltages[0] = readAdcVolts(Wire, ADDR_U9_U11);  // AIN0(U9)=JP7
    shared_voltages[2] = readAdcVolts(Wire, ADDR_U10_U12); // AIN0(U10)=RP2

    // 5. DISPARAR CANAL 1 — U9 y U10
    triggerConversion(Wire, ADDR_U9_U11, 1);
    triggerConversion(Wire, ADDR_U10_U12, 1);
    timer = micros();
    while (micros() - timer < ADC_CONVERSION_TIME_US) { yield(); }

    // 6. LEER CANAL 1 — U9 y U10
    shared_voltages[1] = readAdcVolts(Wire, ADDR_U9_U11);  // AIN1(U9)=RP1
    shared_voltages[3] = readAdcVolts(Wire, ADDR_U10_U12); // AIN1(U10)=RP3

    // 7. SINCRONIZACIÓN CON NÚCLEO 0
    // En la práctica Core 0 suele terminar antes por no tener la carga de la SPI
    while (!core0_ready) { yield(); }

    // 8. EMPAQUETAR VOLTAJES PARA EL PRÓXIMO CICLO SPI
    datosSalida.firma = 0xABCD1234;
    for (int i = 0; i < 8; i++) datosSalida.voltajes[i] = shared_voltages[i];
        // los voltajes se enviarán en el siguiente ciclo con un desfase
        // de ~4,5 ms, despreciable frente a las constantes de tiempo del fenómeno

    measurementCount++;
    core0_ready = false; // libera a Core 0 para el siguiente ciclo

    // 9. MONITOR USB — frecuencia de muestreo real cada 500 ms
    uint32_t currentTime = micros();
    if (currentTime - lastPrintTime >= SERIAL_PRINT_INTERVAL_US) {
        float freq = measurementCount / ((currentTime - lastPrintTime) / 1000000.0f);
        lastPrintTime    = currentTime;
        measurementCount = 0;
        Serial.printf("Freq: %.1f Hz | V0:%.4f V1:%.4f V2:%.4f V3:%.4f V4:%.4f V5:%.4f V6:%.4f V7:%.4f\n",
                      freq,
                      shared_voltages[0], shared_voltages[1],
                      shared_voltages[2], shared_voltages[3],
                      shared_voltages[4], shared_voltages[5],
                      shared_voltages[6], shared_voltages[7]);
    }
}