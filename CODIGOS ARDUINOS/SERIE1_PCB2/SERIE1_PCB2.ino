// ==========================================
// ARDUINO 1 — FIRMWARE SERIE, VERSIÓN PCB2
// ==========================================
//
// Proyecto: Digital-PAVE (TED2021-131474B-I00)
// Autor:    Juan Ignacio Martín Moreno
// Tutor:    Giuseppe Conti (ETSIDI-UPM)
// Cotutor:  Federico Gulisano (ETSICCP-UPM)
//
// Descripción:
//   Firmware del primer Arduino Nano ESP32 del sistema en configuración
//   serie para la PCB2. Gestiona cuatro convertidores ADS1115 repartidos
//   en dos buses I2C independientes (Wire y Wire1), adquiere los ocho
//   voltajes crudos de los nodos de la cadena de medida y los transmite
//   a la Raspberry Pi 5 por SPI.
//
//   A diferencia de la PCB1, este firmware NO calcula resistencias: envía
//   únicamente los voltajes crudos para que la Raspberry Pi realice el
//   cálculo centralizado según la topología declarada por el usuario.
//
// Arquitectura de doble núcleo:
//   Core 0 (tarea FreeRTOS): gestiona Wire1 → U3 (0x48) y U4 (0x49)
//                             → voltajes[4..7]
//   Core 1 (loop principal): gestiona Wire  → U1 (0x48) y U2 (0x49)
//                             → voltajes[0..3]
//                             + comunicación SPI con la Raspberry Pi
//
//   La transacción SPI se solapa con el tiempo de conversión del ADC
//   del canal 0 (1350 µs), eliminando el coste de comunicación del ciclo
//   y maximizando la frecuencia de muestreo efectiva .
//
// Mapeo de voltajes → nodos de la cadena serie:
//   V[0] = AIN0(U1) → nodo entre R_ref y R1
//   V[1] = AIN1(U1) → nodo entre R1 y R2
//   V[2] = AIN0(U2) → nodo entre R2 y R3
//   V[3] = AIN1(U2) → nodo entre R3 y R4
//   V[4] = AIN0(U3) → nodo entre R4 y R5
//   V[5] = AIN1(U3) → nodo entre R5 y R6
//   V[6] = AIN0(U4) → nodo entre R6 y R7
//   V[7] = AIN1(U4) → extremo final de la cadena (asumido 0 V en la RPi)
//
// Protocolo SPI:
//   Esclavo SPI2 a 1 MHz, modo 0 (CPOL=0, CPHA=0).
//   Payload fijo de 36 bytes: 4 bytes de firma (0xABCD1234) + 8 floats
//   de voltaje. La firma permite a la Raspberry Pi detectar transacciones
//   corruptas o desincronizadas y descartarlas.
//   El pin DRDY (GPIO 2) se activa en HIGH para señalar a la Raspberry Pi
//   que hay datos disponibles, evitando el polling continuo.
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
// Dirección de U1 y U3
constexpr uint8_t ADDR_U1_U3 = 0x48;
// Dirección de U2 y U4
constexpr uint8_t ADDR_U2_U4 = 0x49;

// Ganancia / fondo de escala: configurado a ±4,096V. El fondo de escala
// de 16 bits da 0,000125V por bit — resolución adecuada para el rango
// de voltaje esperado en el circuito (0-3,44V)
constexpr float VOLTS_PER_BIT = 0.000125f;

// Tiempo de seguridad EN MICROSEGUNDOS para garantizar la conversión
// del ADS1115, sin leer datos incompletos ni perder frecuencia de muestreo
constexpr uint32_t ADC_CONVERSION_TIME_US   = 1350;
constexpr uint32_t SERIAL_PRINT_INTERVAL_US = 500000UL;

// Configuración pines SPI
const int GPIO_MOSI = 38;
const int GPIO_MISO = 47;
const int GPIO_SCK  = 48;
const int GPIO_CS   = 21;
const int PIN_DRDY  = 2;  
// señal de dato listo hacia la Raspberry Pi;
// se activa en HIGH al comienzo de cada
// transacción SPI y vuelve a LOW al terminar

// Crea el objeto esclavo SPI
ESP32SPISlave slave;

// ==========================================
// ENVÍO DE DATOS SPI
// ==========================================

// Guarda la configuración de alineamiento y empaqueta a 1 byte, así el
// compilador no añade bytes de relleno que harían que el struct ocupara
// más de lo esperado y se desempaquetara de forma incorrecta en la RPi
#pragma pack(push, 1)

struct SpiPayload {
    uint32_t firma;       // 4 bytes — número que la RPi comprueba para
                           // saber si la transacción SPI ha sido correcta
    float voltajes[8];    // 32 bytes — 8 voltajes crudos de los nodos
};                         // 36 bytes en total, sin relleno
#pragma pack(pop)          // restaura la condición de alineamiento normal

// Variables para el full-duplex: lo que sale y lo que entra simultáneamente
SpiPayload datosSalida;
uint8_t datosEntrada[sizeof(SpiPayload)]; // en esta versión no se recibe
    // nada útil de la Raspberry Pi, pero el protocolo SPI exige
    // estructuralmente un buffer de recepción en cada transacción
    // full-duplex, exista o no contenido útil en lo que llega

// ==========================================
// VOLTAJES COMPARTIDOS ENTRE NÚCLEOS
// ==========================================

// Array donde se almacenan los voltajes: Core 0 escribe las posiciones
// 4 a 7 (U3 y U4), Core 1 escribe las posiciones 0 a 3 (U1 y U2).
// volatile obliga al compilador a leer siempre de memoria RAM real en
// vez de usar un valor cacheado en un registro del procesador, garantizando
// que cada núcleo ve el valor más reciente escrito por el otro
volatile float shared_voltages[8];

// Flag de sincronización entre núcleos: Core 0 lo pone a true cuando
// termina sus 4 voltajes; Core 1 lo pone a false para liberar a Core 0
// y que comience un nuevo ciclo. volatile por la misma razón que arriba
volatile bool core0_ready = false;

TaskHandle_t TaskCore0;

// Contador de ciclos de medida completados, se resetea cada 500 ms
// cuando el monitor USB calcula la frecuencia de muestreo real
uint32_t measurementCount = 0;

// Se inicializa con micros() en el setup y se compara con micros()
// en cada ciclo para saber cuándo han pasado 500000 microsegundos
// y toca imprimir por USB
uint32_t lastPrintTime = 0;

// ==========================================
// FUNCIONES I2C
// ==========================================

// Inicia una conversión en un ADS1115. Recibe el bus I2C, la dirección
// del ADS y el canal (0 para AIN0, 1 para AIN1). El bus se pasa por
// referencia para no duplicar código y que sirva para los cuatro ADS
void triggerConversion(TwoWire& wire, uint8_t addr, uint8_t channel) {
    uint8_t msb = (channel == 0) ? 0xC3 : 0xD3;
        // 0xC3: MUX = AIN0 vs GND, PGA = ±4.096V, modo single-shot,
        //        bit OS=1 para arrancar la conversión
        // 0xD3: mismo que 0xC3 pero MUX = AIN1 vs GND
    wire.beginTransmission(addr);  // prepara el buffer interno de Wire
    wire.write(0x01);              // puntero de registro → registro CONFIG
    wire.write(msb);               // canal + ganancia + modo + inicio
    wire.write(0xE3);              // 860 SPS (velocidad máxima del ADS1115)
                                    // → tiempo de conversión ≈ 1,16 ms
                                    // (se usa 1350 µs como margen de seguridad)
    wire.endTransmission();        // envía los tres bytes por I2C al ADS;
                                    // aquí arranca la conversión analógico-digital
}

// Lee el resultado de la última conversión completada en un ADS1115
// y lo devuelve convertido a voltios. Devuelve 0.0 si hay error I2C
float readAdcVolts(TwoWire& wire, uint8_t addr) {
    wire.beginTransmission(addr);
    wire.write(0x00);   // registro 0x00: registro de conversión de 16 bits
                         // con el resultado de la última conversión,
                         // en formato complemento a dos con signo
    if (wire.endTransmission() != 0) return 0.0f; // error I2C: el ADS
                                                    // no ha respondido con ACK
    if (wire.requestFrom(addr, (uint8_t)2) == 2) { // solicita los 2 bytes
                                                     // del resultado
        int16_t raw = (int16_t)((wire.read() << 8) | wire.read());
            // el primer byte leído es el MSB; se desplaza 8 posiciones
            // a la izquierda y se combina con el LSB. El cast a int16_t
            // garantiza la interpretación correcta en complemento a dos
            // para valores negativos (tensiones por debajo de GND)
        return raw * VOLTS_PER_BIT; // convierte el valor digital a voltios
    }
    return 0.0f;
}

// ==========================================
// TAREA DEL NÚCLEO 0 (Wire1: U3 y U4)
// V[4]=AIN0(U3) V[5]=AIN1(U3)
// V[6]=AIN0(U4) V[7]=AIN1(U4)
// ==========================================

// Esta tarea se ejecuta permanentemente en Core 0 en paralelo con el
// loop() de Core 1. No gestiona SPI ni DRDY: solo adquiere los voltajes
// de U3 y U4 y señala su disponibilidad mediante el flag core0_ready.
// Al no tener la carga de la transacción SPI, Core 0 termina su ciclo
// antes que Core 1 y lo espera bloqueado en el flag
void core0Code(void * pvParameters) {
    for (;;) {
        // Canal 0: dispara ambos ADS simultáneamente y espera
        triggerConversion(Wire1, ADDR_U1_U3, 0);
        triggerConversion(Wire1, ADDR_U2_U4, 0);

        uint32_t timer = micros();
        while (micros() - timer < ADC_CONVERSION_TIME_US) { yield(); }
            // yield() cede tiempo al scheduler de FreeRTOS para tareas
            // internas del sistema (watchdog, comunicaciones), evitando
            // que el microcontrolador se resetee por timeout

        shared_voltages[4] = readAdcVolts(Wire1, ADDR_U1_U3); // AIN0(U3)
        shared_voltages[6] = readAdcVolts(Wire1, ADDR_U2_U4); // AIN0(U4)

        // Canal 1: dispara y espera
        triggerConversion(Wire1, ADDR_U1_U3, 1);
        triggerConversion(Wire1, ADDR_U2_U4, 1);

        timer = micros();
        while (micros() - timer < ADC_CONVERSION_TIME_US) { yield(); }

        shared_voltages[5] = readAdcVolts(Wire1, ADDR_U1_U3); // AIN1(U3)
        shared_voltages[7] = readAdcVolts(Wire1, ADDR_U2_U4); // AIN1(U4)

        core0_ready = true;   // señala a Core 1 que los cuatro voltajes
                               // de este ciclo están disponibles en shared_voltages
        while (core0_ready) { yield(); } // se bloquea hasta que Core 1
                                          // consuma los datos y libere el flag
    }
}

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(921600); // puerto USB a 921600 baudios para verificación
                           // visual en el IDE; no interviene en la medida

    // Inicializa los dos buses I2C en modo rápido (400 kHz)
    Wire.begin(I2C0_SDA_PIN, I2C0_SCL_PIN);
    Wire1.begin(I2C1_SDA_PIN, I2C1_SCL_PIN);
    Wire.setClock(400000);  // Fast Mode — máximo soportado por el ADS1115
    Wire1.setClock(400000);

    pinMode(PIN_DRDY, OUTPUT);
    digitalWrite(PIN_DRDY, LOW); // estado de reposo; se pondrá en HIGH
                                  // justo antes de cada transacción SPI
                                  // para avisar a la Raspberry Pi

    // Configura el periférico SPI2 como esclavo en modo 0
    slave.setDataMode(SPI_MODE0);
    slave.begin(SPI2_HOST, GPIO_SCK, GPIO_MISO, GPIO_MOSI, GPIO_CS);

    // Inicializa el payload SPI con la firma correcta
    memset(&datosSalida, 0, sizeof(SpiPayload));
    datosSalida.firma = 0xABCD1234; // la Raspberry Pi comprueba este valor
                                     // para validar cada transacción
    memset(datosEntrada, 0, sizeof(datosEntrada));

    // Crea la tarea de Core 0 con 10 KB de stack y prioridad 1,
    // anclada al núcleo físico 0 del ESP32-S3
    xTaskCreatePinnedToCore(core0Code, "TaskCore0", 10000, NULL, 1, &TaskCore0, 0);

    lastPrintTime = micros();
    Serial.printf("\n[+] Arduino 1 — Solo voltajes. Iniciado.\n");
}

// ==========================================
// LOOP — NÚCLEO 1 (Wire: U1 y U2)
// V[0]=AIN0(U1) V[1]=AIN1(U1)
// V[2]=AIN0(U2) V[3]=AIN1(U2)
// ==========================================

// Core 1 gestiona U1 y U2 por Wire, la transacción SPI con la Raspberry Pi
// y la sincronización con Core 0. El orden de operaciones dentro del loop
// está diseñado para solapar la transacción SPI con el tiempo de conversión
// del canal 0 del ADC, de forma que la comunicación no añade latencia al ciclo
void loop() {

    // 1. DISPARAR CANAL 0 — U1 y U2
    // Inicia la conversión de AIN0 en ambos ADS y arranca el cronómetro
    triggerConversion(Wire, ADDR_U1_U3, 0);
    triggerConversion(Wire, ADDR_U2_U4, 0);
    uint32_t timer = micros();

    // 2. TRANSACCIÓN SPI CON LA RASPBERRY PI (solapada con la conversión ADC)
    // Se ejecuta mientras los ADS convierten, aprovechando los 1350 µs de espera
    slave.queue((uint8_t*)&datosSalida, datosEntrada, sizeof(SpiPayload));
        // encola el payload del ciclo ANTERIOR para enviarlo en la próxima
        // transacción; cast a uint8_t* porque la librería SPI trabaja con bytes
    digitalWrite(PIN_DRDY, HIGH); // avisa a la Raspberry Pi que hay datos listos
    slave.wait();                  // espera a que la Raspberry Pi complete la
                                   // lectura; a 1 MHz, 36 bytes tardan ≈ 0,288 ms,
                                   // muy por debajo de los 1350 µs de conversión
    digitalWrite(PIN_DRDY, LOW);  // indica fin de transacción
    memset(datosEntrada, 0, sizeof(datosEntrada)); // limpia buffer de entrada

    // 3. ESPERAR LO QUE RESTE DE CONVERSIÓN DEL CANAL 0
    while (micros() - timer < ADC_CONVERSION_TIME_US) { yield(); }

    // 4. LEER CANAL 0 — U1 y U2
    shared_voltages[0] = readAdcVolts(Wire, ADDR_U1_U3); // AIN0(U1)
    shared_voltages[2] = readAdcVolts(Wire, ADDR_U2_U4); // AIN0(U2)

    // 5. DISPARAR CANAL 1 — U1 y U2
    triggerConversion(Wire, ADDR_U1_U3, 1);
    triggerConversion(Wire, ADDR_U2_U4, 1);
    timer = micros();
    while (micros() - timer < ADC_CONVERSION_TIME_US) { yield(); }

    // 6. LEER CANAL 1 — U1 y U2
    shared_voltages[1] = readAdcVolts(Wire, ADDR_U1_U3); // AIN1(U1)
    shared_voltages[3] = readAdcVolts(Wire, ADDR_U2_U4); // AIN1(U2)

    // 7. SINCRONIZACIÓN CON NÚCLEO 0
    // Espera a que Core 0 termine sus cuatro voltajes; en la práctica
    // Core 0 suele terminar antes porque no tiene la carga de la SPI
    while (!core0_ready) { yield(); }

    // 8. EMPAQUETAR VOLTAJES PARA EL PRÓXIMO CICLO SPI
    datosSalida.firma = 0xABCD1234;
    for (int i = 0; i < 8; i++) datosSalida.voltajes[i] = shared_voltages[i];
        // los 8 voltajes se enviarán en el siguiente ciclo del loop,
        // con un desfase de ~4,5 ms entre la medida real y su envío —
        // despreciable frente a las constantes de tiempo del fenómeno físico

    measurementCount++;
    core0_ready = false; // libera a Core 0 para que arranque un nuevo ciclo

    // 9. MONITOR USB — calcula y muestra la frecuencia real cada 500 ms
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