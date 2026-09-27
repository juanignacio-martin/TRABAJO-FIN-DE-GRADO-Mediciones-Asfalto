# ==========================================
# SERVIDOR — RASPBERRY PI 5, PCB2
# ==========================================
#
# Proyecto: Digital-PAVE (TED2021-131474B-I00)
# Autor:    Juan Ignacio Martín Moreno
# Tutor:    Giuseppe Conti (ETSIDI-UPM)
# Cotutor:  Federico Gulisano (ETSICCP-UPM)
#
# Descripción:
#   Servidor central del sistema de adquisición para la PCB2. Se ejecuta
#   en la Raspberry Pi 5 y es responsable de:
#     1. Recoger los voltajes crudos de los tres Arduinos por SPI,
#        disparado por interrupción hardware en cada pin DRDY.
#     2. Calcular las resistencias de cada canal según la topología
#        de jumpers activa (serie 14R, serie 8R+6R, paralelo 7/8 canales).
#     3. Servir la interfaz web al navegador del operador y mantener
#        actualizado el estado del sistema a ~30 Hz por WebSocket.
#     4. Grabar en memoria las medidas de una prueba y exportarlas
#        como CSV al navegador al detener la grabación.
#
# Diferencia clave respecto a la PCB1:
#   En la PCB1 los Arduinos calculaban las resistencias antes de enviarlas.
#   En la PCB2 los Arduinos envían únicamente voltajes crudos, y es este
#   servidor quien aplica el cálculo según la topología declarada por el
#   usuario en la interfaz web. Esto permite cambiar la topología de medida
#   en caliente sin recargar firmware en los Arduinos.
#
# Arquitectura de concurrencia:
#   - 3 recolectores SPI: funciones de callback disparadas por interrupción
#     DRDY (gpiozero), ejecutadas en el hilo de eventos de gpiozero.
#     Depositan datos crudos en sus colas respectivas sin hacer cálculo.
#   - 1 motor de procesamiento: hilo daemon en bucle a 1 ms que vacía
#     las colas, verifica las firmas, calcula resistencias y actualiza
#     el estado global compartido. Escribe en CSV si hay grabación activa.
#   - 1 servidor FastAPI (uvicorn): proceso async que sirve la interfaz
#     web y mantiene el WebSocket con el navegador.
#     Tarea de envío: empuja el estado global al navegador cada 33 ms.
#     Tarea de recepción: escucha comandos del operador (topología,
#     canales, iniciar/detener grabación).
#
# Dependencias (instalar en el entorno virtual env_spi):
#   spidev, gpiozero, fastapi, uvicorn
#
# Arranque:
#   source ~/TFG_Asfalto/env_spi/bin/activate
#   python3 7servidor.py
# ==========================================

import spidev
import time
import struct
import queue
import threading
import json
import asyncio
from gpiozero import DigitalInputDevice
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
import uvicorn

# ==========================================
# 1. CONSTANTES DE CÁLCULO POR DEFECTO
# ==========================================

# Valores nominales de las resistencias de referencia soldadas en la PCB2.
# Los dos primeros son seleccionables por el usuario desde la interfaz web;
# el del paralelo es fijo porque las resistencias están integradas en la placa
# y no hay jumper que permita sustituirlas externamente.
R_REF_JP3           = 10.1  # R_ref de JP3: arranque de la cadena de 6R (Ω)
R_REF_JP5_DEFECTO   = 10.1  # R_ref de JP5: arranque de toda la cadena serie (Ω)
R_REF_PARALELO_FIJA = 10.1  # R_ref soldada en cada canal del paralelo (U9-U12), no seleccionable (Ω)

# ==========================================
# 2. ESTADO GLOBAL COMPARTIDO
# ==========================================

# Diccionario central que concentra todo lo que necesitan conocer
# el motor de procesamiento y el servidor web. El motor escribe los
# resultados de cálculo; el servidor web los lee y los envía al navegador.
# No se usa un Lock aquí porque Python garantiza la atomicidad de las
# asignaciones de objetos simples (GIL), y el patrón de acceso
# (un escritor, un lector) no genera condiciones de carrera en la práctica.
estado_actual = {
    # Voltajes crudos recibidos de cada Arduino (8 por Arduino)
    "voltajes_a1": [0.0] * 8,
    "voltajes_a2": [0.0] * 8,
    "voltajes_a3": [0.0] * 8,

    # Resistencias calculadas — solo se rellenan las del modo activo;
    # las del modo inactivo mantienen su último valor calculado
    "serie14":  [0.0] * 14,
    "serie8":   [0.0] * 8,
    "serie6":   [0.0] * 6,
    "paralelo": [0.0] * 8,

    # Configuración de topología (refleja la posición física de los jumpers)
    # El usuario la declara desde la interfaz web; si no coincide con los
    # jumpers reales, los cálculos serán incorrectos
    "modo_serie":        "8_6",      # "8_6" → dos cadenas independientes de 8R y 6R
                                      # "14"  → cadena única de 14R (JP6 en posición continua)
    "modo_paralelo":     "vref",     # "vref" → JP7 mide V_REF real, 7 canales de asfalto
                                      # "fija" → JP7 como canal adicional, 8 canales, V_REF constante
    "jp4":               "sin_usar", # "sin_usar" | "soldada" | "personalizada"
    "r_jp4":             0.0,        # valor en Ω de la R_ref de JP4, si se usa
    "r_ref_jp3":         R_REF_JP3,
    "r_ref_jp5":         R_REF_JP5_DEFECTO,
    "v_ref_fija_paralelo": 3.41,     # tensión de referencia constante (V), usada solo si modo_paralelo == "fija"

    # Número de canales activos en cada configuración.
    # Permite medir un subconjunto de la cadena sin necesidad de cambiar
    # el cableado: el usuario cierra el GND en el borne correspondiente
    # al último canal que quiere medir y declara aquí cuántos son
    "activas_serie14":   14,
    "activas_serie8":    8,
    "activas_serie6":    6,
    "activas_paralelo":  7,          # 7 si modo_paralelo=="vref", 8 si "fija"

    "hz":       [0, 0, 0],  # frecuencia de muestreo real de cada Arduino (Hz), calculada cada segundo
    "grabando": False,
}

# Los Arduinos siempre miden y envían los 8 voltajes de sus canales físicos.
# El recorte de "activas" es solo de cálculo y visualización en la Raspberry Pi.
ACTIVAS_A1 = 8
ACTIVAS_A2 = 8
ACTIVAS_A3 = 8

hz_count = [0, 0, 0]  # contador interno de paquetes recibidos por Arduino en el último segundo

# ==========================================
# 3. CONTROL DE GRABACIÓN EN MEMORIA
# ==========================================

# La grabación acumula filas en RAM en vez de escribir directamente en disco,
# para evitar la latencia de escritura en tarjeta SD durante la medida.
# Al detener, el CSV completo se envía de una vez al navegador por WebSocket,
# que lo descarga como archivo. lock_grabacion protege la lista compartida
# entre el motor de procesamiento (escritor) y el manejador WebSocket (lector)
grabacion_activa = False
filas_acumuladas = []
lock_grabacion    = threading.Lock()

def construir_cabecera():
    """Genera los nombres de columna del CSV según la topología activa."""
    modo = estado_actual["modo_serie"]
    if modo == "14":
        cols_serie = [f"R14_{i+1}" for i in range(14)]
    else:
        cols_serie = [f"R8_{i+1}" for i in range(8)] + [f"R6_{i+1}" for i in range(6)]

    n_par = 7 if estado_actual["modo_paralelo"] == "vref" else 8
    cols_par = [f"RP_{i+1}" for i in range(n_par)]

    return ["Hora", "Tiempo(s)"] + cols_serie + cols_par

def iniciar_grabacion():
    """Limpia el buffer y activa el flag de grabación de forma atómica."""
    global grabacion_activa, filas_acumuladas
    with lock_grabacion:
        if grabacion_activa:
            return
        filas_acumuladas = []
        grabacion_activa = True
        estado_actual["grabando"] = True
        print("[CSV] Grabación iniciada en memoria.")

def detener_grabacion():
    """Desactiva el flag de grabación sin borrar el buffer, para poder exportarlo."""
    global grabacion_activa
    with lock_grabacion:
        if not grabacion_activa:
            return
        grabacion_activa = False
        estado_actual["grabando"] = False
        print(f"[CSV] Grabación detenida. {len(filas_acumuladas)} filas acumuladas.")

def exportar_csv():
    """Serializa el buffer acumulado en formato CSV listo para descargar."""
    with lock_grabacion:
        cabecera = construir_cabecera()
        lineas = [",".join(map(str, cabecera))]
        for fila in filas_acumuladas:
            lineas.append(",".join(map(str, fila)))
        return "\n".join(lineas)

# ==========================================
# 4. CÁLCULO DE RESISTENCIAS
# ==========================================

def _aplicar_jp4_si_corresponde(v_nodo_final, I_cadena, activas_totales_del_tramo, activas_actuales):
    """
    Corrección de JP4 para el extremo final de la cadena.

    Cuando JP4 está conectado (soldado o personalizado) y el usuario mide
    hasta el último canal físico del tramo, la resistencia de JP4 queda
    en paralelo con la última R de asfalto, derivando parte de la corriente
    de la cadena. Sin corrección, esa derivación inflaría el valor calculado
    de la última R. Esta función descuenta la corriente derivada por JP4
    antes de calcular la resistencia real del último tramo.
    Si JP4 no se usa, o si el usuario ha movido el GND antes del final
    físico del tramo, no hay derivación que corregir y se devuelve
    el cálculo directo.
    """
    jp4 = estado_actual["jp4"]
    es_final_fisico = (activas_actuales == activas_totales_del_tramo)

    if jp4 == "sin_usar" or not es_final_fisico:
        return v_nodo_final / I_cadena

    r_jp4 = estado_actual["r_jp4"]
    if r_jp4 <= 0:
        # Sin valor válido para R_ref de JP4, no se puede corregir.
        # Se devuelve el cálculo sin corrección en vez de fallar
        return v_nodo_final / I_cadena

    # Corriente que se fuga por JP4 (ley de Ohm sobre R_jp4)
    I_derivada = v_nodo_final / r_jp4
    I_real     = I_cadena - I_derivada  # corriente real que atraviesa la última R de asfalto
    if I_real <= 0:
        return 0.0
    return v_nodo_final / I_real


def calcular_serie8(v_a1, v_a2, activas):
    """
    Calcula las resistencias de la cadena de 8R (Arduino 1 + primer nodo de Arduino 2).

    Circuito: 3,3V → R_ref(JP5) → 7 nodos de Arduino 1 → AIN0(U5) → borne4 (GND, sin medir).
    La corriente de la cadena se obtiene de la caída en R_ref(JP5): V[0]-V[1] / R_ref_jp5.
    El último nodo (borne4 = GND) se asume 0V sin que ningún ADS lo mida directamente.
    JP4 no afecta a esta cadena: su nodo físico está en el tramo de la cadena de 6R.
    """
    r8 = [0.0] * 8
    drop_ref = v_a1[0] - v_a1[1]
    if drop_ref > 0.001 and activas > 0:
        I = drop_ref / estado_actual["r_ref_jp5"]
        # nodos[0..6] son los nodos de Arduino 1; nodos[7] es el primer nodo de Arduino 2
        nodos = [v_a1[1], v_a1[2], v_a1[3], v_a1[4], v_a1[5], v_a1[6], v_a1[7], v_a2[0]]
        for i in range(activas - 1):
            r8[i] = (nodos[i] - nodos[i+1]) / I
        v_final = nodos[activas - 1]
        r8[activas - 1] = v_final / I  # último nodo: tensión respecto a GND / I
    return r8


def calcular_serie6(v_a2, activas):
    """
    Calcula las resistencias de la cadena de 6R (autónoma dentro de Arduino 2).

    Circuito: AIN1(U5)=3,3V propio → R_ref(JP3) → 6 nodos → borne10 → [JP4] → borne9 (GND).
    Esta cadena es completamente independiente de la de 8R: tiene su propia excitación
    (3,3V en AIN1 de U5) y su propia R_ref (JP3).
    Si JP4 está activo, la última R de asfalto queda en paralelo con R_jp4 y se aplica
    la corrección correspondiente.
    """
    r6 = [0.0] * 6
    drop_ref = v_a2[1] - v_a2[2]
    if drop_ref > 0.001 and activas > 0:
        I = drop_ref / estado_actual["r_ref_jp3"]
        nodos = [v_a2[2], v_a2[3], v_a2[4], v_a2[5], v_a2[6], v_a2[7]]
        for i in range(activas - 1):
            r6[i] = (nodos[i] - nodos[i+1]) / I
        v_final = nodos[activas - 1]
        r6[activas - 1] = _aplicar_jp4_si_corresponde(v_final, I, 6, activas)
    return r6


def calcular_serie14(v_a1, v_a2, activas):
    """
    Calcula las resistencias de la cadena única de 14R (JP6 en modo continuo).

    Circuito: 3,3V → R_ref(JP5) → 7 nodos de Arduino 1 → 7 nodos de Arduino 2 → [JP4] → GND.
    JP6 en posición continua hace que la cadena de Arduino 1 se prolongue sin corte
    hasta Arduino 2, formando una cadena única de 14 tramos con una única corriente común.
    La misma R_ref(JP5) sirve como sensor de corriente para toda la cadena.
    """
    r14 = [0.0] * 14
    drop_ref = v_a1[0] - v_a1[1]
    if drop_ref > 0.001 and activas > 0:
        I = drop_ref / estado_actual["r_ref_jp5"]
        # 7 nodos de Arduino 1 + 8 nodos de Arduino 2 = 15 nodos totales
        # (el último nodo de Arduino 2 es el extremo de la cadena, antes de JP4/GND)
        nodos = [v_a1[1], v_a1[2], v_a1[3], v_a1[4], v_a1[5], v_a1[6], v_a1[7],
                 v_a2[0], v_a2[1], v_a2[2], v_a2[3], v_a2[4], v_a2[5], v_a2[6], v_a2[7]]
        for i in range(activas - 1):
            r14[i] = (nodos[i] - nodos[i+1]) / I
        v_final = nodos[activas - 1]
        r14[activas - 1] = _aplicar_jp4_si_corresponde(v_final, I, 14, activas)
    return r14


def calcular_paralelo(v_a3, activas):
    """
    Calcula las resistencias de los canales en paralelo (Arduino 3).

    Cada canal es un divisor de tensión independiente:
      R_asfalto = R_ref × V_nodo / (V_ref - V_nodo)

    Modo 'vref': AIN0(U9) mide V_REF real en tiempo real → 7 canales de asfalto (RP1-RP7).
    Modo 'fija': AIN0(U9) es un canal de asfalto adicional → 8 canales (RP1-RP8),
                 y V_REF se toma de la constante calibrada v_ref_fija_paralelo.

    Códigos de error (valores negativos, nunca posibles en una R real):
      -1.0 → tensión próxima a V_REF: canal desconectado o resistencia de asfalto muy alta
      -2.0 → tensión próxima a cero: cortocircuito o ausencia de R_ref
    """
    modo = estado_actual["modo_paralelo"]

    if modo == "vref":
        rp = [0.0] * 7
        v_ref = v_a3[0]  # tensión real de alimentación de los divisores, medida en tiempo real
        if v_ref < 0.1:
            return rp  # V_REF no disponible: JP7 posiblemente no conectado
        for i in range(activas):
            v = v_a3[i + 1]  # voltaje del nodo intermedio del divisor del canal i
            if 0.005 < v < (v_ref - 0.05):
                rp[i] = (v * R_REF_PARALELO_FIJA) / (v_ref - v)
            elif v >= (v_ref - 0.05):
                rp[i] = -1.0  # canal abierto o sin resistencia de asfalto
            else:
                rp[i] = -2.0  # cortocircuito o falta de R_ref
        return rp
    else:
        rp = [0.0] * 8
        v_ref = estado_actual["v_ref_fija_paralelo"]  # constante calibrada por el usuario
        for i in range(activas):
            v = v_a3[i]
            if 0.005 < v < (v_ref - 0.05):
                rp[i] = (v * R_REF_PARALELO_FIJA) / (v_ref - v)
            elif v >= (v_ref - 0.05):
                rp[i] = -1.0
            else:
                rp[i] = -2.0
        return rp

# ==========================================
# 5. MOTOR HARDWARE: RECOLECTORES SPI Y MOTOR DE PROCESAMIENTO
# ==========================================

# Una cola por Arduino para desacoplar los recolectores SPI (productores,
# disparados por interrupción) del motor de procesamiento (consumidor,
# en su propio hilo). maxsize=1000 evita que un retraso del motor cause
# un crecimiento ilimitado de memoria; si la cola está llena, el dato
# más antiguo se descarta silenciosamente.
cola_a1  = queue.Queue(maxsize=1000)
cola_a2  = queue.Queue(maxsize=1000)
cola_a3  = queue.Queue(maxsize=1000)

# spi_lock protege el bus SPI físico de la Raspberry Pi frente a accesos
# concurrentes de los tres recolectores. Aunque cada Arduino usa un chip select
# distinto (CE0, CE1, CE2), la biblioteca spidev no garantiza seguridad de
# acceso concurrente desde varios hilos sobre el mismo controlador SPI.
spi_lock = threading.Lock()

spi1 = spidev.SpiDev(); spi2 = spidev.SpiDev(); spi3 = spidev.SpiDev()
drdy1 = None; drdy2 = None; drdy3 = None

u_tiempo_1 = u_tiempo_2 = u_tiempo_3 = 0.0

# Formato de desempaquetado: little-endian, un uint32 (firma) + 8 floats (voltajes)
# = 36 bytes, sin padding. Debe coincidir exactamente con el struct SpiPayload
# definido en los firmwares de Arduino con #pragma pack(push, 1)
DESEMPAQUETADOR = struct.Struct('<I8f')

def recolector_spi_1():
    """
    Callback disparado por interrupción en DRDY del Arduino 1 (GPIO 23).
    Lee el payload SPI y lo deposita en cola_a1.
    El intervalo mínimo de 2 ms entre llamadas evita lecturas múltiples
    por rebotes en la señal DRDY.
    """
    global u_tiempo_1
    t = time.perf_counter()
    if (t - u_tiempo_1) < 0.002 or not drdy1.is_active: return
    u_tiempo_1 = t
    caja = [0] * struct.calcsize('<I8f')  # buffer de 36 bytes que se envía al Arduino
                                           # (contenido indiferente: SPI full-duplex exige
                                           # enviar algo mientras se recibe)
    with spi_lock: resp = spi1.xfer2(caja)
    try: cola_a1.put_nowait(bytes(resp))
    except queue.Full: pass  # si la cola está llena, se descarta este paquete

def recolector_spi_2():
    """Ídem para Arduino 2 (GPIO 24, CE1)."""
    global u_tiempo_2
    t = time.perf_counter()
    if (t - u_tiempo_2) < 0.002 or not drdy2.is_active: return
    u_tiempo_2 = t
    caja = [0] * struct.calcsize('<I8f')
    with spi_lock: resp = spi2.xfer2(caja)
    try: cola_a2.put_nowait(bytes(resp))
    except queue.Full: pass

def recolector_spi_3():
    """Ídem para Arduino 3 (GPIO 25, CE2)."""
    global u_tiempo_3
    t = time.perf_counter()
    if (t - u_tiempo_3) < 0.002 or not drdy3.is_active: return
    u_tiempo_3 = t
    caja = [0] * struct.calcsize('<I8f')
    with spi_lock: resp = spi3.xfer2(caja)
    try: cola_a3.put_nowait(bytes(resp))
    except queue.Full: pass


def motor_procesamiento():
    """
    Hilo daemon que vacía las tres colas, verifica las firmas, calcula
    resistencias y actualiza el estado global.

    Se ejecuta en bucle a 1 ms (time.sleep(0.001)). En cada iteración:
      1. Vacía completamente las tres colas, procesando todos los paquetes
         disponibles (puede haber varios si el hilo se retrasó).
      2. Si hubo datos nuevos, recalcula las resistencias según el modo activo
         y actualiza estado_actual, que el servidor web leerá en la siguiente
         iteración de envío (cada 33 ms).
      3. Acumula la fila en filas_acumuladas si hay grabación activa.
      4. Cada segundo actualiza el contador de Hz de cada Arduino.

    La verificación de firma (0xABCD1234) descarta paquetes corruptos o
    transacciones SPI que llegaron desincronizadas.
    """
    global hz_count
    print("[MOTOR] Motor iniciado.")
    tiempo_inicio     = time.perf_counter()
    ultimo_calculo_hz = time.time()
    v_a1 = [0.0] * 8
    v_a2 = [0.0] * 8
    v_a3 = [0.0] * 8

    while True:
        timestamp        = time.perf_counter() - tiempo_inicio
        hay_datos_nuevos = False

        # Vaciar cola de Arduino 1
        while not cola_a1.empty():
            try:
                d = DESEMPAQUETADOR.unpack(cola_a1.get_nowait())
                if d[0] == 0xABCD1234:  # firma válida → paquete íntegro
                    v_a1 = list(d[1:9]); hay_datos_nuevos = True
                    hz_count[0] += 1
            except struct.error: pass  # tamaño incorrecto → descarta

        # Vaciar cola de Arduino 2
        while not cola_a2.empty():
            try:
                d = DESEMPAQUETADOR.unpack(cola_a2.get_nowait())
                if d[0] == 0xABCD1234:
                    v_a2 = list(d[1:9]); hay_datos_nuevos = True
                    hz_count[1] += 1
            except struct.error: pass

        # Vaciar cola de Arduino 3
        while not cola_a3.empty():
            try:
                d = DESEMPAQUETADOR.unpack(cola_a3.get_nowait())
                if d[0] == 0xABCD1234:
                    v_a3 = list(d[1:9]); hay_datos_nuevos = True
                    hz_count[2] += 1
            except struct.error: pass

        if hay_datos_nuevos:
            modo = estado_actual["modo_serie"]

            # Calcula resistencias según topología activa y actualiza estado_actual
            if modo == "14":
                r14 = calcular_serie14(v_a1, v_a2, estado_actual["activas_serie14"])
                estado_actual["serie14"] = r14
                fila_serie = r14
            else:
                r8 = calcular_serie8(v_a1, v_a2, estado_actual["activas_serie8"])
                r6 = calcular_serie6(v_a2, estado_actual["activas_serie6"])
                estado_actual["serie8"] = r8
                estado_actual["serie6"] = r6
                fila_serie = r8 + r6

            rp = calcular_paralelo(v_a3, estado_actual["activas_paralelo"])
            estado_actual["paralelo"] = rp

            # Actualiza también los voltajes crudos en el estado,
            # para que estén disponibles en la interfaz web si se necesitan
            estado_actual["voltajes_a1"] = v_a1
            estado_actual["voltajes_a2"] = v_a2
            estado_actual["voltajes_a3"] = v_a3

            # Grabación en memoria: protegida por lock_grabacion porque
            # el manejador WebSocket puede leer filas_acumuladas concurrentemente
            with lock_grabacion:
                if grabacion_activa:
                    hora = time.strftime("%H:%M:%S")
                    fila = ([hora, round(timestamp, 4)]
                            + [round(v, 4) for v in fila_serie]
                            + [round(v, 4) for v in rp])
                    filas_acumuladas.append(fila)

        # Actualiza Hz una vez por segundo
        if time.time() - ultimo_calculo_hz >= 1.0:
            estado_actual["hz"] = hz_count.copy()
            hz_count              = [0, 0, 0]
            ultimo_calculo_hz     = time.time()

        time.sleep(0.001)  # cede 1 ms al sistema operativo entre iteraciones

# ==========================================
# 6. SERVIDOR WEB FASTAPI
# ==========================================

app = FastAPI()

@app.get("/")
async def get_index():
    """Sirve el archivo HTML de la interfaz web al navegador del operador."""
    with open("7index.html", "r", encoding="utf-8") as f:
        return HTMLResponse(content=f.read())

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    """
    Gestiona la conexión WebSocket con el navegador.

    Dos flujos asíncronos en paralelo dentro de la misma conexión:
      - tarea_envio: envía el estado global al navegador cada 33 ms (~30 Hz).
        La frecuencia de 30 Hz es deliberadamente inferior a la de adquisición
        (~540 muestras/s combinadas) porque el ojo humano no distingue
        actualizaciones por encima de 25-30 Hz, y enviar a la cadencia real
        sobrecargaría la red y el navegador sin mejora perceptible.
      - recepción: escucha comandos del operador (configurar_topologia,
        configurar_canales, iniciar_grabacion, detener_grabacion).
        Al detener la grabación, envía el CSV completo por WebSocket
        para que el navegador lo descargue como archivo.
    """
    await websocket.accept()

    async def enviar_datos():
        try:
            while True:
                await websocket.send_text(json.dumps({"tipo": "datos", **estado_actual}))
                await asyncio.sleep(0.033)  # ~30 Hz
        except asyncio.CancelledError: pass
        except Exception: pass

    tarea_envio = asyncio.create_task(enviar_datos())

    try:
        while True:
            datos_recibidos = await websocket.receive_text()
            comando = json.loads(datos_recibidos)
            accion = comando.get("accion")

            if accion == "configurar_topologia":
                # Actualiza el estado de los jumpers declarado por el usuario.
                # Si el valor no viene en el comando, mantiene el actual (get con default)
                estado_actual["modo_serie"]    = comando.get("modo_serie", estado_actual["modo_serie"])
                estado_actual["modo_paralelo"] = comando.get("modo_paralelo", estado_actual["modo_paralelo"])
                estado_actual["jp4"]           = comando.get("jp4", estado_actual["jp4"])
                estado_actual["r_jp4"]         = float(comando.get("r_jp4", estado_actual["r_jp4"]))
                estado_actual["r_ref_jp3"]     = float(comando.get("r_ref_jp3", estado_actual["r_ref_jp3"]))
                estado_actual["r_ref_jp5"]     = float(comando.get("r_ref_jp5", estado_actual["r_ref_jp5"]))
                estado_actual["v_ref_fija_paralelo"] = float(
                    comando.get("v_ref_fija_paralelo", estado_actual["v_ref_fija_paralelo"])
                )
                print(f"[TOPOLOGIA] Actualizada: {estado_actual['modo_serie']} / "
                      f"{estado_actual['modo_paralelo']} / JP4={estado_actual['jp4']}")

            elif accion == "configurar_canales":
                # Actualiza cuántos canales se calculan y visualizan en cada modo
                estado_actual["activas_serie14"]  = int(comando.get("serie14",  estado_actual["activas_serie14"]))
                estado_actual["activas_serie8"]   = int(comando.get("serie8",   estado_actual["activas_serie8"]))
                estado_actual["activas_serie6"]   = int(comando.get("serie6",   estado_actual["activas_serie6"]))
                estado_actual["activas_paralelo"] = int(comando.get("paralelo", estado_actual["activas_paralelo"]))

            elif accion == "iniciar_grabacion":
                iniciar_grabacion()

            elif accion == "detener_grabacion":
                detener_grabacion()
                csv_texto = exportar_csv()
                # Envía el CSV completo como mensaje WebSocket; el navegador
                # lo recibe e inicia la descarga automáticamente como archivo
                await websocket.send_text(json.dumps({"tipo": "csv_export", "contenido": csv_texto}))
                print("[CSV] Enviado al navegador.")

    except WebSocketDisconnect:
        tarea_envio.cancel()  # cancela la tarea de envío al desconectarse el navegador

# ==========================================
# 7. PUNTO DE ENTRADA
# ==========================================
if __name__ == "__main__":
    print("==================================================")
    print("  Panel de Control TFG — PCB2 — Topología completa")
    print("==================================================")

    # Abre los tres canales SPI: bus 0, chip selects CE0/CE1/CE2
    # (correspondientes a los tres Arduinos) a 1 MHz en modo 0
    spi1.open(0, 0); spi1.max_speed_hz = 1000000; spi1.mode = 0
    spi2.open(0, 1); spi2.max_speed_hz = 1000000; spi2.mode = 0
    spi3.open(0, 2); spi3.max_speed_hz = 1000000; spi3.mode = 0

    # Configura los tres pines DRDY como entradas digitales sin pull-up
    # (la señal la maneja el Arduino; pull_up=False evita conflicto)
    drdy1 = DigitalInputDevice(23, pull_up=False)
    drdy2 = DigitalInputDevice(24, pull_up=False)
    drdy3 = DigitalInputDevice(25, pull_up=False)

    # Registra los recolectores como callbacks de flanco de subida.
    # gpiozero los ejecuta en su propio hilo de eventos cuando detecta
    # la activación del pin, sin polling continuo.
    drdy1.when_activated = recolector_spi_1
    drdy2.when_activated = recolector_spi_2
    drdy3.when_activated = recolector_spi_3

    # Llamada manual inicial: si algún Arduino ya tenía DRDY en HIGH
    # cuando arrancó el servidor (p.ej. se encendió antes que la RPi),
    # el flanco de subida ya ocurrió y la interrupción no se dispararía.
    # Esta comprobación garantiza que se lee ese primer paquete pendiente.
    if drdy1.is_active: recolector_spi_1()
    if drdy2.is_active: recolector_spi_2()
    if drdy3.is_active: recolector_spi_3()

    # Lanza el motor de procesamiento como hilo daemon:
    # daemon=True garantiza que el hilo se cierra automáticamente
    # si el programa principal termina (p.ej. por Ctrl+C),
    # sin quedarse bloqueado en su bucle infinito
    threading.Thread(target=motor_procesamiento, daemon=True).start()

    print("\n[+] Hardware configurado.")
    print("[+] Ve a tu navegador: IP de la Raspberry, puerto 8005")
    uvicorn.run(app, host="0.0.0.0", port=8005, access_log=False)