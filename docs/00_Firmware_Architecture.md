# stravaV10 - Arquitetura de Firmware

**Versao:** 1.0
**Data:** 2025-11-25

---

## 1. Visao Geral do Sistema

### 1.1 Descricao

Computador GPS de ciclismo open-source com suporte a:
- Competicao em tempo real com Strava Segments (500+ segmentos)
- Smart trainer indoor via ANT+ FE-C
- Navegacao GPX com zoom dinamico
- Integracao Komoot (turn-by-turn navigation)
- Estimativa de potencia via algoritmos + barometro

### 1.2 Hardware

| Componente | Especificacao |
|------------|---------------|
| MCU | Nordic nRF52840 (Cortex-M4F @ 64MHz) |
| RAM | 256KB |
| Flash | 1MB |
| Display | Sharp Memory LCD LS027 (400x240, 1-bit) |
| RF | BLE 5.0 + ANT+ (SoftDevice s340) |
| GPS | Modulo UART (9600-115200 baud) |
| Sensores | Barometro, Acelerometro/Magnetometro (FXOS) |

### 1.3 Power Budget

| Modo | Consumo | Autonomia (1000mAh) |
|------|---------|---------------------|
| Indoor (sem GPS) | <8mA | 100+ horas |
| Outdoor (GPS ativo) | ~35mA | 30+ horas |

---

## 2. Diagrama de Arquitetura

### 2.1 Arquitetura de Alto Nivel

```
+===========================================================================+
|                              APPLICATION LAYER                             |
|                                 main.cpp                                   |
|  +------------------+  +------------------+  +------------------+          |
|  |   Task: boucle   |  |  Task: ls027     |  | Task: peripherals|          |
|  |   (GPS/FEC Loop) |  |  (Display Mgmt)  |  | (BLE/ANT+ Stack) |          |
|  |   Priority: 1    |  |   Priority: 2    |  |   Priority: 3    |          |
|  +--------+---------+  +--------+---------+  +--------+---------+          |
|           |                     |                     |                    |
+-----------+---------------------+---------------------+--------------------+
            |                     |                     |
            v                     v                     v
+===========================================================================+
|                               MODEL LAYER                                  |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
|  | BoucleCRS   |  | BoucleFEC   |  | Attitude    |  | Locator     |        |
|  | (Outdoor)   |  | (Indoor)    |  | (Fusion)    |  | (Position)  |        |
|  +------+------+  +------+------+  +------+------+  +------+------+        |
|         |                |                |                |               |
|         v                v                v                v               |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
|  | Segments    |  | SegManager  |  | Parcours    |  | ListePoints |        |
|  | (Strava)    |  | (Display)   |  | (GPX Nav)   |  | (History)   |        |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
+===========================================================================+
            |                     |                     |
            v                     v                     v
+===========================================================================+
|                                VUE LAYER                                   |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
|  | Vue         |  | VueCRS      |  | VueFEC      |  | VuePRC      |        |
|  | (Base)      |  | (Outdoor)   |  | (Indoor)    |  | (Parcours)  |        |
|  +------+------+  +------+------+  +------+------+  +------+------+        |
|         |                                                                  |
|         v                                                                  |
|  +-------------+  +-------------+  +-------------+                         |
|  | Adafruit_GFX|  | Cadrans     |  | Menus       |                         |
|  | (Graphics)  |  | (Widgets)   |  | (Navigation)|                         |
|  +-------------+  +-------------+  +-------------+                         |
+===========================================================================+
            |                     |                     |
            v                     v                     v
+===========================================================================+
|                              DRIVERS LAYER                                 |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
|  | ls027.c     |  | spi.c       |  | i2c.c       |  | uart.c      |        |
|  | (LCD)       |  | (SPI Bus)   |  | (I2C Bus)   |  | (GPS UART)  |        |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
|                                                                            |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
|  | fxos.c      |  | AltiBaro    |  | gpio.c      |  | neopixel.c  |        |
|  | (IMU)       |  | (Barometer) |  | (GPIO)      |  | (LED RGB)   |        |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
+===========================================================================+
            |                     |                     |
            v                     v                     v
+===========================================================================+
|                                RF LAYER                                    |
|  +---------------------------+  +---------------------------+              |
|  |         ANT+ Stack        |  |         BLE Stack         |              |
|  |  +------+  +------+       |  |  +------+  +------+       |              |
|  |  | HRM  |  | BSC  |       |  |  | NUS  |  | LNS  |       |              |
|  |  +------+  +------+       |  |  +------+  +------+       |              |
|  |  +------+  +------+       |  |  +------+  +------+       |              |
|  |  | FEC  |  | BS   |       |  |  | CP   |  |Komoot|       |              |
|  |  +------+  +------+       |  |  +------+  +------+       |              |
|  +---------------------------+  +---------------------------+              |
+===========================================================================+
            |                     |                     |
            v                     v                     v
+===========================================================================+
|                           NORDIC SDK LAYER                                 |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
|  | SoftDevice  |  | FreeRTOS    |  | FDS         |  | nrf_sdh     |        |
|  | s340 v6.1.1 |  | (Tasks)     |  | (Flash)     |  | (SD Handler)|        |
|  +-------------+  +-------------+  +-------------+  +-------------+        |
+===========================================================================+
            |
            v
+===========================================================================+
|                              HARDWARE LAYER                                |
|  +--------+  +--------+  +--------+  +--------+  +--------+  +--------+   |
|  |nRF52840|  |LS027   |  | GPS    |  | FXOS   |  | BME280 |  | SD Card|   |
|  | MCU    |  | LCD    |  | Module |  | IMU    |  | Baro   |  | SPI    |   |
|  +--------+  +--------+  +--------+  +--------+  +--------+  +--------+   |
+===========================================================================+
```

### 2.2 Fluxo de Tasks

```
                    +------------------+
                    |   RESET/BOOT     |
                    +--------+---------+
                             |
                             v
                    +------------------+
                    |   init_memory()  |
                    | - Estruturas     |
                    | - Error recovery |
                    +--------+---------+
                             |
                             v
                    +------------------+
                    |  RF Stack Init   |
                    | - SoftDevice     |
                    | - ANT+ profiles  |
                    | - BLE services   |
                    +--------+---------+
                             |
                             v
                    +------------------+
                    | scheduling_init()|
                    | - Create tasks   |
                    | - Init queues    |
                    +--------+---------+
                             |
                             v
                    +------------------+
                    | scheduler_start()|
                    +--------+---------+
                             |
         +-------------------+-------------------+
         |                   |                   |
         v                   v                   v
+----------------+  +----------------+  +------------------+
| boucle_task    |  | ls027_task     |  | peripherals_task |
| (Priority 1)   |  | (Priority 2)   |  | (Priority 3)     |
|                |  |                |  |                  |
| - Wait GPS evt |  | - Wait display |  | - BLE NUS tasks  |
| - Process loc  |  |   event        |  | - ANT+ tasks     |
| - Update segs  |  | - Clear buffer |  | - SD card tasks  |
| - Calc perf    |  | - Render UI    |  |                  |
| - Signal disp  |  | - SPI transfer |  |                  |
+----------------+  +----------------+  +------------------+
```

### 2.3 Maquina de Estados Global

```
                              +----------------+
                              |     INIT       |
                              | (Inicializacao)|
                              +-------+--------+
                                      |
                      +---------------+---------------+
                      |               |               |
                      v               v               v
               +------+------+ +------+------+ +------+------+
               |    CRS      | |    FEC      | |    PRC      |
               |  (Outdoor)  | |  (Indoor)   | |  (Parcours) |
               +------+------+ +------+------+ +------+------+
                      |               |               |
                      |               v               |
                      |        +------+------+        |
                      |        |   ZWIFT     |        |
                      |        | (Simulacao) |        |
                      |        +------+------+        |
                      |               |               |
                      +---------------+---------------+
                                      |
                                      v
                              +-------+--------+
                              |      MSC       |
                              | (Mass Storage) |
                              +----------------+
```

---

## 3. Modulos Principais

### 3.1 Boucle (Loop Principal)

**Arquivos:** `source/model/Boucle.cpp`, `BoucleCRS.cpp`, `BoucleFEC.cpp`

**Responsabilidade:** Orquestracao do loop principal de processamento

**Modos de operacao:**

| Modo | Descricao | GPS | ANT+ FEC | Sensores |
|------|-----------|-----|----------|----------|
| CRS | Outdoor cycling | ON | OFF | Baro + IMU |
| FEC | Indoor trainer | OFF | ON | - |
| PRC | GPX navigation | ON | OFF | Baro + IMU |
| Zwift | Simulacao | OFF | ON | - |

**Fluxo BoucleCRS:**

```
+------------------+
| Wait GPS Event   |
| (TASK_EVENT_LOC) |
+--------+---------+
         |
         v
+------------------+
| Get Position     |
| - GPS/LNS/SIM    |
+--------+---------+
         |
         v
+------------------+
| Update Attitude  |
| - Altitude       |
| - Climb          |
| - Power estimate |
+--------+---------+
         |
         v
+------------------+
| Process FXOS     |
| - Accelerometer  |
| - Magnetometer   |
+--------+---------+
         |
         v
+------------------+
| Kalman Fusion    |
| - Elevation      |
| - Slope          |
| - Alpha zero     |
+--------+---------+
         |
         v
+------------------+
| Update Segments  |
| for each segment:|
|  - Test activate |
|  - Calc perf     |
|  - Update avance |
+--------+---------+
         |
         v
+------------------+
| Notify Display   |
| (Cancel delay)   |
+------------------+
```

### 3.2 Attitude (Fusao Sensorial)

**Arquivo:** `source/model/Attitude.cpp`

**Responsabilidade:** Fusao de GPS + Barometro + IMU via Kalman Filter

**Modelo Kalman:**

```
Estado: X = [elevacao, slope, alpha_zero]

Matriz de Transicao:
        | 1  dl -dl |
  Phi = | 0   1   0 |
        | 0   0   1 |

Onde:
  - elevacao: altitude estimada (m)
  - slope: inclinacao do pitch (rad)
  - alpha_zero: bias do acelerometro (rad)
  - dl: deslocamento = velocidade * dt
```

**Parametros Kalman:**

```cpp
// Ruido do modelo (Q)
Q[0,0] = 0.03   // Elevacao
Q[1,1] = 0.10   // Slope (0.01 lento, 0.2 rapido)
Q[2,2] = 0.0002 // Alpha zero

// Ruido das observacoes (R)
R[0,0] = 1000   // Barometro
R[1,1] = 600    // Pitch do acelerometro
```

**Calculo de Potencia Estimada:**

```cpp
Power = Gravity + Rolling + Aero + Transmission

Gravity = 9.81 * weight * vertical_speed       // W
Rolling = 0.004 * 9.81 * weight * speed        // W
Aero    = 0.204 * speed^3                      // W (CdA assumido)
Transmission = 1.025                           // Rendimento 97.5%
```

### 3.3 Locator (Gerenciamento de Posicao)

**Arquivo:** `source/model/Locator.cpp`

**Responsabilidade:** Multiplexacao de fontes de posicao

**Prioridade de fontes:**

```
1. SIM (Simulacao)     - Maior prioridade (Zwift mode)
     |
     v (se age > 2000ms)
2. GPS (Interno)       - GPS UART
     |
     v (se age > 1500ms)
3. NRF (BLE LNS)       - Location Service via BLE
                       - Apenas se GPS nao tem fix
```

**Estrutura de dados:**

```cpp
struct SLoc {
    float lat;      // Latitude (degrees)
    float lon;      // Longitude (degrees)
    float alt;      // Altitude (m)
    float speed;    // Velocidade (km/h)
    float course;   // Direcao (degrees, 0=Norte)
};

struct SDate {
    uint32_t secj;      // Segundos do dia
    uint32_t date;      // DDMMYY
    uint32_t timestamp; // millis() no momento da captura
};
```

### 3.4 Segment (Strava Segments)

**Arquivo:** `source/routes/Segment.cpp`

**Responsabilidade:** Gerenciamento de segmentos Strava

**Estados do Segmento:**

```
    SEG_OFF (0)
        |
        | testActivation() == true
        v
    SEG_START (1)
        |
        | proximo ciclo
        v
    SEG_ON (2)
        |
        | testDesactivation() == true
        v
    SEG_FIN (-1)
        |
        | apos N ciclos
        v
    SEG_OFF (0)
```

**Algoritmo de Ativacao:**

```cpp
testActivation(ListePoints& mes_points) {
    // 1. Verifica distancia ao primeiro ponto
    if (dist(P1, minha_pos) > DIST_ACT) return 0;

    // 2. Calcula produto escalar (direcao)
    Vecteur minha_dir(pos_anterior, pos_atual);
    Vecteur seg_dir(P1, P2);

    minha_dir.normalize();
    seg_dir.normalize();

    float p_scal = dot(minha_dir, seg_dir);

    // 3. Ativa se direcao correta e aproximando
    if (p_scal > PSCAL_LIM &&
        dist(P2, pos)^2 < dist(P1, pos)^2 + dist(P1, P2)^2)
        return 1;

    return 0;
}
```

**Calculo de Performance:**

```cpp
// Avance = tempo de referencia - tempo atual
// > 0 significa mais rapido que o PR
// < 0 significa mais lento

float avance = tempo_ref_interpolado - tempo_desde_inicio;

// Progresso no segmento (0.0 a 1.0)
float progresso = indice_ponto_atual / total_pontos;
```

---

## 4. Comunicacao RF

### 4.1 ANT+ Stack

**Arquivo:** `rf/ant.c`

**Canais configurados:**

| Canal | Tipo | Device Type | Uso |
|-------|------|-------------|-----|
| 0 | HRM | 0x78 | Heart Rate Monitor |
| 1 | BSC | 0x79 | Bike Speed/Cadence |
| 2 | FEC | 0x11 | Fitness Equipment Control |
| 3 | GLASSES | Custom | Display remoto |
| 4 | BS | Wildcard | Background Search |

**Coexistencia ANT+/BLE:**

```cpp
// Desabilita busca de alta prioridade para nao interferir com BLE
void _ant_coex_setup(uint8_t channel) {
    readCfg.pucBuffer[0] &= ~0x08;
    sd_ant_coex_config_set(channel, &readCfg, NULL);
}
```

### 4.2 FE-C (Fitness Equipment Control)

**Arquivo:** `rf/fec.c`

**Paginas processadas (RX):**

| Pagina | Descricao | Dados |
|--------|-----------|-------|
| 16 | General FE Data | elapsed_time |
| 25 | Trainer Data | instant_power |

**Paginas transmitidas (TX - controle do trainer):**

| Pagina | Descricao | Uso |
|--------|-----------|-----|
| 49 | Target Power | Modo ERG |
| 51 | Track Resistance | Modo simulacao (slope) |

### 4.3 BLE Stack

**Arquivo:** `rf/ble_api6.c`

**Servicos Cliente (Central):**

| Servico | UUID | Funcao |
|---------|------|--------|
| NUS | Nordic UART | Comunicacao com stravaAP |
| CP | Cycling Power | Power meter BLE |
| LNS | Location Nav | GPS externo via BLE |
| Komoot | Custom | Navegacao turn-by-turn |

**Filtros de Scan:**

```cpp
// Conecta automaticamente a:
// 1. Nome "stravaAP"
// 2. Cycling Power Service
// 3. Location and Navigation Service
// 4. Komoot Service
```

---

## 5. GPS Management

**Arquivo:** `source/sensors/GPSMGMT.cpp`

### 5.1 Estados do GPS

```
    INIT
      |
      | sys_message recebida
      v
    RUN_SLOW (9600 baud)
      |
      | apos configuracao
      v
    RUN_FAST (115200 baud)
```

### 5.2 EPO (Extended Prediction Orbit)

Sistema de AGPS para cold start mais rapido:

```cpp
// Envia posicao conhecida (via LNS) ao GPS
void GPS_MGMT::startHostAidingEPO(sLocationData& loc) {
    snprintf(buffer, "$PMTK741,%.6f,%.6f,%d,%u,%u,%u,%02u,%02u,%02u",
             loc.lat, loc.lon, (int)loc.alt,
             year, month, day, hours, minutes, seconds);

    uart_send(buffer, len);
}
```

### 5.3 Watchdog do GPS

```cpp
void GPS_MGMT::runWDT(void) {
    // Se nao recebe dados por 3s, alterna baudrate
    if (millis() - last_toggled > 3000 &&
        gps.time.age() > 3000) {

        // Alterna entre 9600 e 115200 baud
        toggle_uart_speed();
    }
}
```

---

## 6. Display (Vue Layer)

### 6.1 Hardware LCD

**Arquivo:** `drivers/lcd/ls027.c`

**Especificacoes:**

| Parametro | Valor |
|-----------|-------|
| Resolucao | 400 x 240 pixels |
| Cores | 1-bit (preto/branco) |
| Interface | SPI |
| Refresh | VCOM toggle necessario |

**Protocolo SPI:**

```
Byte 0: Command
  - Bit 0: Write command
  - Bit 1: VCOM (deve alternar)
  - Bit 2: Clear

Byte 1+: Line data
  - Endereco da linha (1-240)
  - 50 bytes de dados (400 pixels / 8)
  - Dummy byte
```

### 6.2 Modos de Tela CRS

```
+----------------------+     +----------------------+
|   eVueCRSScreenInit  |     | eVueCRSScreenDataFull|
|   (Aguardando GPS)   |     | (Sem segmentos)      |
+----------------------+     +----------------------+
          |                            |
          | GPS fix                    | 0 segments
          v                            v
+----------------------+     +----------------------+
|  eVueCRSScreenDataSS |     | eVueCRSScreenDataDS  |
|  (1 segmento)        |     | (2+ segmentos)       |
+----------------------+     +----------------------+
```

### 6.3 Layout de Cadrans

**Screen 1 - Full Data (7 linhas, 2 colunas):**

```
+------------------+------------------+
| Dist     xxx.x km| Pwr        xxx W |
+------------------+------------------+
| Speed   xx.x km/h| Climb      xxx m |
+------------------+------------------+
| CAD       xxx rpm| HRM       xxx bpm|
+------------------+------------------+
| SL          xx % | VA      x.xx m/s |
+------------------+------------------+
| Next       xxx m | Alt      xxx.x m |
+------------------+------------------+
| Avg    xx.xx km/h| Score      xx.x  |
+------------------+------------------+
| STC        xx mA | SOC         xx % |
+------------------+------------------+
```

---

## 7. FDIR (Fault Detection, Isolation and Recovery)

### 7.1 Deteccao de Crash

```cpp
// Em Boucle.cpp - detecta crash anterior
if (m_app_error.special == SYSTEM_DESCR_POS_CRC) {
    // Exibe ultimo void executado antes do crash
    String message = "Last void: " + m_app_error.void_id;
    message += " in task: " + m_app_error.task_id;
    vue.addNotif("System", message, 6, eNotificationTypeComplete);
}
```

### 7.2 Restauracao de Estado

```cpp
// Em Attitude.cpp - restaura estado apos crash
if (m_app_error.saved_data.crc == calculate_crc(&saved_att)) {
    if (att.date.date == saved_att.date.date) {
        // Mesmo dia - restaura climb, distance, PR
        att.climb = saved_att.climb;
        att.dist = saved_att.dist;
        att.pr = saved_att.pr;
        vue.addNotif("FDIR", "Attitude restored", 6);
    }
}
```

### 7.3 Salvamento Periodico

```cpp
// A cada 15m de deslocamento, salva estado
if (att.dist > m_last_save_dist + 15.f) {
    // Salva buffer para SD card
    sd_save_pos_buffer(m_st_buffer, ATT_BUFFER_NB_ELEM);

    // Salva estado em caso de crash
    memcpy(&m_app_error.saved_data.att, &att, sizeof(SAtt));
    m_app_error.saved_data.crc = calculate_crc(&att);
}
```

---

## 8. Power Management

**Arquivo:** `source/scheduling/power_scheduler.cpp`

### 8.1 Auto-Shutdown

```cpp
#define POWER_SCHEDULER_MAX_IDLE_MIN  15

void power_scheduler__run(void) {
    // Auto-shutdown apos 15 min sem atividade
    if (millis() - m_last_ping > (60000 * POWER_SCHEDULER_MAX_IDLE_MIN)) {
        power_scheduler__shutdown();
    }
}
```

### 8.2 Sequencia de Shutdown

```cpp
void power_scheduler__shutdown(void) {
    // 1. Limpa dados salvos (evita restauracao invalida)
    memset(&m_app_error.saved_data.att, 0, sizeof(att));

    // 2. Desliga hardware
#if defined(PROTO_V11)
    stc.shutdown();     // Via STC3100 battery monitor
#else
    gpio_set(KILL_PIN); // Kill switch direto
#endif
}
```

---

## 9. Toolchain e Build

### 9.1 Requisitos

| Componente | Versao |
|------------|--------|
| GCC | 6 2017-q2-update |
| nRF SDK | V16.0 |
| SoftDevice | s340 V6.1.1 |
| Make | GNU Make |

### 9.2 Comandos de Build

```bash
cd pca10056/s340/armgcc

# Compilar
make

# Flash SoftDevice
make flash_softdevice   # Via J-Link
make dfu_softdevice     # Via USB DFU

# Flash Application
make flash              # Via J-Link
make dfu                # Via USB DFU
```

### 9.3 Modo DFU

Para entrar no modo DFU (Device Firmware Update):
1. Manter botao direito pressionado
2. Conectar cabo USB
3. Soltar botao apos LED indicar modo DFU

---

## 10. Dependencias

### 10.1 Bibliotecas Internas

| Biblioteca | Funcao |
|------------|--------|
| AdafruitGFX | Primitivas graficas |
| TinyGPS++ | Parsing NMEA |
| kalman | Filtro de Kalman linear/extendido |
| UDMatrix | Operacoes matriciais |
| WString | Strings dinamicas (Arduino-style) |

### 10.2 Nordic SDK

| Modulo | Funcao |
|--------|--------|
| nrf_sdh | SoftDevice handler |
| ble_* | BLE services |
| ant_* | ANT+ profiles |
| fds | Flash Data Storage |
| app_timer | Software timers |

---

## 11. Estrutura de Diretorios

```
stravaV10/
+-- main.cpp                    # Entry point, task creation
+-- app_config.h                # Configuracoes globais
+-- custom_board.h              # Definicao de hardware
|
+-- source/
|   +-- model/
|   |   +-- Boucle.cpp          # Loop principal
|   |   +-- BoucleCRS.cpp       # Modo outdoor
|   |   +-- BoucleFEC.cpp       # Modo indoor
|   |   +-- Attitude.cpp        # Fusao sensorial
|   |   +-- Locator.cpp         # Gerenciamento de posicao
|   |
|   +-- vue/
|   |   +-- Vue.cpp             # Base da UI
|   |   +-- VueCRS.cpp          # UI outdoor
|   |   +-- VueFEC.cpp          # UI indoor
|   |   +-- VuePRC.cpp          # UI navegacao
|   |
|   +-- routes/
|   |   +-- Segment.cpp         # Strava segments
|   |   +-- Parcours.cpp        # GPX navigation
|   |
|   +-- sensors/
|   |   +-- GPSMGMT.cpp         # GPS management
|   |   +-- AltiBaro.cpp        # Barometro
|   |
|   +-- scheduling/
|       +-- power_scheduler.cpp # Power management
|
+-- drivers/
|   +-- lcd/
|   |   +-- ls027.c             # Sharp LCD driver
|   +-- spi.c                   # SPI master
|   +-- i2c.c                   # I2C master
|   +-- uart.c                  # UART (GPS)
|   +-- gpio.c                  # GPIO wrapper
|
+-- rf/
|   +-- ant.c                   # ANT+ stack
|   +-- ble_api6.c              # BLE stack
|   +-- fec.c                   # FE-C protocol
|   +-- hrm.c                   # Heart rate monitor
|   +-- bsc.c                   # Bike speed/cadence
|
+-- libraries/
|   +-- AdafruitGFX/            # Graphics library
|   +-- TinyGPS++/              # NMEA parser
|   +-- kalman/                 # Kalman filter
|
+-- pca10056/s340/armgcc/       # Build files
    +-- Makefile
```

---

## 12. Trade-offs e Decisoes de Design

### 12.1 FreeRTOS vs Bare-metal

**Decisao:** FreeRTOS-style scheduling (via nRF SDK)

**Justificativa:**
- Multiplas tarefas concorrentes (GPS, display, RF)
- Prioridades definidas claramente
- Blocking waits para economia de energia
- nRF SDK ja fornece scheduling

### 12.2 Kalman vs Filtro Simples

**Decisao:** Extended Kalman Filter para fusao

**Justificativa:**
- Fusao otima de GPS + Barometro + IMU
- Estimativa de incerteza (covariance)
- Compensacao automatica de bias do acelerometro
- Custo: ~1ms por update (aceitavel)

### 12.3 ANT+ vs BLE para Sensores

**Decisao:** Ambos (dual-stack via s340)

**Justificativa:**
- ANT+: padrao de facto para sensores cycling
- BLE: conectividade com smartphone, Komoot
- SoftDevice s340 suporta ambos simultaneamente

### 12.4 Display Memory LCD vs OLED

**Decisao:** Sharp Memory LCD (LS027)

**Justificativa:**
- Consumo ultra-baixo (<50uA em modo estatico)
- Visibilidade em luz solar direta
- Nao requer backlight
- Trade-off: apenas 1-bit color

---

## 13. Pontos de Atencao

### 13.1 Critical Sections

- Acesso ao buffer SPI do LCD (mutex implementado)
- Atualizacao de estruturas compartilhadas entre tasks
- Ring buffer de UART (atomico)

### 13.2 Power Consumption

- GPS e maior consumidor (~30mA)
- Standby do GPS reduz para <8mA
- Display refresh consome ~5mA por 50ms

### 13.3 Memory Constraints

- Stack de tasks deve ser monitorado
- Alocacao dinamica evitada em runtime
- Buffer de pontos GPS limitado (HISTO_POINT_SIZE)

---

**Documento gerado automaticamente via analise de codigo**
