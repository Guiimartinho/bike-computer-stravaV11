# stravaV10 - Code Review: Violacoes MISRA e Padroes de Firmware

**Versao:** 1.0
**Data:** 2025-11-25
**Objetivo:** Identificar problemas de codigo para migracao para Zephyr RTOS

---

## 1. Sumario Executivo

Este documento identifica violacoes de:
- MISRA C:2012 / MISRA C++:2008
- Padroes de firmware do 0.memory
- Best practices para sistemas embarcados safety-critical

**Severidade das violacoes:**
- **CRITICO**: Pode causar undefined behavior, crash, ou seguranca comprometida
- **ALTO**: Violacao clara de MISRA ou padrao que afeta manutencao/confiabilidade
- **MEDIO**: Viola best practices, dificulta debug ou portabilidade
- **BAIXO**: Estilo ou convencao que melhora legibilidade

---

## 2. Violacoes Criticas

### 2.1 Variaveis Nao Inicializadas

**MISRA C:2012 Rule 9.1** - Valor de objeto nao inicializado nao deve ser lido

| Arquivo | Linha | Codigo Problematico |
|---------|-------|---------------------|
| `ant.c` | 51-52 | `uint16_t m_last_device_id; uint8_t m_last_rssi = 0;` |
| `BoucleCRS.cpp` | 44 | `int ret;` sem inicializacao |
| `Attitude.cpp` | 126-129 | `static sKalmanExtFeed feed;` sem inicializacao explicita |
| `fec.c` | 69-71 | `uint16_t pusDeviceNumber = 0;` mas outros nao |

**Correcao:**
```c
// RUIM
int ret;
if ((ret = init_liste_segments())) { ... }

// BOM
int ret = 0;
ret = init_liste_segments();
if (ret != 0) { ... }
```

### 2.2 Uso de `static` em Variaveis Locais para Estado

**MISRA C:2012 Rule 8.9** - Escopo de objetos deve ser minimizado

| Arquivo | Linha | Problema |
|---------|-------|----------|
| `Attitude.cpp` | 126 | `static sKalmanExtFeed feed;` dentro de funcao |
| `Attitude.cpp` | 127 | `static uint32_t m_update_time = 0;` |
| `Attitude.cpp` | 221-222 | `static float alti_prev`, `static float alpha_bar` |
| `Locator.cpp` | 21 | `static bool m_is_gps_updated = false;` global oculto |
| `GPSMGMT.cpp` | 46-51 | Multiplos `static` globais sem prefixo |

**Problema:** Estado oculto dificulta teste e debug. Em RTOS, pode causar race conditions.

**Correcao:** Mover para estrutura de contexto ou membro de classe:
```cpp
// RUIM
void Attitude::computeFusion(void) {
    static uint32_t m_update_time = 0;  // Estado oculto!
    ...
}

// BOM - Estado explicito no objeto
class Attitude {
private:
    uint32_t m_update_time = 0;  // Membro da classe
};
```

### 2.3 Casting Implicito e Perda de Precisao

**MISRA C:2012 Rule 10.3** - Valor de expressao nao deve ser atribuido a tipo mais estreito

| Arquivo | Linha | Problema |
|---------|-------|----------|
| `BoucleCRS.cpp` | 125 | `m_dist_next_seg = (uint16_t)tmp_dist;` - truncamento |
| `Attitude.cpp` | 246 | `att.slope = (int8_t)(100.f * slope);` - overflow possivel |
| `Locator.cpp` | 313-315 | `gps_loc.data.date` - operacoes sem verificacao de overflow |
| `fec.c` | 129-136 | `uint8_t new_time` - rollover handling fragil |

**Correcao:**
```c
// RUIM
att.slope = (int8_t)(100.f * slope);  // Pode overflow!

// BOM
float slope_scaled = 100.f * slope;
if (slope_scaled > INT8_MAX) slope_scaled = INT8_MAX;
if (slope_scaled < INT8_MIN) slope_scaled = INT8_MIN;
att.slope = (int8_t)slope_scaled;
```

### 2.4 Falta de Verificacao de Retorno

**MISRA C:2012 Rule 17.7** - Valor retornado por funcao deve ser usado

| Arquivo | Linha | Funcao sem verificacao |
|---------|-------|------------------------|
| `main.cpp` | 470 | `nrf_pwr_mgmt_init();` |
| `GPSMGMT.cpp` | 89-96 | `SEND_TO_GPS()` - sem verificacao de erro |
| `Locator.cpp` | 344 | `(void)totalGPGSVMessages.value();` - cast para void ok, mas sem log |
| `Model.cpp` | 164 | `sd_functions__start_query()` - ret nao usado se != 0 |

### 2.5 Mutex/Sincronizacao Inadequada

**Problema CRITICO para RTOS**

| Arquivo | Linha | Problema |
|---------|-------|----------|
| `ls027.c` | 58-77 | Mutex implementado com busy-wait e decremento |
| `Locator.cpp` | 21 | `m_is_gps_updated` modificado de ISR sem volatile |
| `fec.c` | 44 | `is_fec_tx_pending` sem protecao atomica |

**Problema em `ls027.c`:**
```c
// CRITICO - Race condition e busy-wait
static void _ls027_mutex_take(void) {
    while (m_ls27_mutex_taken) {
        w_task_delay(25);
        m_ls27_mutex_taken -= 1;  // RACE CONDITION!
    }
    m_ls27_mutex_taken = 10;
}
```

**Correcao para Zephyr:**
```c
#include <zephyr/kernel.h>

static K_MUTEX_DEFINE(ls027_mutex);

static void _ls027_mutex_take(void) {
    k_mutex_lock(&ls027_mutex, K_FOREVER);
}

static void _ls027_mutex_give(void) {
    k_mutex_unlock(&ls027_mutex);
}
```

---

## 3. Violacoes de Alto Impacto

### 3.1 Naming Conventions Inconsistentes

**Padrao 0.memory:**
- `s_` para static
- `g_` para global
- `m_` para members
- `PascalCase` para tipos
- `module_action_object` para funcoes

| Arquivo | Problema | Deveria Ser |
|---------|----------|-------------|
| `GPSMGMT.cpp:46` | `m_epo_state` | `s_epo_state` (static file scope) |
| `GPSMGMT.cpp:48` | `m_is_uart_on` | `s_is_uart_on` |
| `ls027.c:48` | `m_M1_bit` | `s_m1_bit` |
| `Locator.cpp:23` | `gps` | `g_gps` (global) |
| `Model.cpp:36` | `att` | `g_att` |
| `ant.c:43` | `ant_evt_bs` | `ant_bs_evt_handler` |

### 3.2 Magic Numbers

**MISRA C:2012 Rule 7.2** - Constantes numericas devem ter sufixo apropriado

| Arquivo | Linha | Magic Number | Descricao |
|---------|-------|--------------|-----------|
| `Attitude.cpp` | 147-157 | `0.03f, 0.10f, 0.0002f` | Kalman Q matrix |
| `Attitude.cpp` | 156-157 | `1000.f, 600.f` | Kalman R matrix |
| `Attitude.cpp` | 212-213 | `40` | Time constant |
| `Attitude.cpp` | 324 | `800.f` | Filter tau |
| `Segment.cpp` | 290-291 | `0.001f` | Norm threshold |
| `power_scheduler.cpp` | 16 | `15` | Idle timeout minutes |
| `ls027.c` | 70 | `10, 25` | Mutex timeout values |

**Correcao:**
```c
// RUIM
if (m_speed_ms < 1.5f) { ... }

// BOM
#define KALMAN_MIN_SPEED_MS  1.5f
if (m_speed_ms < KALMAN_MIN_SPEED_MS) { ... }
```

### 3.3 Funcoes Muito Longas

**Best Practice:** Funcoes devem ter < 50 linhas

| Arquivo | Funcao | Linhas | Complexidade |
|---------|--------|--------|--------------|
| `Attitude.cpp` | `computeFusion()` | ~175 | Alta |
| `BoucleCRS.cpp` | `run_internal()` | ~130 | Alta |
| `Segment.cpp` | `majPerformance()` | ~100 | Alta |
| `GPSMGMT.cpp` | `tasks()` | ~115 | Alta |

### 3.4 Uso de `float` para Comparacoes

**MISRA C:2012 Rule 13.3** - Operacoes de floating-point nao devem ser usadas para igualdade

| Arquivo | Linha | Problema |
|---------|-------|----------|
| `Attitude.cpp` | 353 | `if (m_climb < 0)` |
| `Segment.cpp` | 290 | `if (PC.getNorm() > 0.001f)` |
| `BoucleCRS.cpp` | 124 | `if (tmp_dist > 0.f)` |

**Correcao:**
```c
// RUIM
if (value == 0.0f) { ... }

// BOM
#define FLOAT_EPSILON 1e-6f
if (fabsf(value) < FLOAT_EPSILON) { ... }
```

---

## 4. Violacoes de Medio Impacto

### 4.1 Falta de `const` Correctness

**MISRA C++:2008 Rule 7-1-1** - Variaveis que nao sao modificadas devem ser declaradas const

| Arquivo | Linha | Problema |
|---------|-------|----------|
| `Vue.cpp` | 225 | `void cadranH(uint8_t p_lig, ...)` - parametros devem ser const |
| `Segment.cpp` | 253 | `int testActivation(ListePoints& liste)` - deve ser const& |
| `Attitude.cpp` | 431 | `computeDistance(SLoc& loc_, SDate &date_)` - devem ser const& |

### 4.2 Alocacao Dinamica em Runtime

**Padrao 0.memory:** Zero dynamic allocation em runtime

| Arquivo | Linha | Problema |
|---------|-------|----------|
| `Locator.cpp` | 39 | `static std::vector<sSatellite> sats;` |
| `Locator.cpp` | 360 | `sats.push_back(sat_);` |
| `Segment.cpp` | 90 | `m_p_data = new sSegmentData();` |
| `Vue.cpp` | 116 | `String text;` - dynamic allocation |

**Correcao:**
```cpp
// RUIM
std::vector<sSatellite> sats;
sats.push_back(sat_);

// BOM - Pre-alocacao
static std::array<sSatellite, MAX_SATELLITES> s_sats;
static size_t s_sats_count = 0;

if (s_sats_count < MAX_SATELLITES) {
    s_sats[s_sats_count++] = sat_;
}
```

### 4.3 Uso de `String` Arduino

**Problema:** Fragmentacao de heap, overhead de alocacao

| Arquivo | Contagem | Impacto |
|---------|----------|---------|
| `main.cpp` | 5 usos | Alto |
| `Boucle.cpp` | 4 usos | Alto |
| `Vue.cpp` | 15+ usos | Muito Alto |
| `Locator.cpp` | 6 usos | Alto |
| `GPSMGMT.cpp` | 1 uso | Baixo |

**Correcao:**
```cpp
// RUIM
String message = "Hardfault happened: pc = 0x";
message += String(m_app_error.hf_desc.stck.pc, HEX);

// BOM - Buffer estatico
char message[128];
snprintf(message, sizeof(message),
         "Hardfault happened: pc = 0x%08lX",
         m_app_error.hf_desc.stck.pc);
```

### 4.4 Includes Duplicados

| Arquivo | Linha | Include Duplicado |
|---------|-------|-------------------|
| `main.cpp` | 14, 21 | `#include "i2c.h"` |

### 4.5 Comentarios em Codigo Morto

| Arquivo | Linhas | Problema |
|---------|--------|----------|
| `Vue.cpp` | 147-162 | Codigo comentado (switch rotation) |
| `ant.c` | 372-386 | Codigo comentado (channel status check) |
| `GPSMGMT.cpp` | 202, 211 | `//SEND_TO_GPS(PMTK_STANDBY)` |

---

## 5. Violacoes de Baixo Impacto

### 5.1 Falta de Header Guards Modernos

Usar `#pragma once` ou guards consistentes:
```c
// PADRAO
#ifndef MODULE_NAME_H
#define MODULE_NAME_H
...
#endif // MODULE_NAME_H
```

### 5.2 Uso de `int` sem Largura Fixa

**MISRA C:2012 Rule 10.1** - Operandos devem ter tipo essencial apropriado

| Arquivo | Linha | Problema |
|---------|-------|----------|
| `Vue.cpp` | 196 | `for (int i=1; ...)` |
| `ls027.c` | 132, 274 | `for (int i=0; ...)` |
| `Locator.cpp` | 97 | `for (int i = 0; ...)` |

**Correcao:** Usar `int16_t`, `uint16_t`, `size_t` conforme apropriado.

### 5.3 Falta de `default` em `switch`

**MISRA C:2012 Rule 16.4** - Todo switch deve ter clausula default

| Arquivo | Funcao | Linha |
|---------|--------|-------|
| `Boucle.cpp` | `boucle__change_mode` | 111-137 |
| `fec.c` | `roller_manager` | 191-220 |

---

## 6. Problemas Arquiteturais para Migracao Zephyr

### 6.1 Task Management Atual vs Zephyr

**Atual:**
```c
// Nordic SDK task wrapper
m_tasks_id.boucle_id = task_create(boucle_task, "boucle_tasks", NULL);
task_manager_start(idle_task, &m_tasks_id);
```

**Zephyr:**
```c
K_THREAD_DEFINE(boucle_thread, STACK_SIZE,
                boucle_task, NULL, NULL, NULL,
                PRIORITY, 0, 0);
```

### 6.2 Timer API Incompativel

**Atual:** Nordic SDK `app_timer`
**Zephyr:** `k_timer` ou `k_work_delayable`

### 6.3 Event Flags

**Atual:** `w_task_events_set()`, `w_task_events_wait()`
**Zephyr:** `k_event` ou `k_poll`

### 6.4 GPIO API

**Atual:** `nrf_gpio_*`
**Zephyr:** Device Tree + GPIO API

### 6.5 SPI/I2C

**Atual:** Nordic SPIM, custom wrappers
**Zephyr:** SPI/I2C API unificada

---

## 7. Recomendacoes para Migracao Zephyr

### 7.1 Fase 1: Cleanup (2-3 semanas de esforco)

1. **Remover codigo morto** - Comentarios antigos, funcoes nao usadas
2. **Inicializar todas variaveis** - Zero-init explicito
3. **Substituir String por char[]** - Eliminar alocacao dinamica
4. **Aplicar naming conventions** - Prefixos s_, g_, m_
5. **Definir constantes** - Eliminar magic numbers

### 7.2 Fase 2: Refatoracao (3-4 semanas)

1. **Extrair HAL** - Criar camada de abstracao para GPIO, SPI, I2C, UART
2. **Separar estado** - Remover static locals, criar contextos explicitos
3. **Implementar mutex Zephyr** - Substituir busy-wait por k_mutex
4. **Criar event system** - k_event ou k_msgq para comunicacao inter-task

### 7.3 Fase 3: Portabilidade (4-6 semanas)

1. **Device Tree** - Definir hardware no DTS
2. **Kconfig** - Configuracao de features
3. **Substituir SDK calls** - Nordic SDK -> Zephyr API
4. **Unit tests** - Ztest framework

### 7.4 Estrutura de Diretorios Sugerida para Zephyr

```
stravaV10_zephyr/
├── CMakeLists.txt
├── prj.conf
├── Kconfig
├── boards/
│   └── nrf52840_custom.overlay
├── src/
│   ├── main.c
│   ├── model/
│   │   ├── boucle.c
│   │   ├── attitude.c
│   │   └── locator.c
│   ├── drivers/
│   │   ├── lcd/
│   │   ├── sensors/
│   │   └── gps/
│   ├── rf/
│   │   ├── ant/
│   │   └── ble/
│   └── vue/
├── include/
│   └── (headers organizados por modulo)
└── tests/
    └── (Ztest suites)
```

---

## 8. Checklist de Conformidade

### 8.1 MISRA C:2012 Prioritario

- [ ] Rule 9.1: Inicializar todas variaveis
- [ ] Rule 10.1: Usar tipos com largura fixa
- [ ] Rule 10.3: Evitar conversoes implicitas com perda
- [ ] Rule 13.3: Nao comparar floats com ==
- [ ] Rule 16.4: Default em todos switches
- [ ] Rule 17.7: Verificar retornos de funcao

### 8.2 Padroes 0.memory

- [ ] Naming conventions (s_, g_, m_)
- [ ] Zero dynamic allocation
- [ ] Volatile para dados compartilhados com ISR
- [ ] Const correctness
- [ ] Error handling explicito
- [ ] Funcoes < 50 linhas

### 8.3 Zephyr Readiness

- [ ] HAL abstraido
- [ ] Estado em contextos explicitos
- [ ] Mutex/semaforos adequados
- [ ] Event system via k_event ou k_msgq
- [ ] Device Tree ready

---

## 9. Metricas de Qualidade Atual

| Metrica | Valor | Target |
|---------|-------|--------|
| Violacoes CRITICAS | 12 | 0 |
| Violacoes ALTAS | 28 | 0 |
| Violacoes MEDIAS | 45+ | < 10 |
| Violacoes BAIXAS | 30+ | < 20 |
| Cobertura de testes | ~0% | > 70% |
| Complexidade ciclomatica max | 35+ | < 10 |
| Linhas por funcao (max) | 175 | < 50 |

---

## 10. Referencias

- MISRA C:2012 Guidelines
- MISRA C++:2008 Guidelines
- Zephyr RTOS Documentation
- NASA JPL Coding Standard
- CERT C Secure Coding Standard
- 0.memory - Padroes de Firmware

