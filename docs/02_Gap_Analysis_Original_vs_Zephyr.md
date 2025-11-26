# Gap Analysis: Original nRF5 SDK vs New Zephyr Implementation

## Resumo Executivo

Este documento apresenta uma análise **extremamente detalhada** comparando a implementação original (nRF5 SDK, C++) com a nova implementação (Zephyr RTOS, C). O objetivo é garantir que **nenhuma lógica crítica** seja perdida na migração.

---

## 1. Filtro Kalman - CRÍTICO

### Original (`libraries/kalman/kalman_ext.cpp` + `Attitude.cpp`)

O original usa um **filtro Kalman de 3 estados** para fusão de altitude:

```cpp
// Estado: [elevation, pitch, alpha_zero]
m_k_lin.ker.ker_dim = 3;
m_k_lin.ker.obs_dim = 2;

// Matriz de transição A (3x3):
//     | 1  dl -dl |
// A = | 0   1   0 |
//     | 0   0   1 |
m_k_lin.ker.matA.set(0, 1,  feed.dt);  // dl = speed * dt
m_k_lin.ker.matA.set(0, 2, -feed.dt);

// Ruído de processo Q:
m_k_lin.ker.matQ.set(0, 0, 0.03f);   // elevation
m_k_lin.ker.matQ.set(1, 1, 0.10f);   // pitch
m_k_lin.ker.matQ.set(2, 2, 0.0002f); // alpha_zero

// Ruído de observação R:
m_k_lin.ker.matR.set(0, 0, 1000.f);  // barometer altitude
m_k_lin.ker.matR.set(1, 1, 600.f);   // pitch from accelerometer

// Observações Z:
feed.matZ.set(0, 0, ele);        // barometer elevation
feed.matZ.set(1, 0, pitch_rad);  // pitch from FXOS
```

**Lógica do modelo:**
- `d(elevation)/dt = speed * tan(slope_rad)`
- `slope_rad = pitch_rad - alpha_zero`
- `alpha_zero` compensa a montagem do sensor

### Novo (`zephyr_app/src/model/kalman.c`)

O novo usa um **filtro Kalman simples de 1 estado**:

```c
// Estado único
state->x = state->x + state->k * (measurement - state->x);
```

### GAP IDENTIFICADO
| Aspecto | Original | Novo | Status |
|---------|----------|------|--------|
| Dimensão do estado | 3 (elev, pitch, alpha0) | 1 | **FALTA** |
| Biblioteca matricial | UDMatrix completa | Não existe | **FALTA** |
| Fusão baro/IMU | Sim, com pitch | Não | **FALTA** |
| Estimativa de slope | Via Kalman | Via diferenças simples | **SIMPLIFICADO** |

### AÇÃO REQUERIDA
Implementar `UDMatrix` em C e o filtro de 3 estados em `kalman.c`.

---

## 2. Sistema de Segmentos - CRÍTICO

### Original (`source/routes/Segment.cpp` + `Vecteur.cpp` + `ListePoints.cpp`)

#### 2.1 Ativação de Segmento (`testActivation`)

```cpp
int Segment::testActivation(ListePoints& liste) {
    // Vetores de direção
    PC = Vecteur(PPp, PPc);  // movimento atual
    PS = Vecteur(P1, P2);    // direção do segmento

    // Normalizar e calcular produto escalar
    PC.norm();
    PS.norm();
    p_scal = ScalarProduct(PC, PS);

    // Condições de ativação:
    // 1. Produto escalar > limite (mesma direção)
    // 2. Geometria correta (pythagoras)
    if (p_scal > PSCAL_LIM &&
        distP2 * distP2 < distP1 * distP1 + distP1P2 * distP1P2) {
        return 1;  // ATIVAR
    }
}
```

#### 2.2 Desativação de Segmento (`testDesactivation`)

```cpp
int Segment::testDesactivation(ListePoints& liste) {
    // Verifica se passou o último ponto
    if (distP1 * distP1 > distP2 * distP2 + distP1P2 * distP1P2 &&
        (distP1 < DIST_ACT || distP2 < DIST_ACT)) {
        return 1;  // DESATIVAR (terminou)
    }
}
```

#### 2.3 Performance (`majPerformance`)

```cpp
void Segment::majPerformance(ListePoints& mes_points) {
    // Atualiza posição relativa no segmento
    m_p_data->_lpts.updateRelativePosition(pc);
    vect = m_p_data->_lpts.getPosRelative();

    // Verifica se está dentro da margem
    if (fabsf(vect._y) < MARGE_ACT * DIST_ACT) {
        // Calcula avanço temporal
        m_p_data->_monAvance = (vect._t - pp->_rtime) - m_p_data->_monCur;

        // Progresso percentual
        m_p_data->_monPDist = (float)m_p_data->_lpts.ind_P1 / (float)size();

        // Progresso de elevação
        if (m_p_data->_elevTot > 5.f) {
            m_p_data->_monPElev = (vect._z - pc._alt) / m_p_data->_elevTot;
        }
    }
}
```

### Novo (`zephyr_app/src/model/segment.c`)

```c
// PLACEHOLDER apenas!
static float dist_to_start(uint16_t seg_idx, const loc_data_t *loc) {
    (void)seg_idx;
    (void)loc;
    return 9999.0f;  // Não implementado
}

static float find_progress(uint16_t seg_idx, const loc_data_t *loc, float *advance_out) {
    (void)seg_idx;
    (void)loc;
    (void)advance_out;
    return 0.0f;  // Não implementado
}
```

### GAP IDENTIFICADO
| Funcionalidade | Original | Novo | Status |
|----------------|----------|------|--------|
| Classe Vecteur | Completa | Não existe | **FALTA** |
| ScalarProduct | Implementado | Não existe | **FALTA** |
| testActivation | Completo | Placeholder | **FALTA** |
| testDesactivation | Completo | Placeholder | **FALTA** |
| majPerformance | Completo | Placeholder | **FALTA** |
| Posição relativa | updateRelativePosition | Não existe | **FALTA** |
| Lista de pontos | ListePoints (std::list) | Não existe | **FALTA** |

### AÇÃO REQUERIDA
1. Criar `vecteur.c/h` com operações vetoriais
2. Criar `liste_points.c/h` para gerenciamento de pontos
3. Implementar lógica completa de segmentos

---

## 3. Sistema de Posição Relativa - CRÍTICO

### Original (`source/routes/ListePoints.cpp`)

```cpp
void ListePoints::updateRelativePosition(Point& point) {
    // Encontra os dois pontos mais próximos P1 e P2
    for (auto& tmpPT : m_lpoints) {
        tmp_dist = tmpPT.dist(&point);
        // ... lógica de ordenação
    }

    // Calcula projeção ortogonal
    if (distP1_ > 50. ||
        (distP2_*distP2_ >= p1p2_dist*p1p2_dist + distP1_*distP1_)) {
        // Fora do triângulo
        m_pos_r._x = 0.;
        m_pos_r._y = 0.;
    } else {
        // Dentro do triângulo
        Vecteur P1P = Vecteur(P1, point);
        Vecteur P1P2 = Vecteur(P1, P2);

        // Projeção em P1P2
        m_pos_r._x = ScalarProduct(P1P, P1P2) / P1P2.getNorm();

        // Projeção no vetor ortogonal
        Vecteur orthoP1P2;
        orthoP1P2._x = P1P2._y;
        orthoP1P2._y = -P1P2._x;
        m_pos_r._y = ScalarProduct(P1P, orthoP1P2) / orthoP1P2.getNorm();

        // Interpolação de altitude e tempo
        m_pos_r._z = P1._alt + (P2._alt - P1._alt) * m_pos_r._x / P1P2.getNorm();
        m_pos_r._t = P1._rtime + (P2._rtime - P1._rtime) * m_pos_r._x / P1P2.getNorm();
    }
}
```

### Novo
**NÃO EXISTE** - O sistema de posição relativa não foi implementado.

### GAP IDENTIFICADO
Esta funcionalidade é **essencial** para:
- Calcular avanço/atraso em segmentos
- Determinar posição exata no percurso
- Interpolar altitude e tempo

---

## 4. Fusão Baro/GPS com Correção de Drift - IMPORTANTE

### Original (`source/model/Attitude.cpp`)

```cpp
float Attitude::filterElevation(SLoc& loc_, eLocationSource source_) {
    // High-pass filter para remover drift do barômetro
    if (source_ == eLocationSourceGPS || source_ == eLocationSourceNRF) {
        float ele = 0.;
        m_baro.computeAlti(ele);

        // Filtro de alta constante de tempo
        float input = ele - loc_.alt;  // diferença baro-GPS
        static float alt_div = input;

        const float taub = 800.f / (800.f + 1.f);  // ~800 segundos
        alt_div = taub * alt_div + (1 - taub) * (input);

        // Aplicar correção ao barômetro
        m_baro.setCorrection(alt_div);
    }

    // Histerese para cálculo de subida
    if (m_cur_ele > m_last_stored_ele + CLIMB_ELEVATION_HYSTERESIS_M) {
        m_climb += m_cur_ele - m_last_stored_ele;
        m_last_stored_ele = m_cur_ele;
    }
    // ...
}
```

### Novo (`zephyr_app/src/model/locator.c`)

```c
// Lógica simplificada sem correção de drift
float elev_diff = filtered.alt - prev->loc.alt;
if (elev_diff > 0.5f) {  // Histerese de 0.5m apenas
    total_climb += elev_diff;
}
```

### GAP IDENTIFICADO
| Aspecto | Original | Novo | Status |
|---------|----------|------|--------|
| Correção de drift baro/GPS | Sim, tau=800s | Não | **FALTA** |
| Histerese de elevação | 2.0m | 0.5m | **DIFERENTE** |
| Filtro passa-alta | Implementado | Não | **FALTA** |
| setCorrection no baro | Sim | Não | **FALTA** |

---

## 5. ANT+ Protocol Stack - MUITO IMPORTANTE

### Original (`rf/ant_device_manager.cpp` + `ant.h`)

```cpp
// Perfis ANT+ suportados:
typedef enum {
    eAntPairingSensorTypeNone,
    eAntPairingSensorTypeHRM,  // Heart Rate Monitor
    eAntPairingSensorTypeBSC,  // Bike Speed/Cadence
    eAntPairingSensorTypeFEC   // Fitness Equipment Control (Tacx)
} eAntPairingSensorType;

// Estruturas de dados:
sHrmInfo hrm_info;  // BPM, RR intervals
sBscInfo bsc_info;  // Speed, Cadence, wheel revolutions
sFecInfo fec_info;  // FEC trainer data
```

### Novo
**NÃO EXISTE** - ANT+ não foi portado.

### GAP IDENTIFICADO
| Funcionalidade | Original | Novo | Status |
|----------------|----------|------|--------|
| ANT+ stack | SoftDevice S340 | Não existe | **FALTA** |
| HRM profile | Completo | Não existe | **FALTA** |
| BSC profile | Completo | Não existe | **FALTA** |
| FEC profile | Completo | Não existe | **FALTA** |
| Device pairing | ant_device_manager | Não existe | **FALTA** |

### NOTA
O Zephyr não tem ANT+ nativo. Opções:
1. Usar SoftDevice S340 via driver customizado
2. Substituir por BLE (ANT+ BLE bridge)
3. Implementar ANT+ sobre RAW radio

---

## 6. PowerZone, SufferScore, RRZone - IMPORTANTE

### Original

#### 6.1 PowerZone (`source/model/PowerZone.cpp`)
```cpp
static float pw_lims[PW_ZONES_NB+1] = {
    -100.0f, 0.55f, 0.75f, 0.90f, 1.05f, 1.20f, 1.50f, 100.0f
};

void PowerZone::addPowerData(uint16_t pw_meas, uint32_t timestamp) {
    const uint16_t ftp = u_settings.getFTP();
    for (int i=0; i < PW_ZONES_NB; i++) {
        if (pw_meas >= pw_lims[i] * ftp && pw_meas < pw_lims[i+1] * ftp) {
            m_pw_bins[i] += time_integ;
        }
    }
}
```

#### 6.2 SufferScore (`source/model/SufferScore.cpp`)
```cpp
#define HRM_Z1_PTS_PER_HOUR (16./3600.)
#define HRM_Z2_PTS_PER_HOUR (33./3600.)
#define HRM_Z3_PTS_PER_HOUR (72./3600.)
#define HRM_Z4_PTS_PER_HOUR (85./3600.)
#define HRM_Z5_PTS_PER_HOUR (95./3600.)

void SufferScore::updateScore(void) {
    m_score  = m_hrm_bins[0] * HRM_Z1_PTS_PER_HOUR;
    m_score += m_hrm_bins[1] * HRM_Z2_PTS_PER_HOUR;
    // ...
}
```

#### 6.3 RRZone (`source/model/RRZone.cpp`)
```cpp
// RMSSD calculation for HRV
float var_meas = 0.0F;
float sum_sq = 0.0F;
for (int i = 1; i < VAR_NB_ELEM; i++) {
    float val = tab_meas[i] - tab_meas[i-1];
    sum_sq += val * val;
}
var_meas = my_sqrtf(sum_sq / VAR_NB_ELEM);  // RMSSD
```

### Novo
**NENHUM IMPLEMENTADO**

### GAP IDENTIFICADO
| Funcionalidade | Status |
|----------------|--------|
| Power zones (7 zonas) | **FALTA** |
| Suffer Score | **FALTA** |
| RR Zone (RMSSD HRV) | **FALTA** |
| Binned data base class | **FALTA** |

---

## 7. User Settings + FRAM Storage - IMPORTANTE

### Original (`source/model/UserSettings.cpp`)

```cpp
#define FRAM_SETTINGS_ADDRESS  0x0000
#define FRAM_SETTINGS_VERSION  3

typedef struct {
    uint8_t version;
    uint16_t FTP;
    uint16_t weight;
    uint16_t hrm_devid;
    uint16_t bsc_devid;
    uint16_t fec_devid;
    uint16_t gla_devid;
    uint8_t crc;
} sUserParameters;

bool UserSettings::writeConfig(void) {
    m_params.crc = _calculate_crc(&m_params.flat_user_params,
                                   sizeof(sUserParameters) - sizeof(m_params.crc));
    return fram_write_block(FRAM_SETTINGS_ADDRESS, &m_params.flat_user_params,
                            sizeof(sUserParameters));
}
```

### Novo
**NÃO EXISTE** - Settings são constantes hardcoded.

---

## 8. GPS EPO e Host Aiding - IMPORTANTE

### Original (`source/sensors/GPSMGMT.cpp`)

```cpp
// EPO Update State Machine
typedef enum {
    eGPSMgmtEPOIdle,
    eGPSMgmtEPOStart,
    eGPSMgmtEPORunning,
    eGPSMgmtEPOWaitForEvent,
    eGPSMgmtEPOEnd
} eGPSMgmtEPOState;

// Host Aiding com posição LNS via BLE
void GPS_MGMT::startHostAidingEPO(sLocationData& loc_data, uint32_t age_) {
    snprintf((char*)buffer, sizeof(buffer),
             "$PMTK741,%.6f,%.6f,%d,%u,%u,%u,%02u,%02u,%02u",
             loc_data.lat, loc_data.lon, (int)loc_data.alt,
             _year, _month, _day, hours, minutes, seconds);
    GPS_UART_SEND(buffer, res);
}
```

### Novo (`zephyr_app/src/drivers/gps_mgmt.c`)
**NÃO EXISTE** - Apenas básico GPS on/off.

---

## 9. Crash Recovery (FDIR) - IMPORTANTE

### Original (`source/model/Attitude.cpp`)

```cpp
// Salvar estado para recuperação
memcpy(&m_app_error.saved_data.att, &att, sizeof(SAtt));
m_app_error.saved_data.crc = calculate_crc((uint8_t*)&m_app_error.saved_data.att,
                                            sizeof(m_app_error.saved_data.att));

// Recuperar após reboot
if (m_app_error.saved_data.crc == calculate_crc(...)) {
    if (att.date.date == m_app_error.saved_data.att.date.date) {
        att.climb = m_app_error.saved_data.att.climb;
        att.dist = m_app_error.saved_data.att.dist;
        vue.addNotif("FDIR", "Attitude restored", 6, eNotificationTypeComplete);
    }
}
```

### Novo
**NÃO EXISTE** - Sem recuperação de estado.

---

## 10. SD Card Position Logging - IMPORTANTE

### Original (`source/model/Attitude.cpp`)

```cpp
#define ATT_BUFFER_NB_ELEM  5

// Buffer de posições
sAttSnapshot m_st_buffer[ATT_BUFFER_NB_ELEM];

void Attitude::computeDistance(SLoc& loc_, SDate &date_) {
    if (att.dist > m_last_save_dist + 15.f) {
        // Salvar snapshot completo
        memcpy(&m_st_buffer[m_st_buffer_nb_elem].loc, &loc_, sizeof(SLoc));
        m_st_buffer[m_st_buffer_nb_elem].sensors.pwr = att.pwr;
        m_st_buffer[m_st_buffer_nb_elem].sensors.bpm = hrm_info.bpm;
        m_st_buffer[m_st_buffer_nb_elem].sensors.cadence = bsc_info.cadence;
        m_st_buffer[m_st_buffer_nb_elem].alti.filt_ele = m_cur_ele;
        // ...

        if (m_st_buffer_nb_elem >= ATT_BUFFER_NB_ELEM) {
            sd_save_pos_buffer(m_st_buffer, ATT_BUFFER_NB_ELEM);
            m_st_buffer_nb_elem = 0;
        }
    }
}
```

### Novo
Não há sistema de logging estruturado no SD card.

---

## 11. Multiple Location Sources - IMPORTANTE

### Original (`source/model/Locator.cpp`)

```cpp
typedef enum {
    eLocationSourceNone = 0,
    eLocationSourceGPS,   // GPS interno
    eLocationSourceNRF,   // BLE LNS (Location and Navigation Service)
    eLocationSourceSIM    // Simulador (via BLE NUS)
} eLocationSource;

eLocationSource Locator::getUpdateSource() {
    // Prioridade: SIM > GPS > NRF(LNS)
    if (sim_loc.isUpdated()) return eLocationSourceSIM;
    if (gps_loc.isUpdated()) return eLocationSourceGPS;
    if (nrf_loc.isUpdated() && !gps_mgmt.isFix()) return eLocationSourceNRF;
    return eLocationSourceNone;
}
```

### Novo
Apenas uma fonte de posição (GPS).

---

## 12. TinyGPS++ vs NMEA Parser

### Original (`libraries/TinyGPSPlus/TinyGPS++.cpp`)

```cpp
// Custom fields para GPGSA, GPGSV
TinyGPSCustom hdop(gps, "GPGSA", 16);
TinyGPSCustom vdop(gps, "GPGSA", 17);
TinyGPSCustom satsInView(gps, "GPGSV", 3);
TinyGPSCustom satNumber[NB_SATS_TO_DETAIL];
TinyGPSCustom elevation[NB_SATS_TO_DETAIL];
TinyGPSCustom snr[NB_SATS_TO_DETAIL];
```

### Novo (`zephyr_app/src/drivers/nmea_parser.c`)
Parser básico para GGA, RMC. Faltam:
- GSA (DOP values)
- GSV (satellite info)
- Custom sentence parsing

---

## 13. Sistema de UI (Vue) - MODERADO

### Original (`source/vue/`)

```
Vue.cpp           - Base class
VueCRS.cpp        - Cycling screen
VueFEC.cpp        - Trainer screen
VueGPS.cpp        - GPS debug screen
VuePRC.cpp        - Route screen
VueDebug.cpp      - Debug screen
Menuable.cpp      - Menu system base
MenuObjects.cpp   - Menu items
Screenutils.cpp   - Screen utilities
```

### Novo (`zephyr_app/src/vue/`)

```
vue.c             - Tudo em um arquivo
vue_crs.c         - Placeholder
vue_fec.c         - Placeholder
```

### GAP
Sistema de menus e múltiplas telas não implementado.

---

## 14. Constantes e Parâmetros Críticos

### Original (`source/parameters.h`)

```cpp
#define DIST_ACT            50.f    // Distância de ativação de segmento
#define MARGE_ACT           1.5f    // Margem lateral
#define PSCAL_LIM           0.85f   // Limite de produto escalar
#define DIST_ALLOC          250.f   // Distância de alocação
#define HISTO_POINT_SIZE    40      // Tamanho do histórico
#define NB_RECORDING        40      // Pontos para gravação
#define ATT_BUFFER_NB_ELEM  5       // Buffer de atitude
#define CLIMB_ELEVATION_HYSTERESIS_M  2.f  // Histerese de subida
```

### Novo
Algumas constantes diferentes ou faltando.

---

## 15. Fórmula de Potência

### Original (`source/model/Attitude.cpp:556-575`)

```cpp
float Attitude::computePower(float speed_) {
    const float weight = (float)u_settings.getWeight();

    power += 9.81f * weight * att.vit_asc;              // Gravidade
    power += 0.004f * 9.81f * weight * m_speed_ms;      // Rolamento
    power += 0.204f * m_speed_ms * m_speed_ms * m_speed_ms; // Arrasto
    power *= 1.025f;  // Transmissão (rendimento ~97.5%)

    return power;
}
```

### Novo (`zephyr_app/src/model/attitude.c:65-92`)

```c
static uint16_t estimate_power(float speed_kmh, int8_t slope) {
    float p_roll = 0.005f * total_mass * 9.81f * speed_ms;  // Coef diferente
    float p_air = 0.5f * 1.225f * 0.3f * 0.5f * speed_ms^3; // Fórmula completa
    float p_gravity = total_mass * 9.81f * grade * speed_ms;
    // Sem fator de transmissão
}
```

---

## Resumo de Prioridades

### CRÍTICO (Bloqueia funcionalidade core)
1. Filtro Kalman 3 estados + UDMatrix
2. Sistema de Segmentos completo (Vecteur, testActivation, etc.)
3. ListePoints com posição relativa

### MUITO IMPORTANTE (Funcionalidade significativa)
4. ANT+ ou alternativa BLE
5. PowerZone, SufferScore
6. Fusão Baro/GPS com correção de drift

### IMPORTANTE (Melhorias necessárias)
7. User Settings + FRAM
8. GPS EPO/Host Aiding
9. Crash Recovery
10. SD Logging estruturado
11. Multiple Location Sources

### MODERADO (Nice to have)
12. Sistema de menus completo
13. Múltiplas telas (FEC, PRC, Debug)
14. NMEA parser estendido

---

## Próximos Passos Recomendados

1. **Implementar UDMatrix** em C puro (ou usar CMSIS-DSP)
2. **Portar Vecteur** e operações vetoriais
3. **Implementar ListePoints** com lista encadeada
4. **Completar segment.c** com lógica de ativação
5. **Implementar filtro Kalman 3 estados**
6. **Decidir estratégia ANT+** (SoftDevice híbrido ou BLE bridge)
7. **Adicionar FRAM driver** e user settings
8. **Implementar crash recovery** usando Zephyr retained memory

---

*Documento gerado em: 2025-11-26*
*Versão: 1.0*
