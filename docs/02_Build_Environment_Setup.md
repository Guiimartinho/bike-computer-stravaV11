# stravaV10 - Configuracao do Ambiente de Build

**Versao:** 1.0
**Data:** 2025-11-25

---

## 1. Visao Geral

Este documento descreve duas opcoes de ambiente:

| Opcao | SDK | RTOS | Uso |
|-------|-----|------|-----|
| **A** | nRF5 SDK 17.1.0 | Nenhum | Buildar codigo atual |
| **B** | nRF Connect SDK 3.x | Zephyr | Migracao futura |

**Recomendacao:** Configurar ambos - usar Opcao A para manter o projeto funcionando enquanto migra para Opcao B.

---

## 2. Opcao A: nRF5 SDK (Codigo Atual)

### 2.1 Requisitos

| Componente | Versao | Link |
|------------|--------|------|
| VS Code | Latest | https://code.visualstudio.com/download |
| GCC ARM | 10.3-2021.10 | https://developer.arm.com/downloads/-/gnu-rm |
| nRF5 SDK | 17.1.0 | https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download |
| nRF Command Line Tools | Latest | https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download |
| Make | GNU Make 4.x | Incluido no Git for Windows |

### 2.2 Instalacao GCC ARM

1. Baixar de: https://developer.arm.com/downloads/-/gnu-rm
   - Arquivo: `gcc-arm-none-eabi-10.3-2021.10-win32.exe`

2. Instalar em: `C:\Tools\gcc-arm-none-eabi-10.3-2021.10`

3. Adicionar ao PATH do sistema:
   ```
   C:\Tools\gcc-arm-none-eabi-10.3-2021.10\bin
   ```

4. Verificar instalacao:
   ```bash
   arm-none-eabi-gcc --version
   # Esperado: arm-none-eabi-gcc (GNU Arm Embedded Toolchain 10.3-2021.10) 10.3.1
   ```

### 2.3 Instalacao nRF5 SDK

1. Baixar de: https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download
   - Versao: 17.1.0

2. Extrair para: `C:\Nordic\nRF5_SDK_17.1.0`

3. Configurar toolchain - Editar `C:\Nordic\nRF5_SDK_17.1.0\components\toolchain\gcc\Makefile.windows`:
   ```makefile
   GNU_INSTALL_ROOT := C:/Tools/gcc-arm-none-eabi-10.3-2021.10/bin/
   GNU_VERSION := 10.3.1
   GNU_PREFIX := arm-none-eabi
   ```

### 2.4 Instalacao nRF Command Line Tools

1. Baixar de: https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download

2. Instalar (inclui nrfjprog e mergehex)

3. Verificar:
   ```bash
   nrfjprog --version
   ```

### 2.5 Configurar Projeto stravaV10

Editar `pca10056/s340/armgcc/Makefile.local`:

```makefile
# Windows
SDK_ROOT := C:/Nordic/nRF5_SDK_17.1.0
OUTPUT_DIRECTORY := _build_W
COM_PORT := COM8
```

### 2.6 VS Code Extensions (para nRF5 SDK)

Instalar:
- C/C++ (Microsoft)
- Cortex-Debug
- ARM (dan-c-underwood)

### 2.7 VS Code Tasks

Criar `.vscode/tasks.json`:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "Build",
            "type": "shell",
            "command": "make",
            "options": {
                "cwd": "${workspaceFolder}/pca10056/s340/armgcc"
            },
            "group": {
                "kind": "build",
                "isDefault": true
            },
            "problemMatcher": ["$gcc"]
        },
        {
            "label": "Clean",
            "type": "shell",
            "command": "make clean",
            "options": {
                "cwd": "${workspaceFolder}/pca10056/s340/armgcc"
            }
        },
        {
            "label": "Flash",
            "type": "shell",
            "command": "make flash",
            "options": {
                "cwd": "${workspaceFolder}/pca10056/s340/armgcc"
            },
            "dependsOn": "Build"
        },
        {
            "label": "Flash SoftDevice",
            "type": "shell",
            "command": "make flash_softdevice",
            "options": {
                "cwd": "${workspaceFolder}/pca10056/s340/armgcc"
            }
        },
        {
            "label": "Erase All",
            "type": "shell",
            "command": "make erase",
            "options": {
                "cwd": "${workspaceFolder}/pca10056/s340/armgcc"
            }
        }
    ]
}
```

### 2.8 VS Code Debug (J-Link)

Criar `.vscode/launch.json`:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Cortex Debug (J-Link)",
            "type": "cortex-debug",
            "request": "launch",
            "servertype": "jlink",
            "cwd": "${workspaceFolder}",
            "executable": "${workspaceFolder}/pca10056/s340/armgcc/_build_W/nrf52840_xxaa.out",
            "device": "nRF52840_xxAA",
            "interface": "swd",
            "runToEntryPoint": "main",
            "svdFile": "${workspaceFolder}/../nRF5_SDK_17.1.0/modules/nrfx/mdk/nrf52840.svd",
            "rtos": "FreeRTOS"
        }
    ]
}
```

### 2.9 VS Code IntelliSense

Criar `.vscode/c_cpp_properties.json`:

```json
{
    "configurations": [
        {
            "name": "nRF52840",
            "includePath": [
                "${workspaceFolder}/**",
                "C:/Nordic/nRF5_SDK_17.1.0/**"
            ],
            "defines": [
                "NRF52840_XXAA",
                "BOARD_CUSTOM",
                "SOFTDEVICE_PRESENT",
                "S340",
                "BLE_STACK_SUPPORT_REQD",
                "ANT_STACK_SUPPORT_REQD",
                "NRF_SD_BLE_API_VERSION=6",
                "APP_TIMER_V2",
                "APP_TIMER_V2_RTC1_ENABLED",
                "USB_ENABLED",
                "DEBUG",
                "DEBUG_NRF",
                "DEBUG_NRF_USER",
                "__HEAP_SIZE=150000",
                "FLOAT_ABI_HARD",
                "ARM_MATH_CM4"
            ],
            "compilerPath": "C:/Tools/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gcc.exe",
            "cStandard": "c11",
            "cppStandard": "c++14",
            "intelliSenseMode": "gcc-arm"
        }
    ],
    "version": 4
}
```

### 2.10 Build e Flash

```bash
# Terminal no VS Code (Ctrl+`)
cd pca10056/s340/armgcc

# Limpar build anterior
make clean

# Compilar
make -j8

# Flash (primeira vez - inclui SoftDevice)
make flash_softdevice
make flash

# Flash (atualizacoes)
make flash
```

---

## 3. Opcao B: nRF Connect SDK (Zephyr)

### 3.1 Requisitos

| Componente | Versao | Link |
|------------|--------|------|
| VS Code | Latest | https://code.visualstudio.com/download |
| nRF Connect for VS Code | Latest | VS Code Marketplace |
| nRF Connect SDK | 3.0.0+ | Via VS Code extension |
| nRF Command Line Tools | Latest | https://www.nordicsemi.com/Products/Development-tools/nrf-command-line-tools/download |

### 3.2 Instalacao nRF Connect for VS Code

1. Abrir VS Code

2. Extensions (Ctrl+Shift+X)

3. Buscar "nRF Connect for VS Code Extension Pack"

4. Instalar (Nordic Semiconductor)

5. Reiniciar VS Code

### 3.3 Configurar SDK via Extension

1. Clicar no icone nRF Connect na Activity Bar (lateral esquerda)

2. Welcome > Manage SDKs

3. Install SDK > Selecionar versao (recomendado: v3.0.0 ou mais recente)

4. Escolher diretorio: `C:\ncs` (curto, sem espacos!)

5. Aguardar download (~2-5 GB)

### 3.4 Criar Novo Projeto Zephyr

Para iniciar migracao:

1. nRF Connect > Create a new application

2. Selecionar:
   - Application type: Standalone
   - SDK version: v3.0.0+
   - Board: nrf52840dk_nrf52840 (ou custom)
   - Application template: Blank ou sample

3. Estrutura criada:
   ```
   my_app/
   ├── CMakeLists.txt
   ├── prj.conf
   ├── src/
   │   └── main.c
   └── boards/
       └── (custom overlays)
   ```

### 3.5 Build com nRF Connect

1. nRF Connect panel > Add Build Configuration

2. Selecionar board target

3. Clicar em "Build"

4. Flash via "Flash" button

### 3.6 Estrutura de Projeto Zephyr

```
stravaV10_zephyr/
├── CMakeLists.txt
├── prj.conf                 # Configuracao Kconfig
├── Kconfig                  # Custom configs
├── west.yml                 # Manifest (se standalone)
├── boards/
│   └── nrf52840_strava.overlay  # Device tree overlay
├── dts/
│   └── bindings/            # Custom DT bindings
├── src/
│   ├── main.c
│   ├── model/
│   ├── vue/
│   ├── rf/
│   └── drivers/
└── include/
    └── (headers)
```

---

## 4. Comparacao: nRF5 SDK vs nRF Connect SDK

| Aspecto | nRF5 SDK | nRF Connect SDK |
|---------|----------|-----------------|
| RTOS | Opcional (FreeRTOS) | Zephyr (obrigatorio) |
| Build System | Makefile/SES | CMake + west |
| Device Config | sdk_config.h | Kconfig + Device Tree |
| Debugging | Segger/GDB | nRF Connect Debug |
| BLE Stack | SoftDevice s340 | Zephyr BLE ou SoftDevice |
| ANT+ | Suportado (s340) | Requer SoftDevice |
| Manutencao | Legacy (sem updates) | Ativo |
| Documentacao | InfoCenter | docs.nordicsemi.com |

---

## 5. Estrategia de Migracao Recomendada

### Fase 1: Setup Paralelo
1. Manter build nRF5 SDK funcionando (Opcao A)
2. Instalar nRF Connect for VS Code (Opcao B)
3. Criar projeto Zephyr vazio para nRF52840

### Fase 2: Migracao HAL
1. Abstrair GPIO, SPI, I2C, UART no codigo atual
2. Criar drivers Zephyr equivalentes
3. Testar drivers isoladamente

### Fase 3: Migracao de Modulos
1. Portar drivers (ls027, sensors)
2. Portar model layer (Boucle, Attitude, Locator)
3. Portar RF (BLE primeiro, depois ANT+)
4. Portar Vue

### Fase 4: Integracao
1. Integrar todos modulos
2. Testes de sistema
3. Otimizacao de memoria e performance

---

## 6. Troubleshooting

### Build falha: "arm-none-eabi-gcc not found"
- Verificar PATH contem diretorio bin do GCC
- Reiniciar terminal/VS Code

### Build falha: "SDK_ROOT not found"
- Verificar Makefile.local tem path correto
- Usar barras "/" nao "\" no Windows

### Flash falha: "No J-Link found"
- Verificar J-Link conectado
- Instalar drivers J-Link
- Verificar nrfjprog funciona: `nrfjprog --ids`

### IntelliSense nao funciona
- Recarregar VS Code window
- Verificar c_cpp_properties.json tem paths corretos
- Rodar "C/C++: Reset IntelliSense Database"

---

## 7. Referencias

- [nRF5 SDK Download](https://www.nordicsemi.com/Products/Development-software/nRF5-SDK/Download)
- [nRF Connect for VS Code](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect-extension-pack)
- [nRF Connect SDK Docs](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/index.html)
- [GCC ARM Downloads](https://developer.arm.com/downloads/-/gnu-rm)
- [Nordic DevZone](https://devzone.nordicsemi.com/)
- [Zephyr Project Docs](https://docs.zephyrproject.org/)

