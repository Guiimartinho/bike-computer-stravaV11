# Bike Computer - Strava V11

Open-source bicycle GPS computer with Strava Segments support, migrated to **Zephyr RTOS**.

## About This Project

This project is a **fork and evolution** of the excellent [stravaV10](https://github.com/vincent290587/stravaV10) by **Vincent Manoukian** ([@vincent290587](https://github.com/vincent290587)).

### What Changed

The original project used **nRF5 SDK** with a monolithic C++ architecture. This fork migrates the firmware to:

- **Zephyr RTOS** with nRF Connect SDK 3.1.0
- **Modular C architecture** following MISRA C:2012 guidelines
- **Clean separation** between HAL, drivers, model, and UI layers

### Why Zephyr?

- Modern RTOS with active development
- Better tooling (west, devicetree, Kconfig)
- Easier portability to other platforms
- Native support for BLE 5.x features
- Integrated power management

## Features

All original features are being preserved:

- Real-time compete with Strava Segments (500+)
- GPX file following with zoom options
- ANT+ connection (HRM, FE-C, BSC)
- BLE connection (LNS, UART, Komoot)
- Power estimation using barometer + algorithms
- Low power: <8mA indoor, ~35mA outdoor

## Hardware

- **MCU**: Nordic nRF52840 (Cortex-M4F)
- **Display**: Sharp Memory LCD LS027B4DH01
- **Sensors**: Barometer, Accelerometer, Fuel Gauge
- **PCB**: [EAGLE Project](https://github.com/vincent290587/EAGLE/tree/master/Projects/myStravaB_V3)

## Project Structure

```
bike-computer-stravaV11/
├── zephyr_app/          # Active development - Zephyr RTOS
│   ├── src/
│   │   ├── hal/         # Hardware Abstraction Layer
│   │   ├── drivers/     # Device drivers (LCD, GPS, sensors)
│   │   ├── model/       # Business logic (Kalman, segments, zones)
│   │   ├── rf/          # BLE communication
│   │   └── vue/         # UI layer
│   └── include/         # Public headers
├── legacy/              # Original nRF5 SDK code (reference)
├── libraries/           # Shared third-party libraries
├── tools/               # TDD simulator, debug tools
└── docs/                # Documentation
```

## Building

### Zephyr App (New)

```bash
cd zephyr_app
west build -b nrf52840dk_nrf52840
west flash
```

### Legacy (Reference Only)

The original nRF5 SDK build is preserved in `legacy/` for reference:
```bash
cd legacy/pca10056/s340/armgcc
make
```

## Documentation

- [Firmware Architecture](docs/00_Firmware_Architecture.md)
- [Code Review Standards](docs/01_Code_Review_MISRA.md)
- [Build Environment Setup](docs/02_Build_Environment_Setup.md)
- [Migration Gap Analysis](docs/02_Gap_Analysis_Original_vs_Zephyr.md)

## Credits

This project is based on and inspired by:

- **[stravaV10](https://github.com/vincent290587/stravaV10)** by Vincent Manoukian
  - Original firmware architecture
  - Hardware design (PCB, schematics)
  - Strava segment algorithms
  - Power estimation algorithms
  - All the hard work that made this possible

## Screenshots

![Front](docs/front1.png)

**Outdoor modes:**

![CRS](docs/crs.png) ![CRS 2 Segments](docs/crs_2seg.png) ![PRC](docs/prc.png)

**Indoor mode:**

![FEC](docs/FEC.png)

**Menu:**

![Menu 1](docs/menu1.png) ![Menu 2](docs/menu2.png)

## License

This project maintains the same license as the original stravaV10 project.
See [LICENSE.md](LICENSE.md) for details.

## Contributing

Contributions are welcome! Please read the documentation in `docs/` before submitting PRs.
