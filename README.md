# StepperOne Firmware

STM32G0-based firmware for the StepperOne wearable step counter and fitness tracker device.

## Overview

StepperOne is an embedded fitness tracking device that monitors physical activity using a 3-axis accelerometer. The firmware detects steps, analyzes movement patterns to determine activity pace (idle, walking, running), and displays real-time statistics on an integrated OLED display.

## Features

### Core Functionality
- **Step Detection**: Real-time step counting using accelerometer data analysis
- **Pace Recognition**: Automatic detection of user activity state (idle, walking, running)
- **Activity Tracking**:
  - Total steps since power-on
  - Steps per hour monitoring
  - Personal record tracking (max steps in a session)
- **Persistent Storage**: EEPROM emulation for saving calibration data and activity records

### Hardware Capabilities
- **Display**: OLED screen with multiple interface modes
- **User Input**: Physical button controls with debouncing
- **Visual Feedback**: LED indicators for system state
- **Battery Monitoring**: ADC-based voltage measurement
- **Sensor Calibration**: 3-axis accelerometer calibration system

## Hardware Platform

- **MCU**: STM32G071xx (ARM Cortex-M0+)
- **Peripherals**:
  - SPI1 (Display communication)
  - I2C1 (EEPROM interface)
  - USART2 (Debug/communication)
  - ADC1 (Accelerometer & battery monitoring)
  - CRC (Data integrity)

## Project Structure

```
stepperFW/
├── README.md
├── stepperFirmware/
│   ├── Core/
│   │   ├── Inc/
│   │   │   ├── main.h              # Main definitions and data structures
│   │   │   ├── accelerometer.h     # Step detection algorithms
│   │   │   ├── screens.h           # Display management
│   │   │   ├── buttons.h           # Input handling
│   │   │   ├── leds.h              # LED control
│   │   │   ├── helperFunc.h        # Utility functions
│   │   │   └── user.h              # User interaction logic
│   │   └── Src/
│   │       ├── main.c              # Application entry point
│   │       ├── accelerometer.c     # Step detection implementation
│   │       ├── screens.c           # Display rendering
│   │       ├── buttons.c           # Button processing
│   │       ├── leds.c              # LED patterns
│   │       └── helperFunc.c        # Utilities and EEPROM
│   ├── Drivers/                    # STM32 HAL drivers
│   └── stepperFirmware.ioc         # STM32CubeMX configuration
```

## Key Components

### Step Detection Algorithm
The firmware uses a weighted accelerometer approach to detect steps:
- Configurable acceleration thresholds (1.3g - 3.5g)
- Minimum step interval filtering (250ms)
- Z-axis dominant detection with adjustable weights
- Moving average pace calculation

### Display Screens
- **Homescreen**: Real-time steps, pace, and hourly statistics
- **Calibration**: Multi-axis sensor calibration interface
- **System Error**: Diagnostic information display
- **Reset**: Record clearing interface

### EEPROM Data Persistence
- Accelerometer calibration values (offset & sensitivity for X/Y/Z axes)
- Personal step records
- Flash-based emulation with power-loss protection
- CRC validation for data integrity

## Building the Project

### Prerequisites
- STM32CubeIDE or ARM GCC toolchain
- STM32CubeMX (for hardware configuration changes)
- ST-LINK debugger/programmer

### Build Steps
1. Clone the repository:
   ```bash
   git clone <repository-url>
   cd stepperFW
   ```

2. Open in STM32CubeIDE:
   - Import the `stepperFirmware` folder as an existing project
   - Build the project (Project → Build All)

3. Flash to device:
   - Connect ST-LINK programmer
   - Run → Debug (F11) or Flash directly

## Configuration

### Accelerometer Parameters
Edit `accelerometer.h` to adjust:
```c
#define MIN_ACCELERATION    1.3f    // Minimum g-force threshold
#define MIN_STEP_INTERVAL   250     // Milliseconds between steps
#define STEP_WEIGHTS        {0.0f, 0.0f, 1.0f}  // X, Y, Z axis weights
```

### Pace Detection Thresholds
```c
#define MIN_RUNNING_INTERVAL_MS   500    // 120 steps/min
#define MIN_WALKING_INTERVAL_MS   800    // 75 steps/min
#define MIN_IDLE_INTERVAL_MS      1000   // Idle timeout
```

## Usage

### First Time Setup
1. Power on the device
2. The system will initialize the EEPROM and display
3. Perform sensor calibration (follow on-screen instructions)
4. Device is ready for step tracking

### Calibration Procedure
The device supports automatic calibration for all three accelerometer axes:
- Follow the on-screen prompts to orient the device
- Press buttons when prompted to capture calibration points
- Calibration values are automatically saved to EEPROM

### Button Controls
- Navigation through menus
- Calibration control
- Record reset (with confirmation)

## Development Notes

### Memory Management
- EEPROM emulation uses two Flash pages for wear leveling
- Automatic page transfer on fill
- Power-failure recovery via PVD (Programmable Voltage Detector)

### Safety Features
- Flash lock/unlock protection
- CRC-based data validation
- Interrupt-driven power monitoring
- Debounced button inputs

## Recent Updates

- ✅ Fixed button input and UI rendering issues
- ✅ Implemented EEPROM-based record persistence
- ✅ Added 3-axis calibration system
- ✅ Migrated to I2C for EEPROM communication
- ⚠️ UI button centering requires adjustment (known issue)

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/improvement`)
3. Commit your changes (`git commit -m 'Add new feature'`)
4. Push to the branch (`git push origin feature/improvement`)
5. Open a Pull Request


## Acknowledgments

Built with:
- STM32CubeMX for hardware abstraction
- STM32 HAL drivers
- CMSIS for ARM Cortex-M support

---


