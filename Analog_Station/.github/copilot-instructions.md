# Analog Station - AI Coding Instructions

## Project Overview
The Analog Station is an ESP32-C3 based dual-mode monitor (Audio Spectrum Analyzer & Environmental Monitor) written in C++ (Arduino framework). It combines vintage analog aesthetics (PWM-driven meters) with digital processing (FFT, BME280).

## Architecture & Core Concepts
- **Single-File Structure:** All logic resides in `Analog_Station.ino`.
- **Dual Modes:**
  - **Audio Mode:** Real-time FFT analysis (Low/Mid/High bands) driving analog meters.
  - **Sensor Mode:** BME280 readings (Temp/Humidity/Pressure) with calibration offsets.
- **State Management:** Global booleans (`sensorMode`, `calibrationAdjustMode`) control the active state.
- **Persistence:** Uses `Preferences` library to save calibration data (offsets, noise floor) to non-volatile storage.

## Hardware & Pinout (ESP32-C3)
- **Input:**
  - Microphone: `MIC_PIN` (ADC, 12-bit)
  - Toggle Switch: `TOGGLE_PIN` (Digital Input)
  - Sensor: BME280 via I2C (`SDA_PIN`, `SCL_PIN`)
- **Output:**
  - Meters: `METER_LOW`, `METER_MID`, `METER_HIGH` (PWM via `ledc`)
  - LEDs: `LED_LOW`, `LED_MID`, `LED_HIGH`, `BUILTIN_LED_C3`

## Key Patterns & Conventions

### 1. Signal Processing (Audio)
- **FFT Workflow:** Sample -> Remove DC -> Windowing (Hamming) -> Compute -> Magnitude.
- **Band Processing:**
  - Frequencies are binned into Low/Mid/High.
  - **Lambda Usage:** Use the `processBand` lambda for consistent Noise Gating -> Smoothing -> Scaling logic.
  - **Noise Floor:** Dynamic calibration via `calibrateNoiseFloor()`.

### 2. Sensor Handling & Calibration
- **Polling:** Sensors are read every 10s (normal) or 2s (adjustment mode).
- **Calibration:**
  - **Offsets:** Raw values are adjusted by `tempOffset`, `pressureOffset`, `humidityOffset`.
  - **Live Adjustment:** `+/-` serial commands modify offsets in real-time.
  - **Auto-Save:** Changes are saved to `Preferences` after `AUTO_SAVE_DELAY` idle time.

### 3. Serial Communication
- **Command Interface:** Single-char commands (`s`, `a`, `n`, `d`, `c`, `+`, `-`) control modes and calibration.
- **Debugging:** Use `Serial.printf()` for formatted status updates.
- **Blocking:** `waitForFloatInput()` is a blocking function used only during manual calibration wizards.

### 4. Timing & Concurrency
- **Non-Blocking Loop:** Use `millis()` for periodic tasks (sensor reads, auto-save).
- **Avoid `delay()`:** Keep the main loop tight for responsive audio visualization.

## Development Workflow
- **Libraries:** `Adafruit_BME280`, `arduinoFFT`, `Preferences`, `Wire`.
- **PWM:** Use ESP32 `ledcAttach` and `ledcWrite` (not `analogWrite`).
- **ADC:** Configured for 12-bit resolution (`analogReadResolution(12)`).

## Common Tasks
- **Adding a Command:** Update `checkSerialCommands()` switch statement.
- **Tuning Audio:** Adjust `SCALING_X`, `SMOOTHING_X`, or bin ranges in `processAudioMode()`.
- **Modifying Hardware:** Update `#define` pin assignments at the top of the file.
