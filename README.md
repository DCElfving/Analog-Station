# The Analog Station

An ESP32-based "Hi-Fi" device for audio visualization and indoor climate monitoring. Merging the tactile vibe of analog voltage meters with modern sensor technologies and a retro-styled wireless web interface.

![Analog Station Front](/images/analog1.jpeg)

## Features

*   **Audio Visualizer**: 3-Band Spectrum Analyzer (Bass, Mids, Treble) displayed on physical analog meters.
*   **Environmental Monitor**: Precision Temperature, Humidity, and Pressure readings via BME280.
*   **Web Interface**: A mobile-responsive, 80s Hi-Fi styled web app hosted directly on the ESP32.
    *   **Accordion UI**: Seamless switching between Audio, Sensor, and Calibration modes.
    *   **Real-time Feedback**: View live sensor data and audio levels in the browser.
    *   **Calibration**: Fine-tune meter needles, noise floor, and sensitivity wirelessly.

## Hardware Requirements

*   **ESP32-C3 SuperMini** (or compatible ESP32 board)
*   **BME280** Environmental Sensor (I2C) - *Ensure address 0x76 or 0x77*
*   **MAX4466** Electret Microphone Amplifier
*   **3x Analog Panel Volt Meters** (DC 0-3V, e.g., 85C1)
*   **1x SPST Toggle Switch** (Optional hardware mode toggle)
*   **3x Warm White LEDs** (Meter backlights)

## Software Setup

### 1. Library Dependencies
Install the following libraries via the Arduino Library Manager:
*   **Adafruit Unified Sensor** (Adafruit)
*   **Adafruit BME280 Library** (Adafruit)
*   **arduinoFFT** (Enrique Condés) -> *Important: Use Version 2.0.0+*

### 2. Filesystem (LittleFS)
The web interface (HTML/CSS/JS) is stored in the ESP32's flash memory. You **must** upload the data folder separately from the code.

1.  **Install LittleFS Uploader**:
    *   *Arduino IDE 1.8*: Install the [ESP32 Sketch Data Upload](https://github.com/lorol/arduino-esp32fs-plugin) plugin.
    *   *PlatformIO*: Use the `Upload Filesystem Image` task.
2.  **Upload**:
    *   Connect your ESP32.
    *   Select **Tools > ESP32 Sketch Data Upload**.
    *   Wait for the "LittleFS Image Uploaded" message.

### 3. Usage
1.  **Power On**: The device creates a WiFi Access Point named `Analog Station [ID]`.
2.  **Connect**: Join the WiFi network (no password by default).
3.  **Browse**: Open `http://192.168.4.1` in your browser.
4.  **Configure**:
    *   **Audio Mode**: Adjust sensitivity sliders for Low/Mid/High bands.
    *   **Sensor Mode**: View environmental data.
    *   **Calibrate**: Use the "Nudge" sliders to align physical needles with digital values.

## License
See [LICENSE](LICENSE) file for details.

---
Original concept created for teaching and learning creative technology skills.
[Build Guide Presentation](https://docs.google.com/presentation/d/1KcfG4H2xHetNgzfBoPxqG8-ehrnyuGnOou0_SotSfiw/edit?usp=sharing)
