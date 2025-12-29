# Analog Station - Workshop Guide
Welcome to the Analog Station! Follow these steps to connect to your device and calibrate it.

## 1. Power Up & Identify
1.  Connect your ESP32-C3 module to power (USB).
2.  Wait a few seconds for the device to start.
3.  Open your computer or phone's Wi-Fi settings.
4.  Look for a network named **"Analog Station XXXX"** (where XXXX are 4 unique characters, e.g., "A1B2").
    *   *Note: If you see multiple "Analog Station" networks, check the label on your device or ask an instructor.*

## 2. Connect
1.  Select the network **"Analog Station XXXX"**.
2.  There is **no password**.
3.  Once connected, your device may warn you that there is no internet access. This is normal; stay connected.

## 3. Configure & Calibrate
1.  Open a web browser (Chrome, Safari, etc.).
2.  Type **`192.168.4.1`** into the address bar and press Enter.
3.  You should see the **Analog Station Config** page.

### Sensor Calibration (Sensor Mode)
*   **Aligning the Needles:**
    *   Look at the physical meters and the printed stickers.
    *   Use the **[ + ]** and **[ - ]** buttons in the web interface to nudge the needle until it points to the correct value.
    *   *Example:* If the room is 72°F, tap the buttons until the Temperature needle points to 72.
    *   The settings will **auto-save** after 3 seconds.

### Audio Tuning (Audio Mode)
*   **Scaling Factors:**
    *   These numbers control how sensitive the meters are to sound.
    *   **Lower number = More sensitive** (Meter moves more).
    *   **Higher number = Less sensitive** (Meter moves less).
    *   *Typical values:* Low: `45`, Mid: `20`, High: `5`.
    *   Play some music and adjust these values until the meters bounce nicely without hitting the maximum (pegging) too often.

## 4. Save Changes
1.  Click the **"Save Settings"** button.
2.  The device will save your settings to its permanent memory and reload the page.

## 5. Troubleshooting
If you cannot connect via Wi-Fi, you can use the Serial Monitor:
1.  Open the Arduino IDE or a Serial Terminal.
2.  Select the correct COM port for your device.
3.  She Serial Monitor will display your **Wi-Fi Name** and **IP Address** when the device starts.
5.  It will also show the accurate sensor readings (unaffected by your needle adjustments) and IP address.
