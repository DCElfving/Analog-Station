# The Analog Station!
Arduino code and files for building The Analog Station, an ESP32 based device for audio visualization and indoor climate monitoring. Complete with a wieless web interface and vintage glow!

## Overview
Intended as a hands-on workshop in a kit, The Analog Station merges the tactile vibe of analog meters with modern sensor technologies and creative code.  

The goal is to share creative technology skills with artists and makers. If you're confident building on your own, order a PCB with the included Gerber file, source components, laser cut a faceplate (glue it at a 15º angle to some plywood), and solder one up. Check out the <a href="https://docs.google.com/presentation/d/1KcfG4H2xHetNgzfBoPxqG8-ehrnyuGnOou0_SotSfiw/edit?usp=sharing">build instructions and workshop guide</a> on Google Slides.

**Audio Mode**: Bass, mids, and treble each get their own meter. They respond to music, ambient noise, and voice.

**Sensor Mode**: Temperature, humidity, and barometric pressure displayed on analog dials.

### Hardware
- **ESP32-C3 SuperMini** microcontroller 
- **BME280 Environmental Sensor** 
- **MAX4466 Microphone** 
- **3x Analog Panel Volt Meters** (Uxcell DC 0-3V Analog Panel Voltage Gauge Volt Meter 85C1 or similar)
- **1x Toggle Switch 2 Pin 2 Position ON/Off 6A 125V MTS-101**
- **3x Warm White LEDs and resistors**
- **7X 2 Pin 2.54mm Pitch PCB Screw Terminal Block Connector**
- **22awg 0.35mm² Solid Wire for connecting meters and switch**
- **PCB** (see Gerber file in this repository)

![Analog Station Front](/images/analog1.jpeg)
![Analog Station Back](/images/analog2.jpeg)
![Analog Station circuit boards and faceplates](/images/analog3.jpeg)

## Contributing
This project is open for educational use and modification. If you build your own Analog Station or create improvements:
- Share your modifications and extensions
- Document your build process with photos
- Submit pull requests for code improvements
- Report issues or bugs

## License

See LICENSE file for details.
