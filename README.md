# micropython-room-co2

This project builds on the [smaller room sensor project](https://github.com/msgarbossa/micropython-room).  It uses a larger, but similarly styled case to allow room for the MH-Z19B CO2 sensor and additional airflow.  It uses MicroPython on ESP32 with an SSD1306 OLED and has a few options for I2C temperature sensors (BME280, ATH10).  ESP32's deep sleep is used with 2 wakeup sources, either by timer every 5 minutes or when the capacitive touch sensor is triggered.  The OLED only turns on when powered on or when the capacitive touch is triggered in order to limit light pollution in a dark room.  The anytemp module has a class to abstract different sensors to make them easier to swap since reading a temperature sensor tends to have the same interface.

## Diagram

2 separate I2C busses are used.  See [note about MicroPython and I2C](https://msgarbossa.github.io/documentation/MicroPython/libraries.html#note-about-micropython-and-i2c)

![wiring diagram](/img/esp32_temp_co2.png)

## Using the code

- copy boot.py.sample to boot.py and update variables
- echo "PASS = 'password'" > webrepl_cfg.py
- [upload all *.py files](https://msgarbossa.github.io/documentation/MicroPython/ampy.html) to ESP32 controller [flashed with MicroPython](https://msgarbossa.github.io/documentation/MicroPython/flash_firmware.html)

## 3D printed case

See [3d-printed-case](./3d-printed-case) for STL and FreeCAD files.  The FreeCAD file contains a spreadsheet to adjust most dimensions.

![front](/img/esp32-station-wide-front.png)

![back](/img/esp32-station-wide-back.png)

![side](/img/esp32-station-wide-inside.png)

