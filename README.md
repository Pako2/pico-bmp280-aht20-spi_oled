### Purpose
The purpose of this "project" was to test several things:
- RP2040 Pico programming in the VSC/PlatformIO environment
- work with both RP2040 cores
- simultaneous communication via SPI (display) and I2C (sensor)
- display of measured values ​​as running UTF-8 text on the display

### Components used
SBC:&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;RP2040 Pico\
Sensor:&nbsp;&nbsp;&nbsp;BMP280+AHT20\
Display:&nbsp;&nbsp;0.96 inch SPI OLED with SSD1306 controller

### Connection
I wanted to limit the self-heating of the sensor as much as possible, and therefore the power supply of the sensor is connected to one GPIO and it is always switched on only for the moment of data acquisition.
The following table shows the pin layout used to connect peripherals to the Pico:

| Signal     | RP2040 Pico | SPI OLED Display | BMP280+AHT20 I2C |
|:----------:|:-----------:|:----------------:|:----------------:|
| GND        | GND         | 1/GND            | 2/GND            |
| VDD        | 36/3V3      | 2/VDD            |                  |
| SCK (SPI)  | 25/GPIO-19  | 3/SCK            |                  |
| SDA (SPI)  | 24/GPIO-18  | 4/SDA            |                  |
| RES (SPI)  | 26/GPIO-20  | 5/RES            |                  |
| DC (SPI)   | 27/GPIO-21  | 6/DC             |                  |
| CS (SPI)   | 29/GPIO-22  | 7/CS             |                  |
| SENSOR VDD | 9/GPIO-6    |                  | 4/VDD            |
| SDA (I2C)  | 6/GPIO-4    |                  | 3/SDA            |
| SCL (I2C)  | 7/GPIO-5    |                  | 1/SCL            |
<img src="assets\fritzing_bb.webp" alt="breadboard" title="breadboard" height="320"/> 

### Firmware
Core 0 is only used for display control.\
Core 1 is used for communication with the sensor and calculation of values ​​to be displayed. The measurement is performed once every 30 seconds.

### License
The code parts written by the author of the **pico-bmp280-aht20-spi_oled** project are licensed under [MIT License](https://github.com/Pako2/pico-bmp280-aht20-spi_oled/blob/master/LICENSE), 3rd party libraries that are used by this project are licensed under different license schemes, please check them out as well.
