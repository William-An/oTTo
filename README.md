# oTTo

Purdue University ECE 47700 senior design project

## Intro

oTTo is a two-wheel self-balancing robot designed as an experimental hardware platform for control theory students to gain hands-on experience on real-world control problems. In addition to the microcontroller part, a MATLAB/Simulink interface will be also provided to facilitate the implementation of various controllers with Simulink.

### Main Components

1. [A4988 stepper motor driver](https://www.amazon.com/gp/product/B01FFGAKK8/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&psc=1#descriptionAndDetails)
2. [Adafruit TDK InvenSense ICM-20948 9-DoF IMU](https://www.adafruit.com/product/4554)
3. [ESP32 microcontroller](https://www.espressif.com/en/products/socs/esp32)
4. [Stepper Motor](https://www.amazon.com/gp/product/B07TJWMZZY/ref=ppx_yo_dt_b_search_asin_title?ie=UTF8&th=1)
5. [eInk Display](https://www.adafruit.com/product/4687)

### Concept Sketch

![Concept sketch](https://user-images.githubusercontent.com/19645713/119400346-0e936380-bca8-11eb-92e4-784602a6f2d4.png)

### Functional Diagrams

![Hardware platform functional diagram](https://user-images.githubusercontent.com/19645713/119400410-24a12400-bca8-11eb-8841-9a9575c44fbc.png)
![Simulink interface functional diagram](https://user-images.githubusercontent.com/19645713/119400416-266ae780-bca8-11eb-8e7e-a8d4a10a9e9a.png)

## Objectives

1. Hardware & Embedded Software
    1. Read IMU sensors
    2. Control motor via driver chips
    3. Host a UDP server for data transmission
    4. Send/receive sensor readings/control signals to/from main PC over UDP
2. MATLAB/Simulink
    1. Create Simulink block interface
    2. Use UDP client to send/receive information to/from hardware

## Resources

### Datasheet

1. [ESP32 chip datasheet](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
2. [ESP32 chip technical reference](https://www.espressif.com/sites/default/files/documentation/esp32_technical_reference_manual_en.pdf)
3. [A4988 stepper motor driver chip datasheet](https://www.pololu.com/file/0J450/A4988.pdf)
4. [Stepper Motor](https://datasheetspdf.com/pdf-file/928661/MotionKing/17HS4401/1)
5. [eInk Display Controller](https://cdn-learn.adafruit.com/assets/assets/000/092/748/original/SSD1675_0.pdf?1593792604)

### SDKs & Libraries

1. [ESP32 RTOS SDK](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/freertos.html)
2. [ESP32 IDF SDK](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/index.html), since the RTOS SDK is very close to it
3. [ICM-20948 driver](https://github.com/adafruit/Adafruit_ICM20X)
4. [Custom Simulink library](https://ww2.mathworks.cn/help/simulink/ug/creating-block-libraries.html?lang=en)
5. [Custom Simulink block](https://www.mathworks.com/help//simulink/ug/tutorial-creating-a-custom-block.html?requestedDomain=)
6. [MATLAB UDP functions](https://www.mathworks.com/help/instrument/udp-interface.html?s_tid=CRUX_lftnav)
7. [eInk Display Driver](https://github.com/adafruit/Adafruit_EPD)

### Debugging

1. [ESP32 IDF JTAG Debug Setup](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-guides/jtag-debugging/index.html#how-it-works)
2. [Openocd](http://openocd.org/)
3. [Openocd 8266 Port](https://github.com/sysprogs/esp8266-openocd)
   1. Not an official release from neither Espressif nor Openocd
   2. Though the Espressif mentions that they might support esp8266 with openocd in the future but it is not their first priority: [GitHub Issue](https://github.com/espressif/openocd-esp32/issues/111)

### Miscellaneous

1. [FreeRTOS](https://www.freertos.org/)
2. [ESP8266 UDP examples](https://github.com/espressif/ESP8266_RTOS_SDK/tree/master/examples/protocols/sockets)
