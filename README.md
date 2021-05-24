# oTTo
Purdue University ECE 47700 senior design project

## Intro

oTTo is a two-wheel self-balancing robot designed as an experimental hardware platform for control theory students to gain hands-on experience on real-world control problems. In addition to the microcontroller part, a MATLAB/Simulink interface will be also provided to facilitate the implementation of various controllers with Simulink.

### Main Components

1. [A4988 stepper motor driver](https://www.pololu.com/product/1182)
2. [Adafruit BNO085 9-DOF IMU](https://www.adafruit.com/product/4754)
3. [ESP8266 microcontroller](https://www.espressif.com/en/products/socs/esp8266)

### Concept Sketch:
![Concept sketch](https://user-images.githubusercontent.com/19645713/119400346-0e936380-bca8-11eb-92e4-784602a6f2d4.png)

### Functional Diagrams:
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

1. [ESP8266 chip datasheet](https://espressif.com/sites/default/files/documentation/0a-esp8266ex_datasheet_en.pdf)
2. [ESP8266 chip technical reference](https://www.espressif.com/sites/default/files/documentation/esp8266-technical_reference_en.pdf)
3. [A4988 stepper motor driver chip datasheet](https://www.pololu.com/file/0J450/A4988.pdf)

### SDKs & Libraries

1. [ESP8266 RTOS SDK](https://docs.espressif.com/projects/esp8266-rtos-sdk/en/latest/get-started/index.html)
2. [BNO080/BNO085 driver](https://github.com/hcrest/bno080-driver)
3. [Custom Simulink library](https://ww2.mathworks.cn/help/simulink/ug/creating-block-libraries.html?lang=en)
4. [Custom Simulink block](https://www.mathworks.com/help//simulink/ug/tutorial-creating-a-custom-block.html?requestedDomain=)
5. [MATLAB UDP functions](https://www.mathworks.com/help/instrument/udp-interface.html?s_tid=CRUX_lftnav)

### Miscellaneous

1. [FreeRTOS](https://www.freertos.org/)
2. [ESP8266 UDP examples](https://github.com/espressif/ESP8266_RTOS_SDK/tree/master/examples/protocols/sockets)
