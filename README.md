# MBLDC
Custom PCB design which serves as a ESC or BLDC controller, to drive brushless BLDC motors using my own control algorithms.

[Read more on maakbaas.com](https://maakbaas.com/diy-field-oriented-control-esc/)

## Features of the PCB
- Three low side phase current shunt resistors
- Phase voltage measurements
- Connector for hall sensors or an encoder
- Max 10S, absolute maximum voltage is 45V 
- External communication using I2C or a custom protocol

## Features of the firmware
- Sensorless block commutation
- Bi-directional
- Open loop start
- Duty ratio control
- Current control 
- Speed control
- Controllable through the debug port 

## Photo of the PCB
![](https://cdn.hackaday.io/images/5273121611317168532.jpg)
