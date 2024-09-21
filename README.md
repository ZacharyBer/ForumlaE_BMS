# ForumlaE_BMS
Formula E BMS State Machine Project

Slides containing the state machine diagram:
https://docs.google.com/presentation/d/1_cIkpFZxs-6S7Ug5nnC2FRThi5nA-JUUAkOwVYvHE0g/edit?usp=sharing 

This project aims to simulate the behavior of a Formula E Battery Management System (BMS) on a NUCLEO-L476RG
The inputs to this simulation are:
2 ADC inputs
3 Buttons
1 Encoder

The outputs are:
Serial Print (picocom tty.___ -b 115200)
1 LED

The ADC's act as 1 cell's input with a voltage and temperature measurement

The buttons can:
Wake the system from sleep
Enter cell balancing
Enter Self-Discharge

The encoder serves to give an overall current reading

The LED serves to indicate the state of TS

Printed are:
The current state
The current reading
The cell voltage
The cell temperature

This is all encompassed in the skeleton of a program which is set to utiliza SPI and CAN over multiple cells, enable discharge circuitry, etc while also reading back the most information possible and indicating any rule infracton as per the Formula SAE rulebook

Please feel free to recreate using the ioc for setup, all code written is in the Core/src/main.c
