Laser distance sensor
-----------------------


Firmware and hardware design for short-range laser distance sensor.

Preliminary feature list:
  
  * Based on [ST VL53L0X ToF distance sensor](http://www.st.com/en/imaging-and-photonics-solutions/vl53l0x.html),
  * Multiple ranging profiles/modes (default, high accuracy, long range),
  * Up to 2m range (indoors) in long range mode,
  * +/- 3mm accuracy (indoors) in hight accuracy mode,
  * On-board power regulation for 3.3-5.5V input,
  * On-board microcontroller [STM32F042](http://www.st.com/en/microcontrollers/stm32f042f6.html)
  * USB (CDC) and TTL serial outputs,
  * 24x24mm size, M3 mounting holes.

![Rev1 PCB render](https://raw.githubusercontent.com/festlv/laser-distance-sensor/master/doc/pcb-rev1-screenshot.jpg)


HERE BE DRAGONS
===================

This project is WIP. Specs might be inaccurate, hardware might let out smoke,
firmware might not compile.

Current status
=================

Hardware: first prototypes have been ordered.
Firmware: waiting on hardware.
