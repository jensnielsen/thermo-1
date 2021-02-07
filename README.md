# thermo-l1
Simple thermometer based on STM32L-DISCOVERY and ds18b20.

The internal temperature sensor on both my discovery boards apparently had no calibration, so this is a project based on the ST example but with ds18b20 instead. Measured 12.5 uA in idle.

Instructions:
* Get an appropriate battery holder for CR2032 coin cell battery, I used Harwin S8421-45R since I couldn't find the recommended B7410AP2L
* Modify the discovery board according to user manual to run from battery (remove SB100 & SB21, solder in battery holder & pin header at JP2)
* Remove jumpers at CN3 & JP1
* Do not set a jumper on JP2, instead connect wire from JP2 "VBAT" to JP1 middle pin (VDD_MCU) to power only mcu and not other stuff on the board
* Connect PD2 or VBAT to ds18b20 VDD (PD2 is output high since there's no pin with VBAT available and didn't want a breadboard)
* Connect GND to ds18b20 GND
* Connect PC12 to ds18b20 data
