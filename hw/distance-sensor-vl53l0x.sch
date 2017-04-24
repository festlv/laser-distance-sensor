EESchema Schematic File Version 2
LIBS:power
LIBS:device
LIBS:transistors
LIBS:conn
LIBS:linear
LIBS:regul
LIBS:74xx
LIBS:cmos4000
LIBS:adc-dac
LIBS:memory
LIBS:xilinx
LIBS:microcontrollers
LIBS:dsp
LIBS:microchip
LIBS:analog_switches
LIBS:motorola
LIBS:texas
LIBS:intel
LIBS:audio
LIBS:interface
LIBS:digital-audio
LIBS:philips
LIBS:display
LIBS:cypress
LIBS:siliconi
LIBS:opto
LIBS:atmel
LIBS:contrib
LIBS:valves
LIBS:sensors
LIBS:_sensors
LIBS:custom-power
LIBS:stm32
LIBS:mic5504
LIBS:distance-sensor-vl53l0x-cache
EELAYER 25 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title ""
Date ""
Rev ""
Comp ""
Comment1 ""
Comment2 ""
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L VL53L0X U3
U 1 1 58FB95F8
P 9450 1600
F 0 "U3" H 9750 2150 60  0000 C CNN
F 1 "VL53L0X" H 9250 2150 60  0000 C CNN
F 2 "_sensors:VL53L0X" H 9500 1550 60  0001 C CNN
F 3 "" H 9500 1550 60  0000 C CNN
	1    9450 1600
	1    0    0    -1  
$EndComp
$Comp
L +2.8V #PWR01
U 1 1 58FB9E6C
P 8650 750
F 0 "#PWR01" H 8650 710 30  0001 C CNN
F 1 "+2.8V" H 8650 860 30  0000 C CNN
F 2 "" H 8650 750 60  0001 C CNN
F 3 "" H 8650 750 60  0001 C CNN
	1    8650 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 750  8650 1300
Wire Wire Line
	8250 1200 8850 1200
Wire Wire Line
	8650 1300 8850 1300
Connection ~ 8650 1200
$Comp
L C C6
U 1 1 58FB9E8D
P 8250 1400
F 0 "C6" H 8275 1500 50  0000 L CNN
F 1 "0.1uF" H 8275 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 8288 1250 50  0001 C CNN
F 3 "" H 8250 1400 50  0001 C CNN
	1    8250 1400
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 58FB9EDE
P 8500 1400
F 0 "C7" H 8525 1500 50  0000 L CNN
F 1 "4.7uF" H 8525 1300 50  0000 L CNN
F 2 "Capacitors_SMD:C_0805" H 8538 1250 50  0001 C CNN
F 3 "" H 8500 1400 50  0001 C CNN
	1    8500 1400
	1    0    0    -1  
$EndComp
Wire Wire Line
	8500 1250 8500 1200
Wire Wire Line
	8250 1200 8250 1250
Connection ~ 8500 1200
Wire Wire Line
	8250 1550 8800 1550
Wire Wire Line
	8800 1500 8800 2100
Wire Wire Line
	8800 1500 8850 1500
Wire Wire Line
	8800 1600 8850 1600
Connection ~ 8800 1550
Wire Wire Line
	8800 1700 8850 1700
Connection ~ 8800 1600
Wire Wire Line
	8800 1800 8850 1800
Connection ~ 8800 1700
Wire Wire Line
	8800 1900 8850 1900
Connection ~ 8800 1800
Connection ~ 8500 1550
Wire Wire Line
	10050 1600 11100 1600
Text Label 10050 1500 0    60   ~ 0
SDA
Text Label 10050 1600 0    60   ~ 0
SCL
Wire Wire Line
	10050 1300 10600 1300
Wire Wire Line
	10050 1200 10400 1200
Wire Wire Line
	10400 1200 10400 1000
Wire Wire Line
	10600 1300 10600 1000
$Comp
L +2.8V #PWR02
U 1 1 58FBA163
P 10500 550
F 0 "#PWR02" H 10500 510 30  0001 C CNN
F 1 "+2.8V" H 10500 660 30  0000 C CNN
F 2 "" H 10500 550 60  0001 C CNN
F 3 "" H 10500 550 60  0001 C CNN
	1    10500 550 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10400 550  11100 550 
Wire Wire Line
	10600 550  10600 700 
Wire Wire Line
	10400 550  10400 700 
Connection ~ 10500 550 
$Comp
L R R9
U 1 1 58FBA1E2
P 10900 850
F 0 "R9" V 10980 850 50  0000 C CNN
F 1 "4K7" V 10900 850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 10830 850 50  0001 C CNN
F 3 "" H 10900 850 50  0001 C CNN
	1    10900 850 
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 58FBA21C
P 11100 850
F 0 "R10" V 11180 850 50  0000 C CNN
F 1 "4K7" V 11100 850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 11030 850 50  0001 C CNN
F 3 "" H 11100 850 50  0001 C CNN
	1    11100 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	10900 1000 10900 1500
Wire Wire Line
	11100 1600 11100 1000
Wire Wire Line
	10900 1500 10050 1500
Wire Wire Line
	10900 550  10900 700 
Connection ~ 10600 550 
Wire Wire Line
	11100 550  11100 700 
Connection ~ 10900 550 
Connection ~ 8800 1900
$Comp
L GND #PWR03
U 1 1 58FBABCC
P 8800 2100
F 0 "#PWR03" H 8800 2100 30  0001 C CNN
F 1 "GND" H 8800 2030 30  0001 C CNN
F 2 "" H 8800 2100 60  0001 C CNN
F 3 "" H 8800 2100 60  0001 C CNN
	1    8800 2100
	1    0    0    -1  
$EndComp
$Comp
L STM32F042F6Px U1
U 1 1 58FBB0AA
P 5100 3300
F 0 "U1" H 2300 4225 50  0000 L BNN
F 1 "STM32F042F6Px" H 7900 4225 50  0000 R BNN
F 2 "Housings_SSOP:TSSOP-20_4.4x6.5mm_Pitch0.65mm" H 7900 4175 50  0001 R TNN
F 3 "" H 5100 3300 50  0001 C CNN
	1    5100 3300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 1600 5000 2300
Wire Wire Line
	5100 2300 5100 1900
Wire Wire Line
	5100 1900 5000 1900
Connection ~ 5000 1900
$Comp
L +3.3V #PWR04
U 1 1 58FBB1B4
P 5000 1600
F 0 "#PWR04" H 5000 1560 30  0001 C CNN
F 1 "+3.3V" H 5000 1710 30  0000 C CNN
F 2 "" H 5000 1600 60  0001 C CNN
F 3 "" H 5000 1600 60  0001 C CNN
	1    5000 1600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR05
U 1 1 58FBB210
P 5100 4600
F 0 "#PWR05" H 5100 4600 30  0001 C CNN
F 1 "GND" H 5100 4530 30  0001 C CNN
F 2 "" H 5100 4600 60  0001 C CNN
F 3 "" H 5100 4600 60  0001 C CNN
	1    5100 4600
	1    0    0    -1  
$EndComp
Wire Wire Line
	5100 4600 5100 4200
$Comp
L C C3
U 1 1 58FBB2A6
P 5450 1850
F 0 "C3" H 5475 1950 50  0000 L CNN
F 1 "0.1uF" H 5475 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5488 1700 50  0001 C CNN
F 3 "" H 5450 1850 50  0001 C CNN
	1    5450 1850
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 58FBB2F6
P 5700 1850
F 0 "C4" H 5725 1950 50  0000 L CNN
F 1 "1uF" H 5725 1750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5738 1700 50  0001 C CNN
F 3 "" H 5700 1850 50  0001 C CNN
	1    5700 1850
	1    0    0    -1  
$EndComp
Wire Wire Line
	5700 1600 5700 1700
Wire Wire Line
	5000 1600 5700 1600
Wire Wire Line
	5450 1600 5450 1700
Connection ~ 5450 1600
Wire Wire Line
	5450 2000 5700 2000
$Comp
L GND #PWR06
U 1 1 58FBB3D6
P 5550 2150
F 0 "#PWR06" H 5550 2150 30  0001 C CNN
F 1 "GND" H 5550 2080 30  0001 C CNN
F 2 "" H 5550 2150 60  0001 C CNN
F 3 "" H 5550 2150 60  0001 C CNN
	1    5550 2150
	1    0    0    -1  
$EndComp
Wire Wire Line
	5550 2150 5550 2000
Connection ~ 5550 2000
$Comp
L R R2
U 1 1 58FBB49E
P 1750 2250
F 0 "R2" V 1830 2250 50  0000 C CNN
F 1 "4K7" V 1750 2250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1680 2250 50  0001 C CNN
F 3 "" H 1750 2250 50  0001 C CNN
	1    1750 2250
	1    0    0    -1  
$EndComp
Wire Wire Line
	1250 2700 2200 2700
Wire Wire Line
	1750 2700 1750 2400
$Comp
L C C1
U 1 1 58FBB584
P 1750 2850
F 0 "C1" H 1775 2950 50  0000 L CNN
F 1 "0.1uF" H 1775 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 1788 2700 50  0001 C CNN
F 3 "" H 1750 2850 50  0001 C CNN
	1    1750 2850
	1    0    0    -1  
$EndComp
Connection ~ 1750 2700
$Comp
L GND #PWR07
U 1 1 58FBB5FE
P 1750 3150
F 0 "#PWR07" H 1750 3150 30  0001 C CNN
F 1 "GND" H 1750 3080 30  0001 C CNN
F 2 "" H 1750 3150 60  0001 C CNN
F 3 "" H 1750 3150 60  0001 C CNN
	1    1750 3150
	1    0    0    -1  
$EndComp
Wire Wire Line
	1750 3150 1750 3000
Wire Wire Line
	1750 2100 1750 1800
$Comp
L +3.3V #PWR08
U 1 1 58FBB6B1
P 1750 1800
F 0 "#PWR08" H 1750 1760 30  0001 C CNN
F 1 "+3.3V" H 1750 1910 30  0000 C CNN
F 2 "" H 1750 1800 60  0001 C CNN
F 3 "" H 1750 1800 60  0001 C CNN
	1    1750 1800
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 3400 1650 3400
Wire Wire Line
	2200 3500 1650 3500
Text Label 1650 3400 0    60   ~ 0
SDA
Text Label 1650 3500 0    60   ~ 0
SCL
Wire Wire Line
	8000 3500 8600 3500
Wire Wire Line
	8000 3600 8600 3600
Text Label 8250 3500 0    60   ~ 0
USB_D_N
Text Label 8250 3600 0    60   ~ 0
USB_D_P
Wire Wire Line
	8000 3700 8600 3700
Text Label 8250 3700 0    60   ~ 0
SWDIO
$Comp
L CONN_02X05 J3
U 1 1 58FBBE01
P 8900 5100
F 0 "J3" H 8900 5400 50  0000 C CNN
F 1 "CONN_02X05" H 8900 4800 50  0000 C CNN
F 2 "pin_headers_1.27mm:Pin_Header_1.27mm_2x5" H 8900 3900 50  0001 C CNN
F 3 "" H 8900 3900 50  0001 C CNN
	1    8900 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 4900 8350 4900
Wire Wire Line
	8350 4900 8350 4750
$Comp
L +3.3V #PWR09
U 1 1 58FBBF30
P 8350 4750
F 0 "#PWR09" H 8350 4710 30  0001 C CNN
F 1 "+3.3V" H 8350 4860 30  0000 C CNN
F 2 "" H 8350 4750 60  0001 C CNN
F 3 "" H 8350 4750 60  0001 C CNN
	1    8350 4750
	1    0    0    -1  
$EndComp
Wire Wire Line
	8650 5000 8350 5000
Wire Wire Line
	8350 5000 8350 5600
Wire Wire Line
	8650 5100 8350 5100
Connection ~ 8350 5100
Wire Wire Line
	8650 5300 8350 5300
Connection ~ 8350 5300
$Comp
L GND #PWR010
U 1 1 58FBC0C1
P 8350 5600
F 0 "#PWR010" H 8350 5600 30  0001 C CNN
F 1 "GND" H 8350 5530 30  0001 C CNN
F 2 "" H 8350 5600 60  0001 C CNN
F 3 "" H 8350 5600 60  0001 C CNN
	1    8350 5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	9150 4900 9750 4900
Text Label 9450 4900 0    60   ~ 0
SWDIO
Wire Wire Line
	9150 5000 9750 5000
Text Label 9450 5000 0    60   ~ 0
SWCLK
Wire Wire Line
	9150 5100 9750 5100
Text Label 9450 5100 0    60   ~ 0
SWO
Wire Wire Line
	9150 5300 9750 5300
Text Label 9450 5300 0    60   ~ 0
NRST
NoConn ~ 9150 5200
Text Label 1250 2700 0    60   ~ 0
NRST
Wire Wire Line
	8000 3800 8600 3800
Text Label 8250 3800 0    60   ~ 0
SWCLK
Wire Wire Line
	8000 2900 8600 2900
Wire Wire Line
	8000 2800 8600 2800
Wire Wire Line
	8000 3000 8600 3000
Text Label 8200 2800 0    60   ~ 0
USART_DE
Text Label 8200 2900 0    60   ~ 0
USART_TX
Text Label 8200 3000 0    60   ~ 0
USART_RX
Wire Wire Line
	1450 3800 2200 3800
Text Label 1650 3800 0    60   ~ 0
BOOT0
$Comp
L R R1
U 1 1 58FBC736
P 1450 4000
F 0 "R1" V 1530 4000 50  0000 C CNN
F 1 "4K7" V 1450 4000 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 1380 4000 50  0001 C CNN
F 3 "" H 1450 4000 50  0001 C CNN
	1    1450 4000
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 3800 1450 3850
$Comp
L GND #PWR011
U 1 1 58FBC817
P 1450 4350
F 0 "#PWR011" H 1450 4350 30  0001 C CNN
F 1 "GND" H 1450 4280 30  0001 C CNN
F 2 "" H 1450 4350 60  0001 C CNN
F 3 "" H 1450 4350 60  0001 C CNN
	1    1450 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 4350 1450 4150
Wire Wire Line
	8000 3100 9100 3100
Wire Wire Line
	9100 3100 9100 2950
$Comp
L R R5
U 1 1 58FBC9CC
P 9100 2800
F 0 "R5" V 9180 2800 50  0000 C CNN
F 1 "220R" V 9100 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 9030 2800 50  0001 C CNN
F 3 "" H 9100 2800 50  0001 C CNN
	1    9100 2800
	1    0    0    -1  
$EndComp
$Comp
L LED D1
U 1 1 58FBCA4F
P 9100 2400
F 0 "D1" H 9100 2500 50  0000 C CNN
F 1 "LED" H 9100 2300 50  0000 C CNN
F 2 "LEDs:LED_0603" H 9100 2400 50  0001 C CNN
F 3 "" H 9100 2400 50  0001 C CNN
	1    9100 2400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9100 2550 9100 2650
$Comp
L +3.3V #PWR012
U 1 1 58FBCB3F
P 9100 2200
F 0 "#PWR012" H 9100 2160 30  0001 C CNN
F 1 "+3.3V" H 9100 2310 30  0000 C CNN
F 2 "" H 9100 2200 60  0001 C CNN
F 3 "" H 9100 2200 60  0001 C CNN
	1    9100 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9100 2200 9100 2250
Wire Wire Line
	8000 3200 9550 3200
Wire Wire Line
	9550 3200 9550 2950
$Comp
L R R6
U 1 1 58FBCE70
P 9550 2800
F 0 "R6" V 9630 2800 50  0000 C CNN
F 1 "220R" V 9550 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 9480 2800 50  0001 C CNN
F 3 "" H 9550 2800 50  0001 C CNN
	1    9550 2800
	1    0    0    -1  
$EndComp
$Comp
L LED D2
U 1 1 58FBCE77
P 9550 2400
F 0 "D2" H 9550 2500 50  0000 C CNN
F 1 "LED" H 9550 2300 50  0000 C CNN
F 2 "LEDs:LED_0603" H 9550 2400 50  0001 C CNN
F 3 "" H 9550 2400 50  0001 C CNN
	1    9550 2400
	0    -1   -1   0   
$EndComp
Wire Wire Line
	9550 2550 9550 2650
$Comp
L +3.3V #PWR013
U 1 1 58FBCE7F
P 9550 2200
F 0 "#PWR013" H 9550 2160 30  0001 C CNN
F 1 "+3.3V" H 9550 2310 30  0000 C CNN
F 2 "" H 9550 2200 60  0001 C CNN
F 3 "" H 9550 2200 60  0001 C CNN
	1    9550 2200
	1    0    0    -1  
$EndComp
Wire Wire Line
	9550 2200 9550 2250
$Comp
L USB_OTG J2
U 1 1 58FBD014
P 2250 6250
F 0 "J2" H 2050 6700 50  0000 L CNN
F 1 "USB_OTG" H 2050 6600 50  0000 L CNN
F 2 "Connectors_Molex:USB_Micro-B_Molex_47346-0001" H 2400 6200 50  0001 C CNN
F 3 "" H 2400 6200 50  0001 C CNN
	1    2250 6250
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 6050 2900 6050
Wire Wire Line
	2900 6050 2900 5450
Wire Wire Line
	2250 6900 2250 6650
Wire Wire Line
	2150 6900 2250 6900
Wire Wire Line
	2150 6900 2150 6650
$Comp
L GND #PWR014
U 1 1 58FBD230
P 2200 7000
F 0 "#PWR014" H 2200 7000 30  0001 C CNN
F 1 "GND" H 2200 6930 30  0001 C CNN
F 2 "" H 2200 7000 60  0001 C CNN
F 3 "" H 2200 7000 60  0001 C CNN
	1    2200 7000
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 7000 2200 6900
Connection ~ 2200 6900
$Comp
L +5V #PWR015
U 1 1 58FBD2DA
P 2900 5450
F 0 "#PWR015" H 2900 5540 20  0001 C CNN
F 1 "+5V" H 2900 5540 30  0000 C CNN
F 2 "" H 2900 5450 60  0001 C CNN
F 3 "" H 2900 5450 60  0001 C CNN
	1    2900 5450
	1    0    0    -1  
$EndComp
Wire Wire Line
	2550 6250 3000 6250
Text Label 3400 6250 0    60   ~ 0
USB_D_P
Text Label 3400 6450 0    60   ~ 0
USB_D_N
NoConn ~ 2550 6450
$Comp
L R R3
U 1 1 58FBD548
P 3150 6250
F 0 "R3" V 3230 6250 50  0000 C CNN
F 1 "22R" V 3150 6250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3080 6250 50  0001 C CNN
F 3 "" H 3150 6250 50  0001 C CNN
	1    3150 6250
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 58FBD5A1
P 3150 6450
F 0 "R4" V 3230 6450 50  0000 C CNN
F 1 "22R" V 3150 6450 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 3080 6450 50  0001 C CNN
F 3 "" H 3150 6450 50  0001 C CNN
	1    3150 6450
	0    1    1    0   
$EndComp
Wire Wire Line
	3300 6250 3750 6250
Wire Wire Line
	3300 6450 3750 6450
Wire Wire Line
	3000 6450 3000 6350
Wire Wire Line
	3000 6350 2550 6350
$Comp
L MIC5504 U2
U 1 1 58FBDABD
P 5950 5700
F 0 "U2" H 6250 6100 60  0000 C CNN
F 1 "MIC5504-3.3" H 5850 5950 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 5850 5700 60  0001 C CNN
F 3 "" H 5850 5700 60  0001 C CNN
	1    5950 5700
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR016
U 1 1 58FBDB92
P 5000 5300
F 0 "#PWR016" H 5000 5390 20  0001 C CNN
F 1 "+5V" H 5000 5390 30  0000 C CNN
F 2 "" H 5000 5300 60  0001 C CNN
F 3 "" H 5000 5300 60  0001 C CNN
	1    5000 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	5000 5300 5000 5600
Wire Wire Line
	5000 5600 5400 5600
$Comp
L C C2
U 1 1 58FBDC44
P 5000 5750
F 0 "C2" H 5025 5850 50  0000 L CNN
F 1 "1uF" H 5025 5650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 5038 5600 50  0001 C CNN
F 3 "" H 5000 5750 50  0001 C CNN
	1    5000 5750
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 58FBDC9E
P 5950 6500
F 0 "#PWR017" H 5950 6500 30  0001 C CNN
F 1 "GND" H 5950 6430 30  0001 C CNN
F 2 "" H 5950 6500 60  0001 C CNN
F 3 "" H 5950 6500 60  0001 C CNN
	1    5950 6500
	1    0    0    -1  
$EndComp
Wire Wire Line
	5950 6500 5950 6150
Wire Wire Line
	5000 6150 6700 6150
Wire Wire Line
	5000 6150 5000 5900
$Comp
L C C5
U 1 1 58FBDE31
P 6700 5750
F 0 "C5" H 6725 5850 50  0000 L CNN
F 1 "1uF" H 6725 5650 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 6738 5600 50  0001 C CNN
F 3 "" H 6700 5750 50  0001 C CNN
	1    6700 5750
	1    0    0    -1  
$EndComp
Wire Wire Line
	6450 5600 7000 5600
Wire Wire Line
	7000 5600 7000 5300
Connection ~ 6700 5600
$Comp
L +3.3V #PWR018
U 1 1 58FBDFD0
P 7000 5300
F 0 "#PWR018" H 7000 5260 30  0001 C CNN
F 1 "+3.3V" H 7000 5410 30  0000 C CNN
F 2 "" H 7000 5300 60  0001 C CNN
F 3 "" H 7000 5300 60  0001 C CNN
	1    7000 5300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6700 6150 6700 5900
Connection ~ 5950 6150
$Comp
L CONN_01X02 J1
U 1 1 58FBE3F2
P 850 3850
F 0 "J1" H 850 4000 50  0000 C CNN
F 1 "DFU" V 950 3850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x02_Pitch1.27mm" H 850 3850 50  0001 C CNN
F 3 "" H 850 3850 50  0001 C CNN
	1    850  3850
	-1   0    0    1   
$EndComp
Wire Wire Line
	1050 3800 1500 3800
Connection ~ 1500 3800
Wire Wire Line
	1050 3900 1300 3900
Wire Wire Line
	1300 3900 1300 4150
Wire Wire Line
	1300 4150 1450 4150
$Comp
L CONN_01X04 J4
U 1 1 58FBEB50
P 4850 7000
F 0 "J4" H 4850 7250 50  0000 C CNN
F 1 "UART" V 4950 7000 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x04_Pitch2.54mm" H 4850 7000 50  0001 C CNN
F 3 "" H 4850 7000 50  0001 C CNN
	1    4850 7000
	1    0    0    -1  
$EndComp
$Comp
L +5V #PWR019
U 1 1 58FBECE0
P 4550 6550
F 0 "#PWR019" H 4550 6640 20  0001 C CNN
F 1 "+5V" H 4550 6640 30  0000 C CNN
F 2 "" H 4550 6550 60  0001 C CNN
F 3 "" H 4550 6550 60  0001 C CNN
	1    4550 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	4550 6550 4550 6850
Wire Wire Line
	4550 6850 4650 6850
Wire Wire Line
	4550 6950 4650 6950
Wire Wire Line
	4250 7050 4650 7050
Wire Wire Line
	4650 7150 4550 7150
Wire Wire Line
	4550 7150 4550 7400
$Comp
L GND #PWR020
U 1 1 58FBEF76
P 4550 7400
F 0 "#PWR020" H 4550 7400 30  0001 C CNN
F 1 "GND" H 4550 7330 30  0001 C CNN
F 2 "" H 4550 7400 60  0001 C CNN
F 3 "" H 4550 7400 60  0001 C CNN
	1    4550 7400
	1    0    0    -1  
$EndComp
$Comp
L R R12
U 1 1 58FBF121
P 4400 6950
F 0 "R12" V 4480 6950 50  0000 C CNN
F 1 "220R" V 4400 6950 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4330 6950 50  0001 C CNN
F 3 "" H 4400 6950 50  0001 C CNN
	1    4400 6950
	0    -1   -1   0   
$EndComp
$Comp
L R R11
U 1 1 58FBF3A1
P 4100 7050
F 0 "R11" V 4180 7050 50  0000 C CNN
F 1 "220R" V 4100 7050 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 4030 7050 50  0001 C CNN
F 3 "" H 4100 7050 50  0001 C CNN
	1    4100 7050
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3500 6950 4250 6950
Wire Wire Line
	3950 7050 3500 7050
Text Label 3500 6950 0    60   ~ 0
USART_TX
Text Label 3500 7050 0    60   ~ 0
USART_RX
Wire Wire Line
	8000 2700 8600 2700
Text Label 10050 1300 0    60   ~ 0
I2C_INT
$Comp
L R R8
U 1 1 58FBA0D6
P 10600 850
F 0 "R8" V 10680 850 50  0000 C CNN
F 1 "4K7" V 10600 850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 10530 850 50  0001 C CNN
F 3 "" H 10600 850 50  0001 C CNN
	1    10600 850 
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 58FBA093
P 10400 850
F 0 "R7" V 10480 850 50  0000 C CNN
F 1 "4K7" V 10400 850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603" V 10330 850 50  0001 C CNN
F 3 "" H 10400 850 50  0001 C CNN
	1    10400 850 
	1    0    0    -1  
$EndComp
Text Label 10050 1200 0    60   ~ 0
I2C_RST
Text Label 8250 3300 0    60   ~ 0
I2C_RST
Wire Wire Line
	2200 3700 1650 3700
$Comp
L MIC5504 U4
U 1 1 58FC0FDE
P 7050 1250
F 0 "U4" H 7350 1650 60  0000 C CNN
F 1 "MIC5504-2.8" H 6950 1500 60  0000 C CNN
F 2 "TO_SOT_Packages_SMD:SOT-23-5" H 6950 1250 60  0001 C CNN
F 3 "" H 6950 1250 60  0001 C CNN
	1    7050 1250
	1    0    0    -1  
$EndComp
Wire Wire Line
	6100 850  6100 1150
Wire Wire Line
	6100 1150 6500 1150
$Comp
L GND #PWR021
U 1 1 58FC0FF2
P 7050 2050
F 0 "#PWR021" H 7050 2050 30  0001 C CNN
F 1 "GND" H 7050 1980 30  0001 C CNN
F 2 "" H 7050 2050 60  0001 C CNN
F 3 "" H 7050 2050 60  0001 C CNN
	1    7050 2050
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 2050 7050 1700
$Comp
L C C8
U 1 1 58FC0FFB
P 7800 1300
F 0 "C8" H 7825 1400 50  0000 L CNN
F 1 "1uF" H 7825 1200 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603" H 7838 1150 50  0001 C CNN
F 3 "" H 7800 1300 50  0001 C CNN
	1    7800 1300
	1    0    0    -1  
$EndComp
Wire Wire Line
	7550 1150 8100 1150
Wire Wire Line
	8100 1150 8100 850 
Connection ~ 7800 1150
$Comp
L +3.3V #PWR022
U 1 1 58FC1144
P 6100 850
F 0 "#PWR022" H 6100 810 30  0001 C CNN
F 1 "+3.3V" H 6100 960 30  0000 C CNN
F 2 "" H 6100 850 60  0001 C CNN
F 3 "" H 6100 850 60  0001 C CNN
	1    6100 850 
	1    0    0    -1  
$EndComp
$Comp
L +2.8V #PWR023
U 1 1 58FC11CC
P 8100 850
F 0 "#PWR023" H 8100 810 30  0001 C CNN
F 1 "+2.8V" H 8100 960 30  0000 C CNN
F 2 "" H 8100 850 60  0001 C CNN
F 3 "" H 8100 850 60  0001 C CNN
	1    8100 850 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7050 1700 7800 1700
Wire Wire Line
	7800 1700 7800 1450
Wire Wire Line
	8000 3300 8600 3300
Wire Wire Line
	8000 3400 8600 3400
Text Label 8250 3400 0    60   ~ 0
I2C_INT
Wire Wire Line
	8650 5200 8350 5200
Connection ~ 8350 5200
$EndSCHEMATC
