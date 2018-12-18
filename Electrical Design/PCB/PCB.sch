EESchema Schematic File Version 4
LIBS:PCB-cache
EELAYER 26 0
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
L MCU_Microchip_ATmega:ATxmega128A1U-AU U?
U 1 1 5C152EB5
P 5450 5550
F 0 "U?" H 5450 2864 50  0000 C CNN
F 1 "ATxmega128A1U-AU" H 5450 2773 50  0000 C CNN
F 2 "Package_QFP:TQFP-100_14x14mm_P0.5mm" H 5450 5550 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8385-8-and-16-bit-AVR-Microcontroller-ATxmega64A1U-ATxmega128A1U_datasheet.pdf" H 5450 5550 50  0001 C CNN
	1    5450 5550
	0    -1   -1   0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5C153140
P 1050 5550
F 0 "#PWR?" H 1050 5400 50  0001 C CNN
F 1 "+3.3V" H 1065 5723 50  0000 C CNN
F 2 "" H 1050 5550 50  0001 C CNN
F 3 "" H 1050 5550 50  0001 C CNN
	1    1050 5550
	0    -1   1    0   
$EndComp
$Comp
L pspice:INDUCTOR L?
U 1 1 5C15325D
P 1300 5550
F 0 "L?" H 1300 5765 50  0000 C CNN
F 1 "10μH" H 1300 5674 50  0000 C CNN
F 2 "" H 1300 5550 50  0001 C CNN
F 3 "~" H 1300 5550 50  0001 C CNN
	1    1300 5550
	-1   0    0    -1  
$EndComp
$Comp
L Device:C C?
U 1 1 5C153EFD
P 2750 5700
F 0 "C?" H 2865 5746 50  0000 L CNN
F 1 "100nF" H 2865 5655 50  0000 L CNN
F 2 "" H 2788 5550 50  0001 C CNN
F 3 "~" H 2750 5700 50  0001 C CNN
	1    2750 5700
	-1   0    0    1   
$EndComp
Connection ~ 2750 5550
Wire Wire Line
	2750 5550 2850 5550
$Comp
L Device:Ferrite_Bead_Small FB?
U 1 1 5C155C8A
P 1950 5450
F 0 "FB?" H 1850 5496 50  0000 R CNN
F 1 "Ferrite_Bead_Small" H 1850 5405 50  0000 R CNN
F 2 "" V 1880 5450 50  0001 C CNN
F 3 "~" H 1950 5450 50  0001 C CNN
	1    1950 5450
	0    -1   1    0   
$EndComp
$Comp
L Device:C C?
U 1 1 5C15687D
P 2750 5300
F 0 "C?" V 2498 5300 50  0000 C CNN
F 1 "100nF" V 2589 5300 50  0000 C CNN
F 2 "" H 2788 5150 50  0001 C CNN
F 3 "~" H 2750 5300 50  0001 C CNN
	1    2750 5300
	1    0    0    1   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C15837A
P 2750 5850
F 0 "#PWR?" H 2750 5600 50  0001 C CNN
F 1 "GND" H 2755 5677 50  0000 C CNN
F 2 "" H 2750 5850 50  0001 C CNN
F 3 "" H 2750 5850 50  0001 C CNN
	1    2750 5850
	0    -1   -1   0   
$EndComp
$Comp
L Device:CP1 C?
U 1 1 5C158FD7
P 2350 5300
F 0 "C?" V 2098 5300 50  0000 C CNN
F 1 "10μF" V 2189 5300 50  0000 C CNN
F 2 "" H 2350 5300 50  0001 C CNN
F 3 "~" H 2350 5300 50  0001 C CNN
	1    2350 5300
	1    0    0    1   
$EndComp
$Comp
L Device:CP1 C?
U 1 1 5C15911D
P 1750 5700
F 0 "C?" V 1498 5700 50  0000 C CNN
F 1 "10μF" V 1589 5700 50  0000 C CNN
F 2 "" H 1750 5700 50  0001 C CNN
F 3 "~" H 1750 5700 50  0001 C CNN
	1    1750 5700
	1    0    0    -1  
$EndComp
Connection ~ 2750 5450
Wire Wire Line
	2750 5450 2850 5450
$Comp
L power:GND #PWR?
U 1 1 5C159D44
P 2750 5150
F 0 "#PWR?" H 2750 4900 50  0001 C CNN
F 1 "GND" H 2755 4977 50  0000 C CNN
F 2 "" H 2750 5150 50  0001 C CNN
F 3 "" H 2750 5150 50  0001 C CNN
	1    2750 5150
	0    -1   -1   0   
$EndComp
Connection ~ 2350 5450
Wire Wire Line
	2350 5450 2750 5450
Wire Wire Line
	1550 5550 1750 5550
Connection ~ 1850 5550
Wire Wire Line
	1850 5550 2750 5550
Wire Wire Line
	1850 5550 1850 5450
Wire Wire Line
	2050 5450 2350 5450
Wire Wire Line
	2350 5150 2750 5150
Connection ~ 2750 5150
Connection ~ 1750 5550
Wire Wire Line
	1750 5550 1850 5550
Wire Wire Line
	1750 5850 2750 5850
Connection ~ 2750 5850
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J?
U 1 1 5C15DCBD
P 3200 7050
F 0 "J?" H 3250 7367 50  0000 C CNN
F 1 "Header 3x2" H 3250 7276 50  0000 C CNN
F 2 "" H 3200 7050 50  0001 C CNN
F 3 "~" H 3200 7050 50  0001 C CNN
	1    3200 7050
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C15E1D2
P 3300 6750
F 0 "#PWR?" H 3300 6500 50  0001 C CNN
F 1 "GND" V 3305 6622 50  0000 R CNN
F 2 "" H 3300 6750 50  0001 C CNN
F 3 "" H 3300 6750 50  0001 C CNN
	1    3300 6750
	1    0    0    1   
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 5C15E3E2
P 3000 6750
F 0 "#PWR?" H 3000 6600 50  0001 C CNN
F 1 "VDD" H 3017 6923 50  0000 C CNN
F 2 "" H 3000 6750 50  0001 C CNN
F 3 "" H 3000 6750 50  0001 C CNN
	1    3000 6750
	0    -1   -1   0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 5C15ED03
P 3150 6550
F 0 "R?" V 2954 6550 50  0000 C CNN
F 1 "10kΩ" V 3045 6550 50  0000 C CNN
F 2 "" H 3150 6550 50  0001 C CNN
F 3 "~" H 3150 6550 50  0001 C CNN
	1    3150 6550
	1    0    0    -1  
$EndComp
Wire Wire Line
	3000 6750 3100 6750
Wire Wire Line
	3100 6750 3100 6650
Wire Wire Line
	3100 6650 3150 6650
Connection ~ 3100 6750
Wire Wire Line
	3150 6450 3150 6400
Wire Wire Line
	3100 7250 3100 7400
Wire Wire Line
	3100 7400 3400 7400
Wire Wire Line
	3400 7400 3400 6400
Wire Wire Line
	3400 6400 3250 6400
Wire Wire Line
	3250 6400 3250 6350
Wire Wire Line
	3300 7250 3300 7350
Wire Wire Line
	3300 7350 2750 7350
Wire Wire Line
	2750 6400 3150 6400
Wire Wire Line
	2750 7350 2750 6400
Connection ~ 3150 6400
Wire Wire Line
	3150 6400 3150 6350
$Comp
L Custom_Components:MS5607 U?
U 1 1 5C1576F0
P 1450 750
F 0 "U?" H 2050 265 50  0000 C CNN
F 1 "MS5607" H 2050 174 50  0000 C CNN
F 2 "" H 2050 50  50  0001 C CNN
F 3 "" H 2050 50  50  0001 C CNN
	1    1450 750 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5C158279
P 2600 1500
F 0 "#PWR?" H 2600 1350 50  0001 C CNN
F 1 "+3.3V" H 2615 1673 50  0000 C CNN
F 2 "" H 2600 1500 50  0001 C CNN
F 3 "" H 2600 1500 50  0001 C CNN
	1    2600 1500
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C15E020
P 2700 1650
F 0 "C?" H 2792 1696 50  0000 L CNN
F 1 "100nF" H 2792 1605 50  0000 L CNN
F 2 "" H 2700 1650 50  0001 C CNN
F 3 "~" H 2700 1650 50  0001 C CNN
	1    2700 1650
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 1500 2600 1550
Wire Wire Line
	2600 1550 2500 1550
Wire Wire Line
	2600 1550 2700 1550
Connection ~ 2600 1550
Wire Wire Line
	2700 1750 2550 1750
Wire Wire Line
	2500 1650 2550 1650
Wire Wire Line
	2550 1650 2550 1750
Connection ~ 2550 1750
Wire Wire Line
	2550 1750 2500 1750
Wire Wire Line
	2700 1750 2700 1800
Connection ~ 2700 1750
$Comp
L power:GND #PWR?
U 1 1 5C1606B9
P 2700 1800
F 0 "#PWR?" H 2700 1550 50  0001 C CNN
F 1 "GND" H 2705 1627 50  0000 C CNN
F 2 "" H 2700 1800 50  0001 C CNN
F 3 "" H 2700 1800 50  0001 C CNN
	1    2700 1800
	1    0    0    -1  
$EndComp
$Comp
L Custom_Components:AP3419 U?
U 1 1 5C162E1F
P 1350 2950
F 0 "U?" H 1425 3075 50  0000 C CNN
F 1 "AP3419" H 1425 2984 50  0000 C CNN
F 2 "" H 1450 2950 50  0001 C CNN
F 3 "" H 1450 2950 50  0001 C CNN
	1    1350 2950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C1639C2
P 1150 3150
F 0 "#PWR?" H 1150 2900 50  0001 C CNN
F 1 "GND" V 1155 3022 50  0000 R CNN
F 2 "" H 1150 3150 50  0001 C CNN
F 3 "" H 1150 3150 50  0001 C CNN
	1    1150 3150
	0    1    1    0   
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5C163C3D
P 1700 3450
F 0 "#PWR?" H 1700 3300 50  0001 C CNN
F 1 "+3.3V" H 1715 3623 50  0000 C CNN
F 2 "" H 1700 3450 50  0001 C CNN
F 3 "" H 1700 3450 50  0001 C CNN
	1    1700 3450
	1    0    0    1   
$EndComp
Wire Wire Line
	1700 3450 1700 3350
Wire Wire Line
	1700 3350 1750 3350
Connection ~ 1700 3350
Wire Wire Line
	1700 3350 1700 3250
$Comp
L Device:C_Small C?
U 1 1 5C164C8F
P 1850 3350
F 0 "C?" V 1621 3350 50  0000 C CNN
F 1 "22µF" V 1712 3350 50  0000 C CNN
F 2 "" H 1850 3350 50  0001 C CNN
F 3 "~" H 1850 3350 50  0001 C CNN
	1    1850 3350
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C16527F
P 1950 3350
F 0 "#PWR?" H 1950 3100 50  0001 C CNN
F 1 "GND" V 1955 3222 50  0000 R CNN
F 2 "" H 1950 3350 50  0001 C CNN
F 3 "" H 1950 3350 50  0001 C CNN
	1    1950 3350
	0    -1   1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 5C166029
P 2400 3050
F 0 "R?" V 2504 3050 50  0000 C CNN
F 1 "150kΩ" V 2595 3050 50  0000 C CNN
F 2 "" H 2400 3050 50  0001 C CNN
F 3 "~" H 2400 3050 50  0001 C CNN
	1    2400 3050
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C1665BD
P 2500 3050
F 0 "#PWR?" H 2500 2800 50  0001 C CNN
F 1 "GND" V 2505 2922 50  0000 R CNN
F 2 "" H 2500 3050 50  0001 C CNN
F 3 "" H 2500 3050 50  0001 C CNN
	1    2500 3050
	0    -1   1    0   
$EndComp
$Comp
L Device:L_Small L?
U 1 1 5C16A4F0
P 1000 3250
F 0 "L?" V 915 3250 50  0000 C CNN
F 1 "2.2µH" V 824 3250 50  0000 C CNN
F 2 "" H 1000 3250 50  0001 C CNN
F 3 "~" H 1000 3250 50  0001 C CNN
	1    1000 3250
	0    1    -1   0   
$EndComp
Wire Wire Line
	1150 3250 1100 3250
Wire Wire Line
	900  3250 800  3250
Wire Wire Line
	800  3250 800  2750
Wire Wire Line
	800  2750 1800 2750
Wire Wire Line
	1800 2750 1800 2800
Connection ~ 1800 2750
$Comp
L Device:C_Small C?
U 1 1 5C170630
P 1800 2900
F 0 "C?" H 1892 2946 50  0000 L CNN
F 1 "22pF" H 1892 2855 50  0000 L CNN
F 2 "" H 1800 2900 50  0001 C CNN
F 3 "~" H 1800 2900 50  0001 C CNN
	1    1800 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1700 3050 1800 3050
Wire Wire Line
	1800 3050 1800 3000
Connection ~ 1800 3050
Wire Wire Line
	1800 2750 2200 2750
Wire Wire Line
	1800 3050 2200 3050
$Comp
L Device:R_Small R?
U 1 1 5C174703
P 2200 2900
F 0 "R?" H 2259 2946 50  0000 L CNN
F 1 "300kΩ" H 2259 2855 50  0000 L CNN
F 2 "" H 2200 2900 50  0001 C CNN
F 3 "~" H 2200 2900 50  0001 C CNN
	1    2200 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 2750 2200 2800
Wire Wire Line
	2200 3000 2200 3050
Wire Wire Line
	2300 3050 2200 3050
Connection ~ 2200 3050
Wire Wire Line
	2200 2750 2300 2750
Connection ~ 2200 2750
$Comp
L Device:C_Small C?
U 1 1 5C178C15
P 2400 2750
F 0 "C?" V 2171 2750 50  0000 C CNN
F 1 "22µF" V 2262 2750 50  0000 C CNN
F 2 "" H 2400 2750 50  0001 C CNN
F 3 "~" H 2400 2750 50  0001 C CNN
	1    2400 2750
	0    1    1    0   
$EndComp
Wire Wire Line
	2500 2750 2600 2750
$Comp
L Device:C_Small C?
U 1 1 5C179E29
P 2700 2750
F 0 "C?" V 2471 2750 50  0000 C CNN
F 1 "22µF" V 2562 2750 50  0000 C CNN
F 2 "" H 2700 2750 50  0001 C CNN
F 3 "~" H 2700 2750 50  0001 C CNN
	1    2700 2750
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C17A145
P 2800 2750
F 0 "#PWR?" H 2800 2500 50  0001 C CNN
F 1 "GND" V 2805 2622 50  0000 R CNN
F 2 "" H 2800 2750 50  0001 C CNN
F 3 "" H 2800 2750 50  0001 C CNN
	1    2800 2750
	0    -1   -1   0   
$EndComp
$EndSCHEMATC