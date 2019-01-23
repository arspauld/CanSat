EESchema Schematic File Version 4
LIBS:PCB-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "SkyFire PCB"
Date "January 22, 2019"
Rev "v01"
Comp "UAH Space Hardware Club"
Comment1 "Zach Tyler"
Comment2 "Alex Spaulding"
Comment3 "Noah Schwalb"
Comment4 "Electrical Team: "
$EndDescr
$Comp
L MCU_Microchip_ATmega:ATxmega128A4U-AU U?
U 1 1 5C4816FD
P 9650 4650
F 0 "U?" H 9900 3100 50  0000 C CNN
F 1 "ATxmega128A4U-AU" H 9650 3000 50  0000 C CNN
F 2 "Package_QFP:TQFP-44_10x10mm_P0.8mm" H 9650 4650 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8387-8-and16-bit-AVR-Microcontroller-XMEGA-A4U_Datasheet.pdf" H 9650 4650 50  0001 C CNN
	1    9650 4650
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0101
U 1 1 5C4817C1
P 6700 5750
F 0 "#PWR0101" H 6700 5600 50  0001 C CNN
F 1 "+3.3V" H 6715 5923 50  0000 C CNN
F 2 "" H 6700 5750 50  0001 C CNN
F 3 "" H 6700 5750 50  0001 C CNN
	1    6700 5750
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5C481A0E
P 9650 6050
F 0 "#PWR0102" H 9650 5800 50  0001 C CNN
F 1 "GND" H 9655 5877 50  0000 C CNN
F 2 "" H 9650 6050 50  0001 C CNN
F 3 "" H 9650 6050 50  0001 C CNN
	1    9650 6050
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 5C481BB8
P 7100 5850
F 0 "C?" V 7050 5550 50  0000 C CNN
F 1 "10uF" V 7150 5550 50  0000 C CNN
F 2 "" H 7100 5850 50  0001 C CNN
F 3 "~" H 7100 5850 50  0001 C CNN
	1    7100 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 5C481D4D
P 7500 5550
F 0 "C?" V 7550 5300 50  0000 C CNN
F 1 "10uF" V 7450 5300 50  0000 C CNN
F 2 "" H 7500 5550 50  0001 C CNN
F 3 "~" H 7500 5550 50  0001 C CNN
	1    7500 5550
	-1   0    0    1   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C481E8E
P 7700 5550
F 0 "C?" V 7650 5800 50  0000 C CNN
F 1 "100nF" V 7750 5800 50  0000 C CNN
F 2 "" H 7700 5550 50  0001 C CNN
F 3 "~" H 7700 5550 50  0001 C CNN
	1    7700 5550
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C481F76
P 7300 5850
F 0 "C?" V 7250 5550 50  0000 C CNN
F 1 "100nF" V 7350 5550 50  0000 C CNN
F 2 "" H 7300 5850 50  0001 C CNN
F 3 "~" H 7300 5850 50  0001 C CNN
	1    7300 5850
	1    0    0    -1  
$EndComp
$Comp
L Device:L_Small L?
U 1 1 5C4820F6
P 6900 5750
F 0 "L?" H 6948 5796 50  0000 L CNN
F 1 "10uH" H 6948 5705 50  0000 L CNN
F 2 "" H 6900 5750 50  0001 C CNN
F 3 "~" H 6900 5750 50  0001 C CNN
	1    6900 5750
	0    -1   -1   0   
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB?
U 1 1 5C48226D
P 7300 5650
F 0 "FB?" H 7400 5696 50  0000 L CNN
F 1 "220@100MHz" H 7400 5605 50  0000 L CNN
F 2 "" V 7230 5650 50  0001 C CNN
F 3 "~" H 7300 5650 50  0001 C CNN
	1    7300 5650
	0    -1   -1   0   
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5C4834EF
P 7450 5950
F 0 "#PWR0103" H 7450 5700 50  0001 C CNN
F 1 "GND" H 7455 5777 50  0000 C CNN
F 2 "" H 7450 5950 50  0001 C CNN
F 3 "" H 7450 5950 50  0001 C CNN
	1    7450 5950
	0    -1   -1   0   
$EndComp
Wire Wire Line
	6700 5750 6800 5750
Wire Wire Line
	7000 5750 7100 5750
Wire Wire Line
	7100 5950 7300 5950
Connection ~ 7300 5950
Wire Wire Line
	7300 5950 7450 5950
Wire Wire Line
	7100 5750 7200 5750
Connection ~ 7100 5750
Connection ~ 7200 5750
Wire Wire Line
	7200 5750 7300 5750
Wire Wire Line
	7400 5650 7500 5650
Connection ~ 7500 5650
Wire Wire Line
	7500 5650 7700 5650
Wire Wire Line
	7500 5450 7700 5450
$Comp
L power:GND #PWR0104
U 1 1 5C484B92
P 7800 5450
F 0 "#PWR0104" H 7800 5200 50  0001 C CNN
F 1 "GND" H 7950 5400 50  0000 C CNN
F 2 "" H 7800 5450 50  0001 C CNN
F 3 "" H 7800 5450 50  0001 C CNN
	1    7800 5450
	0    -1   -1   0   
$EndComp
Wire Wire Line
	7700 5450 7800 5450
Connection ~ 7700 5450
Wire Wire Line
	7200 5750 7200 5650
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J?
U 1 1 5C487514
P 1150 4250
F 0 "J?" H 1200 4550 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 1200 4450 50  0000 C CNN
F 2 "" H 1150 4250 50  0001 C CNN
F 3 "~" H 1150 4250 50  0001 C CNN
	1    1150 4250
	1    0    0    -1  
$EndComp
NoConn ~ 1450 4250
NoConn ~ 950  4250
Text Label 7400 5750 0    50   ~ 0
MCU_VCC
Wire Wire Line
	7300 5750 7400 5750
Connection ~ 7300 5750
Text Label 9650 3250 1    50   ~ 0
MCU_VCC
Text Label 9750 3250 1    50   ~ 0
MCU_AVCC
Text Label 7700 5650 0    50   ~ 0
MCU_AVCC
Text Label 950  4150 2    50   ~ 0
PDI_DATA
Text Label 8950 3650 2    50   ~ 0
PDI_DATA
$Comp
L Device:R_Small R?
U 1 1 5C48AF52
P 1700 4350
F 0 "R?" H 1759 4396 50  0000 L CNN
F 1 "10k" H 1759 4305 50  0000 L CNN
F 2 "" H 1700 4350 50  0001 C CNN
F 3 "~" H 1700 4350 50  0001 C CNN
	1    1700 4350
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0105
U 1 1 5C48B786
P 1450 4350
F 0 "#PWR0105" H 1450 4100 50  0001 C CNN
F 1 "GND" H 1455 4177 50  0000 C CNN
F 2 "" H 1450 4350 50  0001 C CNN
F 3 "" H 1450 4350 50  0001 C CNN
	1    1450 4350
	1    0    0    -1  
$EndComp
Wire Wire Line
	1450 4150 1700 4150
Wire Wire Line
	1700 4150 1700 4000
Connection ~ 1700 4150
Wire Wire Line
	1700 4150 1700 4250
Wire Wire Line
	950  4350 850  4350
Wire Wire Line
	850  4350 850  4550
Wire Wire Line
	850  4550 1700 4550
Wire Wire Line
	1700 4550 1700 4450
Text Label 8950 3550 2    50   ~ 0
PDI_CLK
Text Label 1700 4600 3    50   ~ 0
PDI_CLK
$Comp
L MS560702BA03-50:MS560702BA03-50 U?
U 1 1 5C48EC0D
P 4750 5750
F 0 "U?" H 4750 6217 50  0000 C CNN
F 1 "MS560702BA03-50" H 4750 6126 50  0000 C CNN
F 2 "SON125P300X500X100-8N" H 4750 5750 50  0001 L BNN
F 3 "https://www.te.com/usa-en/product-MS560702BA03-50.html?te_bu=Cor&te_type=disp&te_campaign=seda_glo_cor-seda-global-disp-prtnr-fy19-seda-model-bom-cta_sma-317_1&elqCampaignId=32493" H 4750 5750 50  0001 L BNN
F 4 "MS560702BA03-50" H 4750 5750 50  0001 L BNN "Field4"
F 5 "MS560702BA03-50" H 4750 5750 50  0001 L BNN "Field5"
F 6 "Unavailable" H 4750 5750 50  0001 L BNN "Field6"
F 7 "Barometric Pressure Sensor, With Stainless Steel Cap" H 4750 5750 50  0001 L BNN "Field7"
F 8 "None" H 4750 5750 50  0001 L BNN "Field8"
F 9 "SMD-8 Measurement Specialties" H 4750 5750 50  0001 L BNN "Field9"
F 10 "TE Connectivity" H 4750 5750 50  0001 L BNN "Field10"
	1    4750 5750
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C48F1E8
P 5750 5750
F 0 "C?" H 5842 5796 50  0000 L CNN
F 1 "100nF" H 5842 5705 50  0000 L CNN
F 2 "" H 5750 5750 50  0001 C CNN
F 3 "~" H 5750 5750 50  0001 C CNN
	1    5750 5750
	1    0    0    -1  
$EndComp
Text Label 5350 5750 0    50   ~ 0
MISO
Text Label 4150 5950 2    50   ~ 0
MOSI
Text Label 4150 5750 2    50   ~ 0
SCLK
Wire Wire Line
	5350 5550 5600 5550
Wire Wire Line
	5600 5550 5600 5450
Wire Wire Line
	5600 5550 5750 5550
Wire Wire Line
	5750 5550 5750 5650
Connection ~ 5600 5550
Wire Wire Line
	5750 5850 5750 5950
Wire Wire Line
	5750 5950 5450 5950
Wire Wire Line
	4150 5650 3900 5650
Wire Wire Line
	3900 5650 3900 6200
Wire Wire Line
	3900 6200 5450 6200
Wire Wire Line
	5450 6200 5450 5950
Connection ~ 5450 5950
Wire Wire Line
	5450 5950 5350 5950
$Comp
L power:GND #PWR0108
U 1 1 5C49115C
P 5750 5950
F 0 "#PWR0108" H 5750 5700 50  0001 C CNN
F 1 "GND" H 5755 5777 50  0000 C CNN
F 2 "" H 5750 5950 50  0001 C CNN
F 3 "" H 5750 5950 50  0001 C CNN
	1    5750 5950
	1    0    0    -1  
$EndComp
Connection ~ 5750 5950
Text Label 4150 5550 2    50   ~ 0
SS_5607
$Comp
L BNO080:BNO080 U?
U 1 1 5C492D9E
P 1950 2100
F 0 "U?" H 1900 3067 50  0000 C CNN
F 1 "BNO080" H 1900 2976 50  0000 C CNN
F 2 "LGA-28" H 1950 2100 50  0001 L BNN
F 3 "Imu Accel/Gyro/Mag I2c 32bit" H 1950 2100 50  0001 L BNN
F 4 "Hillcrest Laboratories," H 1950 2100 50  0001 L BNN "Field4"
F 5 "https://www.digikey.com/product-detail/en/hillcrest-laboratories-inc/BNO080/1888-1000-1-ND/7917169?utm_source=snapeda&utm_medium=aggregator&utm_campaign=symbol" H 1950 2100 50  0001 L BNN "Field5"
F 6 "BNO080" H 1950 2100 50  0001 L BNN "Field6"
F 7 "TFLGA-28 Hillcrest Laboratories" H 1950 2100 50  0001 L BNN "Field7"
F 8 "1888-1000-1-ND" H 1950 2100 50  0001 L BNN "Field8"
	1    1950 2100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0109
U 1 1 5C4934FB
P 1050 2900
F 0 "#PWR0109" H 1050 2650 50  0001 C CNN
F 1 "GND" H 1055 2727 50  0000 C CNN
F 2 "" H 1050 2900 50  0001 C CNN
F 3 "" H 1050 2900 50  0001 C CNN
	1    1050 2900
	1    0    0    -1  
$EndComp
Wire Wire Line
	1150 2700 1050 2700
Wire Wire Line
	1050 2700 1050 2800
Wire Wire Line
	1150 2800 1050 2800
Connection ~ 1050 2800
Wire Wire Line
	1050 2800 1050 2900
$Comp
L Device:CP1_Small C?
U 1 1 5C494687
P 1400 900
F 0 "C?" H 1350 1200 50  0000 L CNN
F 1 "0.1uF" H 1300 1100 50  0000 L CNN
F 2 "" H 1400 900 50  0001 C CNN
F 3 "~" H 1400 900 50  0001 C CNN
	1    1400 900 
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 5C494873
P 1650 900
F 0 "C?" H 1600 1200 50  0000 L CNN
F 1 "0.1uF" H 1550 1100 50  0000 L CNN
F 2 "" H 1650 900 50  0001 C CNN
F 3 "~" H 1650 900 50  0001 C CNN
	1    1650 900 
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 5C4949E1
P 2700 2950
F 0 "C?" H 2791 2996 50  0000 L CNN
F 1 "0.1uF" H 2791 2905 50  0000 L CNN
F 2 "" H 2700 2950 50  0001 C CNN
F 3 "~" H 2700 2950 50  0001 C CNN
	1    2700 2950
	1    0    0    -1  
$EndComp
Text Label 1150 1700 2    50   ~ 0
MOSI
Text Label 1150 1800 2    50   ~ 0
MISO
Text Label 1150 1900 2    50   ~ 0
SCLK
$Comp
L power:GND #PWR0110
U 1 1 5C495B95
P 1400 1100
F 0 "#PWR0110" H 1400 850 50  0001 C CNN
F 1 "GND" H 1405 927 50  0000 C CNN
F 2 "" H 1400 1100 50  0001 C CNN
F 3 "" H 1400 1100 50  0001 C CNN
	1    1400 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0111
U 1 1 5C495DC2
P 1050 700
F 0 "#PWR0111" H 1050 550 50  0001 C CNN
F 1 "+3.3V" H 1065 873 50  0000 C CNN
F 2 "" H 1050 700 50  0001 C CNN
F 3 "" H 1050 700 50  0001 C CNN
	1    1050 700 
	1    0    0    -1  
$EndComp
Wire Wire Line
	1400 750  1400 800 
Wire Wire Line
	1400 750  1650 750 
Wire Wire Line
	1650 750  1650 800 
Connection ~ 1400 750 
Wire Wire Line
	1650 1000 1650 1050
Wire Wire Line
	1650 1050 1400 1050
Wire Wire Line
	1400 1050 1400 1100
Wire Wire Line
	1400 1000 1400 1050
Connection ~ 1400 1050
Wire Wire Line
	1050 750  1050 1400
Wire Wire Line
	1050 1500 1150 1500
Wire Wire Line
	1050 1400 1150 1400
Connection ~ 1050 1400
Wire Wire Line
	1050 1400 1050 1500
Connection ~ 1050 1500
$Comp
L power:GND #PWR0112
U 1 1 5C49E493
P 2700 3100
F 0 "#PWR0112" H 2700 2850 50  0001 C CNN
F 1 "GND" H 2705 2927 50  0000 C CNN
F 2 "" H 2700 3100 50  0001 C CNN
F 3 "" H 2700 3100 50  0001 C CNN
	1    2700 3100
	1    0    0    -1  
$EndComp
Wire Wire Line
	2650 2800 2700 2800
Wire Wire Line
	2700 2800 2700 2850
Wire Wire Line
	2700 3050 2700 3100
Text Label 1150 2400 2    50   ~ 0
BNO_WAKE
Text Label 1150 1600 2    50   ~ 0
BNO_CLOCKSEL0
Wire Wire Line
	500  1500 500  2500
Wire Wire Line
	500  1500 1050 1500
Wire Wire Line
	500  2500 1150 2500
Text Label 1150 2000 2    50   ~ 0
HOST_INTN
Text Label 1150 2100 2    50   ~ 0
SS_BNO080
Text Label 1150 2200 2    50   ~ 0
BNO_BOOTN
Text Label 2650 1400 0    50   ~ 0
BNO_NRST
$Comp
L power:GND #PWR0113
U 1 1 5C4AB245
P 3350 2900
F 0 "#PWR0113" H 3350 2650 50  0001 C CNN
F 1 "GND" H 3355 2727 50  0000 C CNN
F 2 "" H 3350 2900 50  0001 C CNN
F 3 "" H 3350 2900 50  0001 C CNN
	1    3350 2900
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 5C4AB644
P 2950 2700
F 0 "C?" H 3041 2746 50  0000 L CNN
F 1 "22pF" H 3041 2655 50  0000 L CNN
F 2 "" H 2950 2700 50  0001 C CNN
F 3 "~" H 2950 2700 50  0001 C CNN
	1    2950 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 5C4AB92E
P 3350 2700
F 0 "C?" H 3441 2746 50  0000 L CNN
F 1 "22pF" H 3441 2655 50  0000 L CNN
F 2 "" H 3350 2700 50  0001 C CNN
F 3 "~" H 3350 2700 50  0001 C CNN
	1    3350 2700
	1    0    0    -1  
$EndComp
$Comp
L Device:Crystal Y?
U 1 1 5C4ABCD3
P 3150 2400
F 0 "Y?" H 3150 2668 50  0000 C CNN
F 1 "?" H 3150 2577 50  0000 C CNN
F 2 "" H 3150 2400 50  0001 C CNN
F 3 "~" H 3150 2400 50  0001 C CNN
	1    3150 2400
	1    0    0    -1  
$EndComp
Wire Wire Line
	3300 2400 3350 2400
Wire Wire Line
	3350 2800 3350 2850
Connection ~ 2950 2400
Wire Wire Line
	2950 2400 3000 2400
Wire Wire Line
	2950 2800 2950 2850
Wire Wire Line
	2950 2850 3350 2850
Connection ~ 3350 2850
Wire Wire Line
	3350 2850 3350 2900
Wire Wire Line
	2650 2500 2750 2500
Wire Wire Line
	2750 2500 2750 2550
Wire Wire Line
	3350 2400 3350 2550
Wire Wire Line
	3350 2550 3350 2600
Connection ~ 3350 2550
Wire Wire Line
	2950 2400 2950 2600
Wire Wire Line
	2650 2400 2950 2400
Wire Wire Line
	2750 2550 3350 2550
$Comp
L Device:Thermistor_NTC TH?
U 1 1 5C4C8D21
P 1100 6950
F 0 "TH?" V 810 6950 50  0000 C CNN
F 1 "NTCLE100" V 901 6950 50  0000 C CNN
F 2 "" H 1100 7000 50  0001 C CNN
F 3 "~" H 1100 7000 50  0001 C CNN
	1    1100 6950
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 5C4C93EB
P 1650 6950
F 0 "R?" V 1454 6950 50  0000 C CNN
F 1 "R_Small" V 1545 6950 50  0000 C CNN
F 2 "" H 1650 6950 50  0001 C CNN
F 3 "~" H 1650 6950 50  0001 C CNN
	1    1650 6950
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR0115
U 1 1 5C4C9DEF
P 2000 6950
F 0 "#PWR0115" H 2000 6700 50  0001 C CNN
F 1 "GND" H 2005 6777 50  0000 C CNN
F 2 "" H 2000 6950 50  0001 C CNN
F 3 "" H 2000 6950 50  0001 C CNN
	1    2000 6950
	1    0    0    -1  
$EndComp
Wire Wire Line
	750  6950 950  6950
Wire Wire Line
	1750 6950 2000 6950
Wire Wire Line
	1400 6950 1400 7050
Connection ~ 1400 6950
Wire Wire Line
	1400 6950 1550 6950
Wire Wire Line
	1250 6950 1400 6950
Text Label 1400 7050 3    50   ~ 0
PA6-Therm
$Comp
L ADP3338AKCZ-5-R7:ADP3338AKCZ-5-R7 U?
U 1 1 5C4D6E8B
P 1750 5700
F 0 "U?" H 1750 6070 50  0000 C CNN
F 1 "ADP3338AKCZ-5-R7" H 1750 5979 50  0000 C CNN
F 2 "SOT230P700X180-4N" H 1750 5700 50  0001 L BNN
F 3 "Unavailable" H 1750 5700 50  0001 L BNN
F 4 "Analog Devices" H 1750 5700 50  0001 L BNN "Field4"
F 5 "ADP3338AKCZ-5-R7" H 1750 5700 50  0001 L BNN "Field5"
F 6 "LDO Regulator Pos 5V 1.6A 4-Pin_3+Tab_ SOT-223 T/R" H 1750 5700 50  0001 L BNN "Field6"
F 7 "SOT-223-3 Analog Devices" H 1750 5700 50  0001 L BNN "Field7"
F 8 "None" H 1750 5700 50  0001 L BNN "Field8"
	1    1750 5700
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0117
U 1 1 5C4D7F6B
P 900 5900
F 0 "#PWR0117" H 900 5650 50  0001 C CNN
F 1 "GND" V 905 5772 50  0000 R CNN
F 2 "" H 900 5900 50  0001 C CNN
F 3 "" H 900 5900 50  0001 C CNN
	1    900  5900
	0    1    1    0   
$EndComp
Wire Wire Line
	900  5900 1050 5900
Wire Wire Line
	2450 5800 2600 5800
Wire Wire Line
	2450 5900 2600 5900
Wire Wire Line
	2600 5800 2600 5850
Connection ~ 2600 5850
Wire Wire Line
	2600 5850 2600 5900
$Comp
L power:-5V #PWR0118
U 1 1 5C4E1B30
P 2900 5850
F 0 "#PWR0118" H 2900 5950 50  0001 C CNN
F 1 "-5V" V 2915 5978 50  0000 L CNN
F 2 "" H 2900 5850 50  0001 C CNN
F 3 "" H 2900 5850 50  0001 C CNN
	1    2900 5850
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C4E2003
P 2800 5950
F 0 "C?" H 2892 5996 50  0000 L CNN
F 1 "1uF" H 2892 5905 50  0000 L CNN
F 2 "" H 2800 5950 50  0001 C CNN
F 3 "~" H 2800 5950 50  0001 C CNN
	1    2800 5950
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C4E29CB
P 900 5600
F 0 "C?" H 992 5646 50  0000 L CNN
F 1 "1uF" H 992 5555 50  0000 L CNN
F 2 "" H 900 5600 50  0001 C CNN
F 3 "~" H 900 5600 50  0001 C CNN
	1    900  5600
	1    0    0    -1  
$EndComp
Wire Wire Line
	2600 5850 2800 5850
Connection ~ 2800 5850
Wire Wire Line
	2800 5850 2900 5850
$Comp
L power:GND #PWR0119
U 1 1 5C4E4FB9
P 2800 6050
F 0 "#PWR0119" H 2800 5800 50  0001 C CNN
F 1 "GND" H 2805 5877 50  0000 C CNN
F 2 "" H 2800 6050 50  0001 C CNN
F 3 "" H 2800 6050 50  0001 C CNN
	1    2800 6050
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0120
U 1 1 5C4E562C
P 900 5500
F 0 "#PWR0120" H 900 5250 50  0001 C CNN
F 1 "GND" H 905 5327 50  0000 C CNN
F 2 "" H 900 5500 50  0001 C CNN
F 3 "" H 900 5500 50  0001 C CNN
	1    900  5500
	-1   0    0    1   
$EndComp
Wire Wire Line
	700  5500 700  5700
Wire Wire Line
	700  5700 900  5700
Connection ~ 900  5700
Wire Wire Line
	900  5700 1050 5700
$Comp
L ADP3338AKCZ-5-R7:ADP3338AKCZ-5-R7 U?
U 1 1 5C4ECC23
P 3450 6900
F 0 "U?" H 3450 7270 50  0000 C CNN
F 1 "ADP3338AKCZ-3.3-R7" H 3450 7179 50  0000 C CNN
F 2 "SOT230P700X180-4N" H 3450 6900 50  0001 L BNN
F 3 "Unavailable" H 3450 6900 50  0001 L BNN
F 4 "Analog Devices" H 3450 6900 50  0001 L BNN "Field4"
F 5 "ADP3338AKCZ-5-R7" H 3450 6900 50  0001 L BNN "Field5"
F 6 "LDO Regulator Pos 5V 1.6A 4-Pin_3+Tab_ SOT-223 T/R" H 3450 6900 50  0001 L BNN "Field6"
F 7 "SOT-223-3 Analog Devices" H 3450 6900 50  0001 L BNN "Field7"
F 8 "None" H 3450 6900 50  0001 L BNN "Field8"
	1    3450 6900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0122
U 1 1 5C4ECC30
P 2600 7100
F 0 "#PWR0122" H 2600 6850 50  0001 C CNN
F 1 "GND" V 2605 6972 50  0000 R CNN
F 2 "" H 2600 7100 50  0001 C CNN
F 3 "" H 2600 7100 50  0001 C CNN
	1    2600 7100
	0    1    1    0   
$EndComp
Wire Wire Line
	2600 7100 2750 7100
Wire Wire Line
	4150 7000 4300 7000
Wire Wire Line
	4150 7100 4300 7100
Wire Wire Line
	4300 7000 4300 7050
Connection ~ 4300 7050
Wire Wire Line
	4300 7050 4300 7100
$Comp
L Device:C_Small C?
U 1 1 5C4ECC42
P 4500 7150
F 0 "C?" H 4592 7196 50  0000 L CNN
F 1 "1uF" H 4592 7105 50  0000 L CNN
F 2 "" H 4500 7150 50  0001 C CNN
F 3 "~" H 4500 7150 50  0001 C CNN
	1    4500 7150
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C4ECC49
P 2600 6800
F 0 "C?" H 2692 6846 50  0000 L CNN
F 1 "1uF" H 2692 6755 50  0000 L CNN
F 2 "" H 2600 6800 50  0001 C CNN
F 3 "~" H 2600 6800 50  0001 C CNN
	1    2600 6800
	1    0    0    -1  
$EndComp
Wire Wire Line
	4300 7050 4500 7050
Connection ~ 4500 7050
Wire Wire Line
	4500 7050 4600 7050
$Comp
L power:GND #PWR0123
U 1 1 5C4ECC53
P 4500 7250
F 0 "#PWR0123" H 4500 7000 50  0001 C CNN
F 1 "GND" H 4505 7077 50  0000 C CNN
F 2 "" H 4500 7250 50  0001 C CNN
F 3 "" H 4500 7250 50  0001 C CNN
	1    4500 7250
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0124
U 1 1 5C4ECC59
P 2600 6700
F 0 "#PWR0124" H 2600 6450 50  0001 C CNN
F 1 "GND" H 2605 6527 50  0000 C CNN
F 2 "" H 2600 6700 50  0001 C CNN
F 3 "" H 2600 6700 50  0001 C CNN
	1    2600 6700
	-1   0    0    1   
$EndComp
Wire Wire Line
	2400 6700 2400 6900
Wire Wire Line
	2400 6900 2600 6900
Connection ~ 2600 6900
Wire Wire Line
	2600 6900 2750 6900
$Comp
L power:-3V3 #PWR0125
U 1 1 5C4F216F
P 4600 7050
F 0 "#PWR0125" H 4600 7150 50  0001 C CNN
F 1 "-3V3" V 4615 7178 50  0000 L CNN
F 2 "" H 4600 7050 50  0001 C CNN
F 3 "" H 4600 7050 50  0001 C CNN
	1    4600 7050
	0    1    1    0   
$EndComp
Wire Wire Line
	1050 750  1400 750 
Wire Wire Line
	1050 750  1050 700 
Connection ~ 1050 750 
$Comp
L SS495A:SS495A U?
U 1 1 5C4FD36A
P 5450 6900
F 0 "U?" H 5750 7067 50  0000 C CNN
F 1 "SS495A" H 5750 6976 50  0000 C CNN
F 2 "SIP3-UA" H 5450 6900 50  0001 L BNN
F 3 "None" H 5450 6900 50  0001 L BNN
F 4 "Sensor; Sink; 4.5 to 10.5 VDC; 8.7 mA @5 VDC; 0.2 VDC _Typ._/0.4 VDC _Min._" H 5450 6900 50  0001 L BNN "Field4"
F 5 "Honeywell Sensing" H 5450 6900 50  0001 L BNN "Field5"
F 6 "SS495A" H 5450 6900 50  0001 L BNN "Field6"
F 7 "Unavailable" H 5450 6900 50  0001 L BNN "Field7"
F 8 "TO-92 Honeywell" H 5450 6900 50  0001 L BNN "Field8"
	1    5450 6900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0127
U 1 1 5C501DB3
P 5150 7250
F 0 "#PWR0127" H 5150 7000 50  0001 C CNN
F 1 "GND" H 5155 7077 50  0000 C CNN
F 2 "" H 5150 7250 50  0001 C CNN
F 3 "" H 5150 7250 50  0001 C CNN
	1    5150 7250
	1    0    0    -1  
$EndComp
Wire Wire Line
	5150 7250 5150 7200
Wire Wire Line
	5150 7200 5250 7200
Text Label 6250 7000 0    50   ~ 0
UNASSIGNED
$Comp
L Device:C_Small C?
U 1 1 5C5057CC
P 5150 6900
F 0 "C?" H 5242 6946 50  0000 L CNN
F 1 "1uF" H 5242 6855 50  0000 L CNN
F 2 "" H 5150 6900 50  0001 C CNN
F 3 "~" H 5150 6900 50  0001 C CNN
	1    5150 6900
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0128
U 1 1 5C50B237
P 5150 6750
F 0 "#PWR0128" H 5150 6500 50  0001 C CNN
F 1 "GND" H 5155 6577 50  0000 C CNN
F 2 "" H 5150 6750 50  0001 C CNN
F 3 "" H 5150 6750 50  0001 C CNN
	1    5150 6750
	-1   0    0    1   
$EndComp
Wire Wire Line
	5000 6950 5000 7000
Wire Wire Line
	5000 7000 5150 7000
Connection ~ 5150 7000
Wire Wire Line
	5150 7000 5250 7000
Wire Wire Line
	5150 6750 5150 6800
Text Notes 11000 3850 3    50   ~ 0
Ports A & B have access to the ADC
Text Label 10350 5350 0    50   ~ 0
SS_5607
Text Label 10350 5450 0    50   ~ 0
MOSI
Text Label 10350 5550 0    50   ~ 0
MISO
Text Label 10350 5650 0    50   ~ 0
SCLK
Wire Wire Line
	1700 4550 1700 4600
Connection ~ 1700 4550
Text Label 10350 5250 0    50   ~ 0
SS_BNO080
Text GLabel 8950 5150 0    50   Input ~ 0
RX
Text GLabel 8950 5250 0    50   Input ~ 0
TX
Text GLabel 8950 5350 0    50   Input ~ 0
TC
Text GLabel 8950 5450 0    50   Input ~ 0
TC
NoConn ~ 2650 1800
NoConn ~ 2650 1900
$Comp
L power:+3.3V #PWR?
U 1 1 5C491155
P 2200 4200
F 0 "#PWR?" H 2200 4050 50  0001 C CNN
F 1 "+3.3V" H 2215 4373 50  0000 C CNN
F 2 "" H 2200 4200 50  0001 C CNN
F 3 "" H 2200 4200 50  0001 C CNN
	1    2200 4200
	1    0    0    -1  
$EndComp
$Comp
L Device:R_Small R?
U 1 1 5C491A8E
P 2900 4300
F 0 "R?" V 2704 4300 50  0000 C CNN
F 1 "10" V 2795 4300 50  0000 C CNN
F 2 "" H 2900 4300 50  0001 C CNN
F 3 "~" H 2900 4300 50  0001 C CNN
	1    2900 4300
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 5C491D93
P 2500 4300
F 0 "R?" V 2304 4300 50  0000 C CNN
F 1 "5" V 2395 4300 50  0000 C CNN
F 2 "" H 2500 4300 50  0001 C CNN
F 3 "~" H 2500 4300 50  0001 C CNN
	1    2500 4300
	0    1    1    0   
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4998EE
P 3200 4300
F 0 "#PWR?" H 3200 4050 50  0001 C CNN
F 1 "GND" V 3205 4172 50  0000 R CNN
F 2 "" H 3200 4300 50  0001 C CNN
F 3 "" H 3200 4300 50  0001 C CNN
	1    3200 4300
	0    -1   -1   0   
$EndComp
Wire Wire Line
	2200 4200 2200 4300
Wire Wire Line
	2200 4300 2400 4300
Wire Wire Line
	2600 4300 2700 4300
Wire Wire Line
	3000 4300 3200 4300
Wire Wire Line
	2700 4300 2700 4100
Connection ~ 2700 4300
Wire Wire Line
	2700 4300 2800 4300
Text Notes 2200 4500 0    50   ~ 0
Voltage Divider (3.3 —> 2.2)
Text Label 10350 4250 0    50   ~ 0
PA7-Voltage
Text Label 2700 4100 1    50   ~ 0
PA7-Voltage
Text Notes 4250 5100 0    50   ~ 0
MS5607 Pressure Sensor
Text Notes 6950 4950 0    50   ~ 0
Power Interface
Text Notes 5400 6650 0    50   ~ 0
Hall Effect Sensor
Text Notes 3050 6450 0    50   ~ 0
6V —> 3.3V Regulator
Text Notes 1350 5200 0    50   ~ 0
6V —> 5V Regulator
Text Notes 950  3800 0    50   ~ 0
PDI Interface
Text Notes 1700 3050 0    50   ~ 0
BNO-080
Text Notes 1200 6600 0    50   ~ 0
Thermistor
$Comp
L power:+6V #PWR?
U 1 1 5C49358D
P 700 5500
F 0 "#PWR?" H 700 5350 50  0001 C CNN
F 1 "+6V" H 715 5673 50  0000 C CNN
F 2 "" H 700 5500 50  0001 C CNN
F 3 "" H 700 5500 50  0001 C CNN
	1    700  5500
	1    0    0    -1  
$EndComp
$Comp
L power:+6V #PWR?
U 1 1 5C49758F
P 2400 6700
F 0 "#PWR?" H 2400 6550 50  0001 C CNN
F 1 "+6V" H 2415 6873 50  0000 C CNN
F 2 "" H 2400 6700 50  0001 C CNN
F 3 "" H 2400 6700 50  0001 C CNN
	1    2400 6700
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5C49ACEC
P 5600 5450
F 0 "#PWR?" H 5600 5300 50  0001 C CNN
F 1 "+3.3V" H 5615 5623 50  0000 C CNN
F 2 "" H 5600 5450 50  0001 C CNN
F 3 "" H 5600 5450 50  0001 C CNN
	1    5600 5450
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5C49E3CA
P 750 6950
F 0 "#PWR?" H 750 6800 50  0001 C CNN
F 1 "+3.3V" H 765 7123 50  0000 C CNN
F 2 "" H 750 6950 50  0001 C CNN
F 3 "" H 750 6950 50  0001 C CNN
	1    750  6950
	1    0    0    -1  
$EndComp
$Comp
L power:+5V #PWR?
U 1 1 5C4A1F82
P 5000 6950
F 0 "#PWR?" H 5000 6800 50  0001 C CNN
F 1 "+5V" H 5015 7123 50  0000 C CNN
F 2 "" H 5000 6950 50  0001 C CNN
F 3 "" H 5000 6950 50  0001 C CNN
	1    5000 6950
	1    0    0    -1  
$EndComp
Text Label 10350 4150 0    50   ~ 0
PA6-Therm
$Comp
L power:+3.3V #PWR?
U 1 1 5C4929D3
P 1700 4000
F 0 "#PWR?" H 1700 3850 50  0001 C CNN
F 1 "+3.3V" H 1715 4173 50  0000 C CNN
F 2 "" H 1700 4000 50  0001 C CNN
F 3 "" H 1700 4000 50  0001 C CNN
	1    1700 4000
	1    0    0    -1  
$EndComp
$EndSCHEMATC
