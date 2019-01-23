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
P 9650 4100
F 0 "U?" H 9900 2550 50  0000 C CNN
F 1 "ATxmega128A4U-AU" H 9650 2450 50  0000 C CNN
F 2 "Package_QFP:TQFP-44_10x10mm_P0.8mm" H 9650 4100 50  0001 C CIN
F 3 "http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-8387-8-and16-bit-AVR-Microcontroller-XMEGA-A4U_Datasheet.pdf" H 9650 4100 50  0001 C CNN
	1    9650 4100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR0101
U 1 1 5C4817C1
P 10550 600
F 0 "#PWR0101" H 10550 450 50  0001 C CNN
F 1 "+3.3V" H 10565 773 50  0000 C CNN
F 2 "" H 10550 600 50  0001 C CNN
F 3 "" H 10550 600 50  0001 C CNN
	1    10550 600 
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0102
U 1 1 5C481A0E
P 9650 5500
F 0 "#PWR0102" H 9650 5250 50  0001 C CNN
F 1 "GND" H 9655 5327 50  0000 C CNN
F 2 "" H 9650 5500 50  0001 C CNN
F 3 "" H 9650 5500 50  0001 C CNN
	1    9650 5500
	1    0    0    -1  
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 5C481BB8
P 10450 1000
F 0 "C?" V 10400 700 50  0000 C CNN
F 1 "10uF" V 10500 700 50  0000 C CNN
F 2 "" H 10450 1000 50  0001 C CNN
F 3 "~" H 10450 1000 50  0001 C CNN
	1    10450 1000
	0    1    1    0   
$EndComp
$Comp
L Device:CP1_Small C?
U 1 1 5C481D4D
P 10750 1400
F 0 "C?" V 10800 1150 50  0000 C CNN
F 1 "10uF" V 10700 1150 50  0000 C CNN
F 2 "" H 10750 1400 50  0001 C CNN
F 3 "~" H 10750 1400 50  0001 C CNN
	1    10750 1400
	0    -1   -1   0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C481E8E
P 10750 1600
F 0 "C?" V 10700 1850 50  0000 C CNN
F 1 "100nF" V 10800 1850 50  0000 C CNN
F 2 "" H 10750 1600 50  0001 C CNN
F 3 "~" H 10750 1600 50  0001 C CNN
	1    10750 1600
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C481F76
P 10450 1200
F 0 "C?" V 10400 900 50  0000 C CNN
F 1 "100nF" V 10500 900 50  0000 C CNN
F 2 "" H 10450 1200 50  0001 C CNN
F 3 "~" H 10450 1200 50  0001 C CNN
	1    10450 1200
	0    1    1    0   
$EndComp
$Comp
L Device:L_Small L?
U 1 1 5C4820F6
P 10550 800
F 0 "L?" H 10598 846 50  0000 L CNN
F 1 "10uH" H 10598 755 50  0000 L CNN
F 2 "" H 10550 800 50  0001 C CNN
F 3 "~" H 10550 800 50  0001 C CNN
	1    10550 800 
	1    0    0    -1  
$EndComp
$Comp
L Device:Ferrite_Bead_Small FB?
U 1 1 5C48226D
P 10650 1200
F 0 "FB?" H 10750 1246 50  0000 L CNN
F 1 "220@100MHz" H 10750 1155 50  0000 L CNN
F 2 "" V 10580 1200 50  0001 C CNN
F 3 "~" H 10650 1200 50  0001 C CNN
	1    10650 1200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR0103
U 1 1 5C4834EF
P 10350 1350
F 0 "#PWR0103" H 10350 1100 50  0001 C CNN
F 1 "GND" H 10355 1177 50  0000 C CNN
F 2 "" H 10350 1350 50  0001 C CNN
F 3 "" H 10350 1350 50  0001 C CNN
	1    10350 1350
	1    0    0    -1  
$EndComp
Wire Wire Line
	10550 600  10550 700 
Wire Wire Line
	10550 900  10550 1000
Wire Wire Line
	10350 1000 10350 1200
Connection ~ 10350 1200
Wire Wire Line
	10350 1200 10350 1350
Wire Wire Line
	10550 1000 10550 1100
Connection ~ 10550 1000
Connection ~ 10550 1100
Wire Wire Line
	10550 1100 10550 1200
Wire Wire Line
	10650 1300 10650 1400
Connection ~ 10650 1400
Wire Wire Line
	10650 1400 10650 1600
Wire Wire Line
	10850 1400 10850 1600
$Comp
L power:GND #PWR0104
U 1 1 5C484B92
P 10850 1700
F 0 "#PWR0104" H 10850 1450 50  0001 C CNN
F 1 "GND" H 11000 1650 50  0000 C CNN
F 2 "" H 10850 1700 50  0001 C CNN
F 3 "" H 10850 1700 50  0001 C CNN
	1    10850 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	10850 1600 10850 1700
Connection ~ 10850 1600
Wire Wire Line
	10550 1100 10650 1100
$Comp
L Connector_Generic:Conn_02x03_Odd_Even J?
U 1 1 5C487514
P 9000 1000
F 0 "J?" H 9050 1300 50  0000 C CNN
F 1 "Conn_02x03_Odd_Even" H 9050 1200 50  0000 C CNN
F 2 "" H 9000 1000 50  0001 C CNN
F 3 "~" H 9000 1000 50  0001 C CNN
	1    9000 1000
	1    0    0    -1  
$EndComp
NoConn ~ 9300 1000
NoConn ~ 8800 1000
Text Label 10550 1300 3    50   ~ 0
MCU_VCC
Wire Wire Line
	10550 1200 10550 1300
Connection ~ 10550 1200
Text Label 9650 2700 1    50   ~ 0
MCU_VCC
Text Label 9750 2700 1    50   ~ 0
MCU_AVCC
Text Label 10650 1600 3    50   ~ 0
MCU_AVCC
Text Label 8800 900  2    50   ~ 0
PDI_DATA
Text Label 8950 3100 2    50   ~ 0
PDI_DATA
$Comp
L Device:R_Small R?
U 1 1 5C48AF52
P 9550 1100
F 0 "R?" H 9609 1146 50  0000 L CNN
F 1 "10k" H 9609 1055 50  0000 L CNN
F 2 "" H 9550 1100 50  0001 C CNN
F 3 "~" H 9550 1100 50  0001 C CNN
	1    9550 1100
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C48B786
P 9300 1100
F 0 "#PWR?" H 9300 850 50  0001 C CNN
F 1 "GND" H 9305 927 50  0000 C CNN
F 2 "" H 9300 1100 50  0001 C CNN
F 3 "" H 9300 1100 50  0001 C CNN
	1    9300 1100
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 5C48BD0F
P 9550 750
F 0 "#PWR?" H 9550 600 50  0001 C CNN
F 1 "VDD" H 9567 923 50  0000 C CNN
F 2 "" H 9550 750 50  0001 C CNN
F 3 "" H 9550 750 50  0001 C CNN
	1    9550 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	9300 900  9550 900 
Wire Wire Line
	9550 900  9550 750 
Connection ~ 9550 900 
Wire Wire Line
	9550 900  9550 1000
Wire Wire Line
	8800 1100 8700 1100
Wire Wire Line
	8700 1100 8700 1300
Wire Wire Line
	8700 1300 9550 1300
Wire Wire Line
	9550 1300 9550 1200
Text Label 8950 3000 2    50   ~ 0
PDI_CLK
Text Label 9550 1300 0    50   ~ 0
PDI_CLK
$Comp
L MS560702BA03-50:MS560702BA03-50 U?
U 1 1 5C48EC0D
P 7000 1050
F 0 "U?" H 7000 1517 50  0000 C CNN
F 1 "MS560702BA03-50" H 7000 1426 50  0000 C CNN
F 2 "SON125P300X500X100-8N" H 7000 1050 50  0001 L BNN
F 3 "https://www.te.com/usa-en/product-MS560702BA03-50.html?te_bu=Cor&te_type=disp&te_campaign=seda_glo_cor-seda-global-disp-prtnr-fy19-seda-model-bom-cta_sma-317_1&elqCampaignId=32493" H 7000 1050 50  0001 L BNN
F 4 "MS560702BA03-50" H 7000 1050 50  0001 L BNN "Field4"
F 5 "MS560702BA03-50" H 7000 1050 50  0001 L BNN "Field5"
F 6 "Unavailable" H 7000 1050 50  0001 L BNN "Field6"
F 7 "Barometric Pressure Sensor, With Stainless Steel Cap" H 7000 1050 50  0001 L BNN "Field7"
F 8 "None" H 7000 1050 50  0001 L BNN "Field8"
F 9 "SMD-8 Measurement Specialties" H 7000 1050 50  0001 L BNN "Field9"
F 10 "TE Connectivity" H 7000 1050 50  0001 L BNN "Field10"
	1    7000 1050
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C48F1E8
P 8000 1050
F 0 "C?" H 8092 1096 50  0000 L CNN
F 1 "100nF" H 8092 1005 50  0000 L CNN
F 2 "" H 8000 1050 50  0001 C CNN
F 3 "~" H 8000 1050 50  0001 C CNN
	1    8000 1050
	1    0    0    -1  
$EndComp
Text Label 7600 1050 0    50   ~ 0
SDO
Text Label 6400 1250 2    50   ~ 0
SDI
Text Label 6400 1050 2    50   ~ 0
SCLK
$Comp
L power:VDD #PWR?
U 1 1 5C48FB82
P 7850 750
F 0 "#PWR?" H 7850 600 50  0001 C CNN
F 1 "VDD" H 7867 923 50  0000 C CNN
F 2 "" H 7850 750 50  0001 C CNN
F 3 "" H 7850 750 50  0001 C CNN
	1    7850 750 
	1    0    0    -1  
$EndComp
Wire Wire Line
	7600 850  7850 850 
Wire Wire Line
	7850 850  7850 750 
Wire Wire Line
	7850 850  8000 850 
Wire Wire Line
	8000 850  8000 950 
Connection ~ 7850 850 
Wire Wire Line
	8000 1150 8000 1250
Wire Wire Line
	8000 1250 7700 1250
Wire Wire Line
	6400 950  6150 950 
Wire Wire Line
	6150 950  6150 1500
Wire Wire Line
	6150 1500 7700 1500
Wire Wire Line
	7700 1500 7700 1250
Connection ~ 7700 1250
Wire Wire Line
	7700 1250 7600 1250
$Comp
L power:GND #PWR?
U 1 1 5C49115C
P 8000 1250
F 0 "#PWR?" H 8000 1000 50  0001 C CNN
F 1 "GND" H 8005 1077 50  0000 C CNN
F 2 "" H 8000 1250 50  0001 C CNN
F 3 "" H 8000 1250 50  0001 C CNN
	1    8000 1250
	1    0    0    -1  
$EndComp
Connection ~ 8000 1250
Text Label 6400 850  2    50   ~ 0
UNASSIGNED
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
L power:GND #PWR?
U 1 1 5C4934FB
P 1050 2900
F 0 "#PWR?" H 1050 2650 50  0001 C CNN
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
SDI
Text Label 1150 1800 2    50   ~ 0
SDO
Text Label 1150 1900 2    50   ~ 0
SCLK
$Comp
L power:GND #PWR?
U 1 1 5C495B95
P 1400 1100
F 0 "#PWR?" H 1400 850 50  0001 C CNN
F 1 "GND" H 1405 927 50  0000 C CNN
F 2 "" H 1400 1100 50  0001 C CNN
F 3 "" H 1400 1100 50  0001 C CNN
	1    1400 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR?
U 1 1 5C495DC2
P 1050 700
F 0 "#PWR?" H 1050 550 50  0001 C CNN
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
L power:GND #PWR?
U 1 1 5C49E493
P 2700 3100
F 0 "#PWR?" H 2700 2850 50  0001 C CNN
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
HOST_CSN
Text Label 1150 2200 2    50   ~ 0
BNO_BOOTN
Text Label 2650 1400 0    50   ~ 0
BNO_NRST
Text Label 2650 1800 0    50   ~ 0
ENV_SDA
Text Label 2650 1900 0    50   ~ 0
ENV_SCL
$Comp
L power:GND #PWR?
U 1 1 5C4AB245
P 3350 2900
F 0 "#PWR?" H 3350 2650 50  0001 C CNN
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
P 4050 1000
F 0 "TH?" V 3760 1000 50  0000 C CNN
F 1 "NTCLE100" V 3851 1000 50  0000 C CNN
F 2 "" H 4050 1050 50  0001 C CNN
F 3 "~" H 4050 1050 50  0001 C CNN
	1    4050 1000
	0    1    1    0   
$EndComp
$Comp
L Device:R_Small R?
U 1 1 5C4C93EB
P 4600 1000
F 0 "R?" V 4404 1000 50  0000 C CNN
F 1 "R_Small" V 4495 1000 50  0000 C CNN
F 2 "" H 4600 1000 50  0001 C CNN
F 3 "~" H 4600 1000 50  0001 C CNN
	1    4600 1000
	0    1    1    0   
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 5C4C9BA3
P 3700 1000
F 0 "#PWR?" H 3700 850 50  0001 C CNN
F 1 "VDD" H 3717 1173 50  0000 C CNN
F 2 "" H 3700 1000 50  0001 C CNN
F 3 "" H 3700 1000 50  0001 C CNN
	1    3700 1000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4C9DEF
P 4950 1000
F 0 "#PWR?" H 4950 750 50  0001 C CNN
F 1 "GND" H 4955 827 50  0000 C CNN
F 2 "" H 4950 1000 50  0001 C CNN
F 3 "" H 4950 1000 50  0001 C CNN
	1    4950 1000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3700 1000 3900 1000
Wire Wire Line
	4700 1000 4950 1000
Wire Wire Line
	4350 1000 4350 1100
Connection ~ 4350 1000
Wire Wire Line
	4350 1000 4500 1000
Wire Wire Line
	4200 1000 4350 1000
Text Label 4350 1100 3    50   ~ 0
ADC
$Comp
L ADP3338AKCZ-5-R7:ADP3338AKCZ-5-R7 U?
U 1 1 5C4D6E8B
P 4350 1800
F 0 "U?" H 4350 2170 50  0000 C CNN
F 1 "ADP3338AKCZ-5-R7" H 4350 2079 50  0000 C CNN
F 2 "SOT230P700X180-4N" H 4350 1800 50  0001 L BNN
F 3 "Unavailable" H 4350 1800 50  0001 L BNN
F 4 "Analog Devices" H 4350 1800 50  0001 L BNN "Field4"
F 5 "ADP3338AKCZ-5-R7" H 4350 1800 50  0001 L BNN "Field5"
F 6 "LDO Regulator Pos 5V 1.6A 4-Pin_3+Tab_ SOT-223 T/R" H 4350 1800 50  0001 L BNN "Field6"
F 7 "SOT-223-3 Analog Devices" H 4350 1800 50  0001 L BNN "Field7"
F 8 "None" H 4350 1800 50  0001 L BNN "Field8"
	1    4350 1800
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 5C4D7B60
P 3300 1600
F 0 "#PWR?" H 3300 1450 50  0001 C CNN
F 1 "VDD" H 3317 1773 50  0000 C CNN
F 2 "" H 3300 1600 50  0001 C CNN
F 3 "" H 3300 1600 50  0001 C CNN
	1    3300 1600
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4D7F6B
P 3500 2000
F 0 "#PWR?" H 3500 1750 50  0001 C CNN
F 1 "GND" V 3505 1872 50  0000 R CNN
F 2 "" H 3500 2000 50  0001 C CNN
F 3 "" H 3500 2000 50  0001 C CNN
	1    3500 2000
	0    1    1    0   
$EndComp
Wire Wire Line
	3500 2000 3650 2000
Wire Wire Line
	5050 1900 5200 1900
Wire Wire Line
	5050 2000 5200 2000
Wire Wire Line
	5200 1900 5200 1950
Connection ~ 5200 1950
Wire Wire Line
	5200 1950 5200 2000
$Comp
L power:-5V #PWR?
U 1 1 5C4E1B30
P 5500 1950
F 0 "#PWR?" H 5500 2050 50  0001 C CNN
F 1 "-5V" V 5515 2078 50  0000 L CNN
F 2 "" H 5500 1950 50  0001 C CNN
F 3 "" H 5500 1950 50  0001 C CNN
	1    5500 1950
	0    1    1    0   
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C4E2003
P 5400 2050
F 0 "C?" H 5492 2096 50  0000 L CNN
F 1 "1uF" H 5492 2005 50  0000 L CNN
F 2 "" H 5400 2050 50  0001 C CNN
F 3 "~" H 5400 2050 50  0001 C CNN
	1    5400 2050
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C4E29CB
P 3500 1700
F 0 "C?" H 3592 1746 50  0000 L CNN
F 1 "1uF" H 3592 1655 50  0000 L CNN
F 2 "" H 3500 1700 50  0001 C CNN
F 3 "~" H 3500 1700 50  0001 C CNN
	1    3500 1700
	1    0    0    -1  
$EndComp
Wire Wire Line
	5200 1950 5400 1950
Connection ~ 5400 1950
Wire Wire Line
	5400 1950 5500 1950
$Comp
L power:GND #PWR?
U 1 1 5C4E4FB9
P 5400 2150
F 0 "#PWR?" H 5400 1900 50  0001 C CNN
F 1 "GND" H 5405 1977 50  0000 C CNN
F 2 "" H 5400 2150 50  0001 C CNN
F 3 "" H 5400 2150 50  0001 C CNN
	1    5400 2150
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4E562C
P 3500 1600
F 0 "#PWR?" H 3500 1350 50  0001 C CNN
F 1 "GND" H 3505 1427 50  0000 C CNN
F 2 "" H 3500 1600 50  0001 C CNN
F 3 "" H 3500 1600 50  0001 C CNN
	1    3500 1600
	-1   0    0    1   
$EndComp
Wire Wire Line
	3300 1600 3300 1800
Wire Wire Line
	3300 1800 3500 1800
Connection ~ 3500 1800
Wire Wire Line
	3500 1800 3650 1800
$Comp
L ADP3338AKCZ-5-R7:ADP3338AKCZ-5-R7 U?
U 1 1 5C4ECC23
P 4900 2850
F 0 "U?" H 4900 3220 50  0000 C CNN
F 1 "ADP3338AKCZ-3.3-R7" H 4900 3129 50  0000 C CNN
F 2 "SOT230P700X180-4N" H 4900 2850 50  0001 L BNN
F 3 "Unavailable" H 4900 2850 50  0001 L BNN
F 4 "Analog Devices" H 4900 2850 50  0001 L BNN "Field4"
F 5 "ADP3338AKCZ-5-R7" H 4900 2850 50  0001 L BNN "Field5"
F 6 "LDO Regulator Pos 5V 1.6A 4-Pin_3+Tab_ SOT-223 T/R" H 4900 2850 50  0001 L BNN "Field6"
F 7 "SOT-223-3 Analog Devices" H 4900 2850 50  0001 L BNN "Field7"
F 8 "None" H 4900 2850 50  0001 L BNN "Field8"
	1    4900 2850
	1    0    0    -1  
$EndComp
$Comp
L power:VDD #PWR?
U 1 1 5C4ECC2A
P 3850 2650
F 0 "#PWR?" H 3850 2500 50  0001 C CNN
F 1 "VDD" H 3867 2823 50  0000 C CNN
F 2 "" H 3850 2650 50  0001 C CNN
F 3 "" H 3850 2650 50  0001 C CNN
	1    3850 2650
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4ECC30
P 4050 3050
F 0 "#PWR?" H 4050 2800 50  0001 C CNN
F 1 "GND" V 4055 2922 50  0000 R CNN
F 2 "" H 4050 3050 50  0001 C CNN
F 3 "" H 4050 3050 50  0001 C CNN
	1    4050 3050
	0    1    1    0   
$EndComp
Wire Wire Line
	4050 3050 4200 3050
Wire Wire Line
	5600 2950 5750 2950
Wire Wire Line
	5600 3050 5750 3050
Wire Wire Line
	5750 2950 5750 3000
Connection ~ 5750 3000
Wire Wire Line
	5750 3000 5750 3050
$Comp
L Device:C_Small C?
U 1 1 5C4ECC42
P 5950 3100
F 0 "C?" H 6042 3146 50  0000 L CNN
F 1 "1uF" H 6042 3055 50  0000 L CNN
F 2 "" H 5950 3100 50  0001 C CNN
F 3 "~" H 5950 3100 50  0001 C CNN
	1    5950 3100
	1    0    0    -1  
$EndComp
$Comp
L Device:C_Small C?
U 1 1 5C4ECC49
P 4050 2750
F 0 "C?" H 4142 2796 50  0000 L CNN
F 1 "1uF" H 4142 2705 50  0000 L CNN
F 2 "" H 4050 2750 50  0001 C CNN
F 3 "~" H 4050 2750 50  0001 C CNN
	1    4050 2750
	1    0    0    -1  
$EndComp
Wire Wire Line
	5750 3000 5950 3000
Connection ~ 5950 3000
Wire Wire Line
	5950 3000 6050 3000
$Comp
L power:GND #PWR?
U 1 1 5C4ECC53
P 5950 3200
F 0 "#PWR?" H 5950 2950 50  0001 C CNN
F 1 "GND" H 5955 3027 50  0000 C CNN
F 2 "" H 5950 3200 50  0001 C CNN
F 3 "" H 5950 3200 50  0001 C CNN
	1    5950 3200
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C4ECC59
P 4050 2650
F 0 "#PWR?" H 4050 2400 50  0001 C CNN
F 1 "GND" H 4055 2477 50  0000 C CNN
F 2 "" H 4050 2650 50  0001 C CNN
F 3 "" H 4050 2650 50  0001 C CNN
	1    4050 2650
	-1   0    0    1   
$EndComp
Wire Wire Line
	3850 2650 3850 2850
Wire Wire Line
	3850 2850 4050 2850
Connection ~ 4050 2850
Wire Wire Line
	4050 2850 4200 2850
$Comp
L power:-3V3 #PWR?
U 1 1 5C4F216F
P 6050 3000
F 0 "#PWR?" H 6050 3100 50  0001 C CNN
F 1 "-3V3" V 6065 3128 50  0000 L CNN
F 2 "" H 6050 3000 50  0001 C CNN
F 3 "" H 6050 3000 50  0001 C CNN
	1    6050 3000
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
P 6450 1950
F 0 "U?" H 6750 2117 50  0000 C CNN
F 1 "SS495A" H 6750 2026 50  0000 C CNN
F 2 "SIP3-UA" H 6450 1950 50  0001 L BNN
F 3 "None" H 6450 1950 50  0001 L BNN
F 4 "Sensor; Sink; 4.5 to 10.5 VDC; 8.7 mA @5 VDC; 0.2 VDC _Typ._/0.4 VDC _Min._" H 6450 1950 50  0001 L BNN "Field4"
F 5 "Honeywell Sensing" H 6450 1950 50  0001 L BNN "Field5"
F 6 "SS495A" H 6450 1950 50  0001 L BNN "Field6"
F 7 "Unavailable" H 6450 1950 50  0001 L BNN "Field7"
F 8 "TO-92 Honeywell" H 6450 1950 50  0001 L BNN "Field8"
	1    6450 1950
	1    0    0    -1  
$EndComp
$Comp
L power:VCC #PWR?
U 1 1 5C4FE341
P 6000 2000
F 0 "#PWR?" H 6000 1850 50  0001 C CNN
F 1 "VCC" H 6017 2173 50  0000 C CNN
F 2 "" H 6000 2000 50  0001 C CNN
F 3 "" H 6000 2000 50  0001 C CNN
	1    6000 2000
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C501DB3
P 6150 2300
F 0 "#PWR?" H 6150 2050 50  0001 C CNN
F 1 "GND" H 6155 2127 50  0000 C CNN
F 2 "" H 6150 2300 50  0001 C CNN
F 3 "" H 6150 2300 50  0001 C CNN
	1    6150 2300
	1    0    0    -1  
$EndComp
Wire Wire Line
	6150 2300 6150 2250
Wire Wire Line
	6150 2250 6250 2250
Text Label 7250 2050 0    50   ~ 0
UNASSIGNED
$Comp
L Device:C_Small C?
U 1 1 5C5057CC
P 6150 1950
F 0 "C?" H 6242 1996 50  0000 L CNN
F 1 "1uF" H 6242 1905 50  0000 L CNN
F 2 "" H 6150 1950 50  0001 C CNN
F 3 "~" H 6150 1950 50  0001 C CNN
	1    6150 1950
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR?
U 1 1 5C50B237
P 6150 1800
F 0 "#PWR?" H 6150 1550 50  0001 C CNN
F 1 "GND" H 6155 1627 50  0000 C CNN
F 2 "" H 6150 1800 50  0001 C CNN
F 3 "" H 6150 1800 50  0001 C CNN
	1    6150 1800
	-1   0    0    1   
$EndComp
Wire Wire Line
	6000 2000 6000 2050
Wire Wire Line
	6000 2050 6150 2050
Connection ~ 6150 2050
Wire Wire Line
	6150 2050 6250 2050
Wire Wire Line
	6150 1800 6150 1850
$EndSCHEMATC
