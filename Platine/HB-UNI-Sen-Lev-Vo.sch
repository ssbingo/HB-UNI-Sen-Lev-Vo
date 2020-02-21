EESchema Schematic File Version 4
LIBS:HB-UNI-Sen-Lev-Vo-cache
EELAYER 26 0
EELAYER END
$Descr A4 11693 8268
encoding utf-8
Sheet 1 1
Title "HB-Sen-Multi"
Date "2019-03-11"
Rev "V1.0.1"
Comp ""
Comment1 "© Silvio Sternitzke"
Comment2 "Multi Sensoren PBC"
Comment3 ""
Comment4 ""
$EndDescr
$Comp
L Homebrew:ArduinoProMini U1
U 1 1 5C7AE7E4
P 2595 3200
F 0 "U1" H 2570 4687 60  0000 C CNN
F 1 "ArduinoProMini" H 2570 4581 60  0000 C CNN
F 2 "Homebrew:ArduinoProMini" H 2595 3100 60  0001 C CNN
F 3 "" H 2595 3100 60  0001 C CNN
	1    2595 3200
	1    0    0    -1  
$EndComp
$Comp
L Homebrew:CC1101 U2
U 1 1 5C7AEACB
P 2390 5570
F 0 "U2" H 2640 6407 60  0000 C CNN
F 1 "CC1101" H 2640 6301 60  0000 C CNN
F 2 "Homebrew:CC1101" H 2390 5570 60  0001 C CNN
F 3 "" H 2390 5570 60  0001 C CNN
	1    2390 5570
	1    0    0    -1  
$EndComp
$Comp
L Switch:SW_DIP_x01 SW1
U 1 1 5C7AED1F
P 4000 2600
F 0 "SW1" H 4225 2650 50  0000 C CNN
F 1 "CONFIG_BUTTON" H 3930 2455 50  0000 C CNN
F 2 "Button_Switch_THT:SW_PUSH_6mm_H5mm" H 4000 2600 50  0001 C CNN
F 3 "" H 4000 2600 50  0001 C CNN
	1    4000 2600
	1    0    0    -1  
$EndComp
$Comp
L Connector_Generic:Conn_01x02 J1
U 1 1 5C7AF15A
P 1170 1170
F 0 "J1" H 1270 1040 50  0000 C CNN
F 1 "Versorgungsspannung" H 1170 1265 50  0000 C CNN
F 2 "TerminalBlock_WAGO:TerminalBlock_WAGO_236-102_1x02_P5.00mm_45Degree" H 1170 1170 50  0001 C CNN
F 3 "~" H 1170 1170 50  0001 C CNN
	1    1170 1170
	-1   0    0    1   
$EndComp
$Comp
L Connector:Conn_01x04_Male J2
U 1 1 5C7AF68A
P 1895 1170
F 0 "J2" H 1960 910 50  0000 R CNN
F 1 "D24V5F5" H 2080 1350 50  0000 R CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x04_P2.54mm_Vertical" H 1895 1170 50  0001 C CNN
F 3 "~" H 1895 1170 50  0001 C CNN
	1    1895 1170
	-1   0    0    1   
$EndComp
$Comp
L Device:CP C1
U 1 1 5C7AF893
P 5280 1100
F 0 "C1" H 5398 1146 50  0000 L CNN
F 1 "10uF" H 5398 1055 50  0000 L CNN
F 2 "Capacitor_THT:CP_Radial_D4.0mm_P1.50mm" H 5318 950 50  0001 C CNN
F 3 "~" H 5280 1100 50  0001 C CNN
	1    5280 1100
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR09
U 1 1 5C7AFACA
P 5280 880
F 0 "#PWR09" H 5280 730 50  0001 C CNN
F 1 "+3.3V" H 5295 1053 50  0000 C CNN
F 2 "" H 5280 880 50  0001 C CNN
F 3 "" H 5280 880 50  0001 C CNN
	1    5280 880 
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR03
U 1 1 5C7AFB07
P 1720 1865
F 0 "#PWR03" H 1720 1715 50  0001 C CNN
F 1 "+3.3V" H 1735 2038 50  0000 C CNN
F 2 "" H 1720 1865 50  0001 C CNN
F 3 "" H 1720 1865 50  0001 C CNN
	1    1720 1865
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR05
U 1 1 5C7AFB44
P 1950 4925
F 0 "#PWR05" H 1950 4775 50  0001 C CNN
F 1 "+3.3V" H 1965 5098 50  0000 C CNN
F 2 "" H 1950 4925 50  0001 C CNN
F 3 "" H 1950 4925 50  0001 C CNN
	1    1950 4925
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR010
U 1 1 5C7AFC9C
P 5280 1320
F 0 "#PWR010" H 5280 1070 50  0001 C CNN
F 1 "GND" H 5285 1147 50  0000 C CNN
F 2 "" H 5280 1320 50  0001 C CNN
F 3 "" H 5280 1320 50  0001 C CNN
	1    5280 1320
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR08
U 1 1 5C7AFD53
P 4300 2730
F 0 "#PWR08" H 4300 2480 50  0001 C CNN
F 1 "GND" H 4305 2557 50  0000 C CNN
F 2 "" H 4300 2730 50  0001 C CNN
F 3 "" H 4300 2730 50  0001 C CNN
	1    4300 2730
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR04
U 1 1 5C7AFD90
P 1755 3830
F 0 "#PWR04" H 1755 3580 50  0001 C CNN
F 1 "GND" H 1760 3657 50  0000 C CNN
F 2 "" H 1755 3830 50  0001 C CNN
F 3 "" H 1755 3830 50  0001 C CNN
	1    1755 3830
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR06
U 1 1 5C7AFDD7
P 1950 5650
F 0 "#PWR06" H 1950 5400 50  0001 C CNN
F 1 "GND" H 1955 5477 50  0000 C CNN
F 2 "" H 1950 5650 50  0001 C CNN
F 3 "" H 1950 5650 50  0001 C CNN
	1    1950 5650
	1    0    0    -1  
$EndComp
$Comp
L Device:R R1
U 1 1 5C7B09BD
P 3690 2200
F 0 "R1" V 3645 2040 50  0000 C CNN
F 1 "330" V 3685 2205 50  0000 C CNN
F 2 "Resistor_THT:R_Axial_DIN0207_L6.3mm_D2.5mm_P2.54mm_Vertical" V 3620 2200 50  0001 C CNN
F 3 "~" H 3690 2200 50  0001 C CNN
	1    3690 2200
	0    -1   -1   0   
$EndComp
Wire Wire Line
	3295 2200 3540 2200
Wire Wire Line
	3840 2200 4000 2200
Wire Wire Line
	3295 2600 3700 2600
Wire Wire Line
	4300 2200 4300 2600
Wire Wire Line
	4300 2600 4300 2730
Connection ~ 4300 2600
Wire Wire Line
	2090 5020 1950 5020
Wire Wire Line
	1950 5020 1950 4925
Wire Wire Line
	1950 5570 2090 5570
Wire Wire Line
	1950 5650 1950 5570
Wire Wire Line
	1755 3700 1845 3700
Wire Wire Line
	1755 3700 1755 3800
Wire Wire Line
	1755 3800 1845 3800
Wire Wire Line
	1755 3800 1755 3830
Connection ~ 1755 3800
Wire Wire Line
	1720 1865 1720 2000
Wire Wire Line
	1720 2000 1845 2000
Wire Wire Line
	3295 2800 3600 2800
Wire Wire Line
	3295 2900 3600 2900
Wire Wire Line
	3295 3000 3600 3000
Wire Wire Line
	3295 3100 3600 3100
Text Label 3600 2800 2    50   ~ 0
SS
Text Label 3600 2900 2    50   ~ 0
MOSI
Text Label 3600 3000 2    50   ~ 0
MISO
Text Label 3600 3100 2    50   ~ 0
SCK
Wire Wire Line
	3600 4200 3295 4200
Text Label 3600 4200 2    50   ~ 0
RSET
NoConn ~ 3295 4000
NoConn ~ 3295 3900
NoConn ~ 3295 3600
NoConn ~ 3295 3500
NoConn ~ 3295 2700
NoConn ~ 3295 2500
NoConn ~ 3295 2400
NoConn ~ 3295 2300
NoConn ~ 3190 5320
Wire Wire Line
	1370 1170 1610 1170
Wire Wire Line
	1370 1070 1535 1070
Wire Wire Line
	1695 895  1695 970 
$Comp
L power:GND #PWR01
U 1 1 5C7C1DF7
P 1535 895
F 0 "#PWR01" H 1535 645 50  0001 C CNN
F 1 "GND" H 1540 722 50  0000 C CNN
F 2 "" H 1535 895 50  0001 C CNN
F 3 "" H 1535 895 50  0001 C CNN
	1    1535 895 
	-1   0    0    1   
$EndComp
Wire Wire Line
	1535 895  1535 1070
Connection ~ 1535 1070
Wire Wire Line
	1535 1070 1695 1070
NoConn ~ 1695 1270
Wire Wire Line
	5280 880  5280 950 
Wire Wire Line
	5280 1250 5280 1320
$Comp
L Connector_Generic:Conn_01x03 J6
U 1 1 5C7D87A9
P 6715 2285
F 0 "J6" H 6795 2327 50  0000 L CNN
F 1 "Sensor analog (1)" H 6795 2236 50  0000 L CNN
F 2 "TerminalBlock_WAGO:TerminalBlock_WAGO_236-103_1x03_P5.00mm_45Degree" H 6715 2285 50  0001 C CNN
F 3 "~" H 6715 2285 50  0001 C CNN
	1    6715 2285
	1    0    0    -1  
$EndComp
Wire Wire Line
	6180 2185 6515 2185
$Comp
L power:GND #PWR016
U 1 1 5C7DD46A
P 6180 2385
F 0 "#PWR016" H 6180 2135 50  0001 C CNN
F 1 "GND" H 6185 2212 50  0000 C CNN
F 2 "" H 6180 2385 50  0001 C CNN
F 3 "" H 6180 2385 50  0001 C CNN
	1    6180 2385
	1    0    0    -1  
$EndComp
Wire Wire Line
	6180 2385 6515 2385
Wire Wire Line
	6515 2285 6180 2285
Wire Wire Line
	3295 3300 3600 3300
Text Label 3600 3300 2    50   ~ 0
A0
Text Label 6180 2285 0    50   ~ 0
A0
$Comp
L Connector_Generic:Conn_01x06 J8
U 1 1 5C7EC873
P 8940 5810
F 0 "J8" H 9019 5802 50  0000 L CNN
F 1 "PROG" H 9019 5711 50  0000 L CNN
F 2 "Connector_PinHeader_2.54mm:PinHeader_1x06_P2.54mm_Vertical" H 8940 5810 50  0001 C CNN
F 3 "~" H 8940 5810 50  0001 C CNN
	1    8940 5810
	1    0    0    -1  
$EndComp
$Comp
L power:+3.3V #PWR011
U 1 1 5C7EDCEF
P 8270 5610
F 0 "#PWR011" H 8270 5460 50  0001 C CNN
F 1 "+3.3V" H 8285 5783 50  0000 C CNN
F 2 "" H 8270 5610 50  0001 C CNN
F 3 "" H 8270 5610 50  0001 C CNN
	1    8270 5610
	1    0    0    -1  
$EndComp
$Comp
L power:GND #PWR012
U 1 1 5C7EDD54
P 8270 5710
F 0 "#PWR012" H 8270 5460 50  0001 C CNN
F 1 "GND" H 8275 5537 50  0000 C CNN
F 2 "" H 8270 5710 50  0001 C CNN
F 3 "" H 8270 5710 50  0001 C CNN
	1    8270 5710
	1    0    0    -1  
$EndComp
Wire Wire Line
	8270 5710 8740 5710
Wire Wire Line
	8270 5610 8740 5610
Wire Wire Line
	8415 5810 8740 5810
Wire Wire Line
	8740 5910 8415 5910
Wire Wire Line
	8740 6010 8415 6010
Wire Wire Line
	8740 6110 8415 6110
Text Label 8415 5810 0    50   ~ 0
MOSI
Text Label 8415 5910 0    50   ~ 0
SCK
Text Label 8415 6010 0    50   ~ 0
MISO
Text Label 8415 6110 0    50   ~ 0
RSET
$Comp
L power:PWR_FLAG #FLG01
U 1 1 5C7F678C
P 2200 810
F 0 "#FLG01" H 2200 885 50  0001 C CNN
F 1 "PWR_FLAG" H 2200 984 50  0000 C CNN
F 2 "" H 2200 810 50  0001 C CNN
F 3 "~" H 2200 810 50  0001 C CNN
	1    2200 810 
	1    0    0    -1  
$EndComp
Wire Wire Line
	2200 810  1760 810 
Wire Wire Line
	1760 810  1760 895 
Wire Wire Line
	1760 895  1695 895 
Wire Wire Line
	3190 5020 3520 5020
Wire Wire Line
	3190 5120 3520 5120
Wire Wire Line
	3190 5220 3520 5220
Wire Wire Line
	3190 5420 3520 5420
Wire Wire Line
	3190 5520 3520 5520
Text Label 3520 5020 2    50   ~ 0
MOSI
Text Label 3520 5120 2    50   ~ 0
SCK
Text Label 3520 5220 2    50   ~ 0
MISO
Text Label 3520 5420 2    50   ~ 0
D2
Text Label 3520 5520 2    50   ~ 0
SS
Wire Wire Line
	3295 2000 3985 2000
Text Label 3985 2000 2    50   ~ 0
D2
$Comp
L power:+12V #PWR07
U 1 1 5C92E18B
P 1610 1260
F 0 "#PWR07" H 1610 1110 50  0001 C CNN
F 1 "+12V" H 1625 1433 50  0000 C CNN
F 2 "" H 1610 1260 50  0001 C CNN
F 3 "" H 1610 1260 50  0001 C CNN
	1    1610 1260
	-1   0    0    1   
$EndComp
Wire Wire Line
	1610 1260 1610 1170
Connection ~ 1610 1170
Wire Wire Line
	1610 1170 1695 1170
$Comp
L power:+12V #PWR014
U 1 1 5C930E32
P 6180 2140
F 0 "#PWR014" H 6180 1990 50  0001 C CNN
F 1 "+12V" H 6195 2313 50  0000 C CNN
F 2 "" H 6180 2140 50  0001 C CNN
F 3 "" H 6180 2140 50  0001 C CNN
	1    6180 2140
	1    0    0    -1  
$EndComp
Wire Wire Line
	6180 2140 6180 2185
NoConn ~ 3295 3400
NoConn ~ 3295 3800
$Comp
L Device:LED D1
U 1 1 5C7AEC5B
P 4150 2200
F 0 "D1" H 4050 2245 50  0000 C CNN
F 1 "LED" H 4145 2300 50  0000 C CNN
F 2 "LED_THT:LED_D3.0mm" H 4150 2200 50  0001 C CNN
F 3 "~" H 4150 2200 50  0001 C CNN
	1    4150 2200
	-1   0    0    1   
$EndComp
NoConn ~ 3295 2100
$Comp
L power:+5V #PWR013
U 1 1 5C93BA54
P 1695 895
F 0 "#PWR013" H 1695 745 50  0001 C CNN
F 1 "+5V" H 1710 1068 50  0000 C CNN
F 2 "" H 1695 895 50  0001 C CNN
F 3 "" H 1695 895 50  0001 C CNN
	1    1695 895 
	1    0    0    -1  
$EndComp
Connection ~ 1695 895 
$Comp
L power:+5V #PWR02
U 1 1 5C93BAEB
P 1510 1860
F 0 "#PWR02" H 1510 1710 50  0001 C CNN
F 1 "+5V" H 1525 2033 50  0000 C CNN
F 2 "" H 1510 1860 50  0001 C CNN
F 3 "" H 1510 1860 50  0001 C CNN
	1    1510 1860
	1    0    0    -1  
$EndComp
Wire Wire Line
	1510 1860 1510 2100
Wire Wire Line
	1510 2100 1845 2100
NoConn ~ 3295 3700
$EndSCHEMATC
