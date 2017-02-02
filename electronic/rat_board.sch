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
LIBS:mylib
LIBS:rat_board-cache
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
L STM32F103C8T6 U3
U 1 1 5877EBF2
P 7250 2600
F 0 "U3" H 7250 2600 50  0001 L CNN
F 1 "STM32F103C8T6" H 6900 2500 50  0001 L CNN
F 2 "Housings_QFP:LQFP-48_7x7mm_Pitch0.5mm" H 6800 2300 50  0001 L CNN
F 3 "STMicroelectronics" H 6900 2400 50  0001 L CNN
	1    7250 2600
	1    0    0    -1  
$EndComp
Text Label 5650 2000 0    60   ~ 0
3V3
Text Label 8750 2200 2    60   ~ 0
SWDIO
Text Label 7750 1000 3    60   ~ 0
SWDCLK
Text Label 5650 2600 0    60   ~ 0
RESET
Text Label 5650 2500 0    60   ~ 0
OSCOUT
Text Label 5650 2400 0    60   ~ 0
OSCIN
$Comp
L R R6
U 1 1 5878DF2C
P 3150 1200
F 0 "R6" V 3230 1200 50  0000 C CNN
F 1 "10k" V 3150 1200 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 3080 1200 50  0001 C CNN
F 3 "" H 3150 1200 50  0000 C CNN
	1    3150 1200
	1    0    0    -1  
$EndComp
$Comp
L C C1
U 1 1 5878DF7B
P 3150 1650
F 0 "C1" H 3175 1750 50  0000 L CNN
F 1 "C" H 3175 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 3188 1500 50  0001 C CNN
F 3 "" H 3150 1650 50  0000 C CNN
	1    3150 1650
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH_SMALL_H SW1
U 1 1 5878DFA7
P 2900 1650
F 0 "SW1" H 2980 1760 50  0000 C CNN
F 1 "SW_PUSH_SMALL_H" H 3400 1700 50  0000 C CNN
F 2 "Buttons_Switches_ThroughHole:SW_PUSH_6mm" H 2900 1850 50  0001 C CNN
F 3 "" H 2900 1850 50  0000 C CNN
	1    2900 1650
	0    -1   -1   0   
$EndComp
$Comp
L GND #PWR01
U 1 1 5878E071
P 3150 1950
F 0 "#PWR01" H 3150 1700 50  0001 C CNN
F 1 "GND" H 3150 1800 50  0000 C CNN
F 2 "" H 3150 1950 50  0000 C CNN
F 3 "" H 3150 1950 50  0000 C CNN
	1    3150 1950
	1    0    0    -1  
$EndComp
$Comp
L R R10
U 1 1 5878E271
P 6000 7300
F 0 "R10" V 6080 7300 50  0000 C CNN
F 1 "100k" V 6000 7300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5930 7300 50  0001 C CNN
F 3 "" H 6000 7300 50  0000 C CNN
	1    6000 7300
	0    1    1    0   
$EndComp
$Comp
L R R9
U 1 1 5878E2B9
P 5100 7300
F 0 "R9" V 5180 7300 50  0000 C CNN
F 1 "100k" V 5100 7300 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 5030 7300 50  0001 C CNN
F 3 "" H 5100 7300 50  0000 C CNN
	1    5100 7300
	0    1    1    0   
$EndComp
$Comp
L GND #PWR02
U 1 1 5878E362
P 5300 7350
F 0 "#PWR02" H 5300 7100 50  0001 C CNN
F 1 "GND" H 5300 7200 50  0000 C CNN
F 2 "" H 5300 7350 50  0000 C CNN
F 3 "" H 5300 7350 50  0000 C CNN
	1    5300 7350
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR03
U 1 1 5878E37F
P 5800 7350
F 0 "#PWR03" H 5800 7100 50  0001 C CNN
F 1 "GND" H 5800 7200 50  0000 C CNN
F 2 "" H 5800 7350 50  0000 C CNN
F 3 "" H 5800 7350 50  0000 C CNN
	1    5800 7350
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR04
U 1 1 5878E417
P 3150 950
F 0 "#PWR04" H 3150 800 50  0001 C CNN
F 1 "+3V3" H 3150 1090 50  0000 C CNN
F 2 "" H 3150 950 50  0000 C CNN
F 3 "" H 3150 950 50  0000 C CNN
	1    3150 950 
	1    0    0    -1  
$EndComp
Text Label 7050 1000 3    60   ~ 0
BOOT0
Text Label 7350 4100 1    60   ~ 0
BOOT1
Text Label 6500 7300 2    60   ~ 0
BOOT0
Text Label 4600 7300 0    60   ~ 0
BOOT1
$Comp
L CONN_01X04 P7
U 1 1 5878EE8C
P 3250 7200
F 0 "P7" H 3250 7450 50  0000 C CNN
F 1 "JTAG" V 3350 7200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x04" H 3250 7200 50  0001 C CNN
F 3 "" H 3250 7200 50  0000 C CNN
	1    3250 7200
	-1   0    0    1   
$EndComp
$Comp
L GND #PWR05
U 1 1 5878EFB2
P 3700 7450
F 0 "#PWR05" H 3700 7200 50  0001 C CNN
F 1 "GND" H 3700 7300 50  0000 C CNN
F 2 "" H 3700 7450 50  0000 C CNN
F 3 "" H 3700 7450 50  0000 C CNN
	1    3700 7450
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR06
U 1 1 5878EFFC
P 3700 6950
F 0 "#PWR06" H 3700 6800 50  0001 C CNN
F 1 "+3V3" H 3700 7090 50  0000 C CNN
F 2 "" H 3700 6950 50  0000 C CNN
F 3 "" H 3700 6950 50  0000 C CNN
	1    3700 6950
	1    0    0    -1  
$EndComp
Text Label 4050 7250 2    60   ~ 0
SWDCLK
Text Label 4050 7150 2    60   ~ 0
SWDIO
Text Label 5650 2200 0    60   ~ 0
OSC32IN
Text Label 5650 2300 0    60   ~ 0
OSC32OUT
$Comp
L C C13
U 1 1 5878F688
P 10300 5900
F 0 "C13" H 10325 6000 50  0000 L CNN
F 1 "0.1uF" H 10325 5800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10338 5750 50  0001 C CNN
F 3 "" H 10300 5900 50  0000 C CNN
	1    10300 5900
	1    0    0    -1  
$EndComp
$Comp
L C C2
U 1 1 5878F74F
P 9500 1650
F 0 "C2" H 9525 1750 50  0000 L CNN
F 1 "100nF" H 9525 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 9538 1500 50  0001 C CNN
F 3 "" H 9500 1650 50  0000 C CNN
	1    9500 1650
	1    0    0    -1  
$EndComp
$Comp
L C C3
U 1 1 5878F78C
P 9800 1650
F 0 "C3" H 9825 1750 50  0000 L CNN
F 1 "100nF" H 9825 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 9838 1500 50  0001 C CNN
F 3 "" H 9800 1650 50  0000 C CNN
	1    9800 1650
	1    0    0    -1  
$EndComp
$Comp
L C C5
U 1 1 5878F7CC
P 10400 1650
F 0 "C5" H 10425 1750 50  0000 L CNN
F 1 "10nF" H 10425 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10438 1500 50  0001 C CNN
F 3 "" H 10400 1650 50  0000 C CNN
	1    10400 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR07
U 1 1 5878F859
P 10300 6050
F 0 "#PWR07" H 10300 5800 50  0001 C CNN
F 1 "GND" H 10300 5900 50  0000 C CNN
F 2 "" H 10300 6050 50  0000 C CNN
F 3 "" H 10300 6050 50  0000 C CNN
	1    10300 6050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR08
U 1 1 5878F900
P 9500 1800
F 0 "#PWR08" H 9500 1550 50  0001 C CNN
F 1 "GND" H 9500 1650 50  0000 C CNN
F 2 "" H 9500 1800 50  0000 C CNN
F 3 "" H 9500 1800 50  0000 C CNN
	1    9500 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR09
U 1 1 5878F932
P 9800 1800
F 0 "#PWR09" H 9800 1550 50  0001 C CNN
F 1 "GND" H 9800 1650 50  0000 C CNN
F 2 "" H 9800 1800 50  0000 C CNN
F 3 "" H 9800 1800 50  0000 C CNN
	1    9800 1800
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR010
U 1 1 5878F964
P 10400 1800
F 0 "#PWR010" H 10400 1550 50  0001 C CNN
F 1 "GND" H 10400 1650 50  0000 C CNN
F 2 "" H 10400 1800 50  0000 C CNN
F 3 "" H 10400 1800 50  0000 C CNN
	1    10400 1800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR011
U 1 1 5878FAD9
P 9500 1050
F 0 "#PWR011" H 9500 900 50  0001 C CNN
F 1 "+3V3" H 9500 1190 50  0000 C CNN
F 2 "" H 9500 1050 50  0000 C CNN
F 3 "" H 9500 1050 50  0000 C CNN
	1    9500 1050
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR012
U 1 1 5878FB0B
P 9800 1050
F 0 "#PWR012" H 9800 900 50  0001 C CNN
F 1 "+3V3" H 9800 1190 50  0000 C CNN
F 2 "" H 9800 1050 50  0000 C CNN
F 3 "" H 9800 1050 50  0000 C CNN
	1    9800 1050
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR013
U 1 1 5878FB3D
P 10400 1050
F 0 "#PWR013" H 10400 900 50  0001 C CNN
F 1 "+3V3" H 10400 1190 50  0000 C CNN
F 2 "" H 10400 1050 50  0000 C CNN
F 3 "" H 10400 1050 50  0000 C CNN
	1    10400 1050
	1    0    0    -1  
$EndComp
$Comp
L C C7
U 1 1 5878FF91
P 10400 2850
F 0 "C7" H 10425 2950 50  0000 L CNN
F 1 "20p" H 10425 2750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10438 2700 50  0001 C CNN
F 3 "" H 10400 2850 50  0000 C CNN
	1    10400 2850
	0    -1   -1   0   
$EndComp
$Comp
L Crystal Y2
U 1 1 587900B3
P 10750 3100
F 0 "Y2" H 10750 3250 50  0000 C CNN
F 1 "8MHz" V 10750 2850 50  0000 C CNN
F 2 "Crystals:Crystal_HC50-U_Vertical" H 10750 3100 50  0001 C CNN
F 3 "" H 10750 3100 50  0000 C CNN
	1    10750 3100
	0    1    1    0   
$EndComp
$Comp
L C C8
U 1 1 5879018F
P 10400 3350
F 0 "C8" H 10425 3450 50  0000 L CNN
F 1 "20p" H 10425 3250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10438 3200 50  0001 C CNN
F 3 "" H 10400 3350 50  0000 C CNN
	1    10400 3350
	0    -1   -1   0   
$EndComp
$Comp
L R ext1
U 1 1 5879026F
P 10100 3350
F 0 "ext1" V 10180 3350 50  0000 C CNN
F 1 "R" V 10100 3350 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 10030 3350 50  0001 C CNN
F 3 "" H 10100 3350 50  0000 C CNN
	1    10100 3350
	0    1    1    0   
$EndComp
Text Label 9350 2850 0    60   ~ 0
OSCIN
Text Label 9350 3350 0    60   ~ 0
OSCOUT
$Comp
L C C9
U 1 1 58790916
P 10100 3850
F 0 "C9" H 10125 3950 50  0000 L CNN
F 1 "20p" H 10125 3750 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10138 3700 50  0001 C CNN
F 3 "" H 10100 3850 50  0000 C CNN
	1    10100 3850
	0    -1   -1   0   
$EndComp
$Comp
L Crystal Y1
U 1 1 5879091C
P 10550 4100
F 0 "Y1" H 10550 4250 50  0000 C CNN
F 1 "32.768kHz" V 10550 3750 50  0000 C CNN
F 2 "Crystals:Crystal_MC-156" H 10550 4100 50  0001 C CNN
F 3 "" H 10550 4100 50  0000 C CNN
	1    10550 4100
	0    1    1    0   
$EndComp
$Comp
L C C10
U 1 1 58790922
P 10100 4350
F 0 "C10" H 10125 4450 50  0000 L CNN
F 1 "20p" H 10125 4250 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10138 4200 50  0001 C CNN
F 3 "" H 10100 4350 50  0000 C CNN
	1    10100 4350
	0    -1   -1   0   
$EndComp
Text Label 9350 3850 0    60   ~ 0
OSC32IN
Text Label 9350 4350 0    60   ~ 0
OSC32OUT
$Comp
L GND #PWR014
U 1 1 58790951
P 10750 4400
F 0 "#PWR014" H 10750 4150 50  0001 C CNN
F 1 "GND" H 10750 4250 50  0000 C CNN
F 2 "" H 10750 4400 50  0000 C CNN
F 3 "" H 10750 4400 50  0000 C CNN
	1    10750 4400
	1    0    0    -1  
$EndComp
$Comp
L C C4
U 1 1 587915B6
P 10100 1650
F 0 "C4" H 10125 1750 50  0000 L CNN
F 1 "4.7uF" H 10125 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10138 1500 50  0001 C CNN
F 3 "" H 10100 1650 50  0000 C CNN
	1    10100 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR015
U 1 1 587915BC
P 10100 1800
F 0 "#PWR015" H 10100 1550 50  0001 C CNN
F 1 "GND" H 10100 1650 50  0000 C CNN
F 2 "" H 10100 1800 50  0000 C CNN
F 3 "" H 10100 1800 50  0000 C CNN
	1    10100 1800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR016
U 1 1 587915C2
P 10100 1050
F 0 "#PWR016" H 10100 900 50  0001 C CNN
F 1 "+3V3" H 10100 1190 50  0000 C CNN
F 2 "" H 10100 1050 50  0000 C CNN
F 3 "" H 10100 1050 50  0000 C CNN
	1    10100 1050
	1    0    0    -1  
$EndComp
Text Label 9500 1400 1    60   ~ 0
VDD2
Text Label 9800 1400 1    60   ~ 0
VDD3
Text Label 10400 1400 1    60   ~ 0
VDDA
Text Label 10100 1400 1    60   ~ 0
VDD3
Text Label 5650 2800 0    60   ~ 0
3V3
Text Label 5650 2700 0    60   ~ 0
GND
$Comp
L C C6
U 1 1 58792524
P 10700 1650
F 0 "C6" H 10725 1750 50  0000 L CNN
F 1 "1uF" H 10725 1550 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 10738 1500 50  0001 C CNN
F 3 "" H 10700 1650 50  0000 C CNN
	1    10700 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR017
U 1 1 5879252A
P 10700 1800
F 0 "#PWR017" H 10700 1550 50  0001 C CNN
F 1 "GND" H 10700 1650 50  0000 C CNN
F 2 "" H 10700 1800 50  0000 C CNN
F 3 "" H 10700 1800 50  0000 C CNN
	1    10700 1800
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR018
U 1 1 58792530
P 10700 1050
F 0 "#PWR018" H 10700 900 50  0001 C CNN
F 1 "+3V3" H 10700 1190 50  0000 C CNN
F 2 "" H 10700 1050 50  0000 C CNN
F 3 "" H 10700 1050 50  0000 C CNN
	1    10700 1050
	1    0    0    -1  
$EndComp
Text Label 10700 1400 1    60   ~ 0
VDDA
$Comp
L GND #PWR019
U 1 1 58792D8A
P 10750 3450
F 0 "#PWR019" H 10750 3200 50  0001 C CNN
F 1 "GND" H 10750 3300 50  0000 C CNN
F 2 "" H 10750 3450 50  0000 C CNN
F 3 "" H 10750 3450 50  0000 C CNN
	1    10750 3450
	1    0    0    -1  
$EndComp
Text Label 6650 1000 3    60   ~ 0
3V3
Text Label 6750 1000 3    60   ~ 0
GND
Text Label 8750 2000 2    60   ~ 0
3V3
Text Label 8750 2100 2    60   ~ 0
GND
Text Label 7650 4100 1    60   ~ 0
GND
Text Label 7750 4100 1    60   ~ 0
3V3
$Comp
L LED D2
U 1 1 58795064
P 1250 1250
F 0 "D2" H 1250 1350 50  0000 C CNN
F 1 "LED_0" H 1250 1150 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 1250 1250 50  0001 C CNN
F 3 "" H 1250 1250 50  0000 C CNN
	1    1250 1250
	1    0    0    -1  
$EndComp
$Comp
L R R2
U 1 1 5879506A
P 1700 1250
F 0 "R2" V 1780 1250 50  0000 C CNN
F 1 "100" V 1700 1250 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1630 1250 50  0001 C CNN
F 3 "" H 1700 1250 50  0000 C CNN
	1    1700 1250
	0    1    1    0   
$EndComp
$Comp
L LED D3
U 1 1 587952A8
P 1250 1550
F 0 "D3" H 1250 1650 50  0000 C CNN
F 1 "LED_1" H 1250 1450 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 1250 1550 50  0001 C CNN
F 3 "" H 1250 1550 50  0000 C CNN
	1    1250 1550
	1    0    0    -1  
$EndComp
$Comp
L LED D4
U 1 1 58795367
P 1250 1850
F 0 "D4" H 1250 1950 50  0000 C CNN
F 1 "LED_2" H 1250 1750 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 1250 1850 50  0001 C CNN
F 3 "" H 1250 1850 50  0000 C CNN
	1    1250 1850
	1    0    0    -1  
$EndComp
$Comp
L LED D5
U 1 1 587954A6
P 1250 2150
F 0 "D5" H 1250 2250 50  0000 C CNN
F 1 "LED_3" H 1250 2050 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 1250 2150 50  0001 C CNN
F 3 "" H 1250 2150 50  0000 C CNN
	1    1250 2150
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR020
U 1 1 5879584C
P 850 3050
F 0 "#PWR020" H 850 2800 50  0001 C CNN
F 1 "GND" H 850 2900 50  0000 C CNN
F 2 "" H 850 3050 50  0000 C CNN
F 3 "" H 850 3050 50  0000 C CNN
	1    850  3050
	1    0    0    -1  
$EndComp
Text Label 2200 1550 2    60   ~ 0
LED1
Text Label 2200 1850 2    60   ~ 0
LED2
Text Label 2200 2150 2    60   ~ 0
LED3
$Comp
L TB6612FNG U2
U 1 1 587983A8
P 7200 5500
F 0 "U2" H 7200 5400 60  0000 C CNN
F 1 "TB6612FNG" H 7200 5600 60  0000 C CNN
F 2 "Housings_SSOP:SSOP-24_5.3x8.2mm_Pitch0.65mm" H 7200 5500 60  0001 C CNN
F 3 "" H 7200 5500 60  0001 C CNN
	1    7200 5500
	1    0    0    -1  
$EndComp
Text Label 8450 5550 2    60   ~ 0
GND
Text Label 5950 5150 0    60   ~ 0
GND
Text Label 5950 5250 0    60   ~ 0
GND
Text Label 5950 5750 0    60   ~ 0
GND
Text Label 5950 5850 0    60   ~ 0
GND
Text Label 8450 4950 2    60   ~ 0
VMX
Text Label 8450 5950 2    60   ~ 0
VMX
Text Label 8450 6050 2    60   ~ 0
VMX
$Comp
L GND #PWR021
U 1 1 58799B29
P 10600 6050
F 0 "#PWR021" H 10600 5800 50  0001 C CNN
F 1 "GND" H 10600 5900 50  0000 C CNN
F 2 "" H 10600 6050 50  0000 C CNN
F 3 "" H 10600 6050 50  0000 C CNN
	1    10600 6050
	1    0    0    -1  
$EndComp
Text Label 10600 5600 1    60   ~ 0
VMX
$Comp
L CP1 C14
U 1 1 58799BC8
P 10600 5900
F 0 "C14" H 10625 6000 50  0000 L CNN
F 1 "10uF" H 10625 5800 50  0000 L CNN
F 2 "SMD_Packages:SMD-1206_Pol" H 10600 5900 50  0001 C CNN
F 3 "" H 10600 5900 50  0000 C CNN
	1    10600 5900
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR022
U 1 1 58799F36
P 10600 5300
F 0 "#PWR022" H 10600 5150 50  0001 C CNN
F 1 "+BATT" H 10600 5440 50  0000 C CNN
F 2 "" H 10600 5300 50  0000 C CNN
F 3 "" H 10600 5300 50  0000 C CNN
	1    10600 5300
	1    0    0    -1  
$EndComp
$Comp
L C C11
U 1 1 5879A0ED
P 9650 5900
F 0 "C11" H 9675 6000 50  0000 L CNN
F 1 "0.1uF" H 9675 5800 50  0000 L CNN
F 2 "Capacitors_SMD:C_0603_HandSoldering" H 9688 5750 50  0001 C CNN
F 3 "" H 9650 5900 50  0000 C CNN
	1    9650 5900
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR023
U 1 1 5879A0F3
P 9650 6050
F 0 "#PWR023" H 9650 5800 50  0001 C CNN
F 1 "GND" H 9650 5900 50  0000 C CNN
F 2 "" H 9650 6050 50  0000 C CNN
F 3 "" H 9650 6050 50  0000 C CNN
	1    9650 6050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR024
U 1 1 5879A0FB
P 9950 6050
F 0 "#PWR024" H 9950 5800 50  0001 C CNN
F 1 "GND" H 9950 5900 50  0000 C CNN
F 2 "" H 9950 6050 50  0000 C CNN
F 3 "" H 9950 6050 50  0000 C CNN
	1    9950 6050
	1    0    0    -1  
$EndComp
$Comp
L CP1 C12
U 1 1 5879A103
P 9950 5900
F 0 "C12" H 9975 6000 50  0000 L CNN
F 1 "10uF" H 9975 5800 50  0000 L CNN
F 2 "SMD_Packages:SMD-1206_Pol" H 9950 5900 50  0001 C CNN
F 3 "" H 9950 5900 50  0000 C CNN
	1    9950 5900
	1    0    0    -1  
$EndComp
Text Label 8450 5350 2    60   ~ 0
3V3
Text Label 9950 5600 1    60   ~ 0
VCC_d
$Comp
L CONN_01X06 P5
U 1 1 5879A7FE
P 1200 6500
F 0 "P5" H 1200 6850 50  0000 C CNN
F 1 "LEFT_MOTOR" V 1300 6500 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 1200 6500 50  0001 C CNN
F 3 "" H 1200 6500 50  0000 C CNN
	1    1200 6500
	-1   0    0    1   
$EndComp
Text Label 1950 6750 2    60   ~ 0
M_A02
Text Label 1950 6250 2    60   ~ 0
M_A01
Text Label 1950 6350 2    60   ~ 0
GND
Text Label 1950 6450 2    60   ~ 0
ENC1B
Text Label 1950 6550 2    60   ~ 0
ENC1A
Text Label 1950 6650 2    60   ~ 0
3V3
$Comp
L CONN_01X06 P6
U 1 1 5879AFAE
P 1200 7200
F 0 "P6" H 1200 7550 50  0000 C CNN
F 1 "RIGHT_MOTOR" V 1300 7200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x06" H 1200 7200 50  0001 C CNN
F 3 "" H 1200 7200 50  0000 C CNN
	1    1200 7200
	-1   0    0    1   
$EndComp
Text Label 1950 7450 2    60   ~ 0
M_B02
Text Label 1950 6950 2    60   ~ 0
M_B01
Text Label 1950 7050 2    60   ~ 0
GND
Text Label 1950 7150 2    60   ~ 0
ENC2B
Text Label 1950 7250 2    60   ~ 0
ENC2A
Text Label 1950 7350 2    60   ~ 0
3V3
$Comp
L CONN_01X03 P1
U 1 1 5879B570
P 1200 4200
F 0 "P1" H 1200 4400 50  0000 C CNN
F 1 "FRONT_LEFT_IR" V 1300 4200 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 1200 4200 50  0001 C CNN
F 3 "" H 1200 4200 50  0000 C CNN
	1    1200 4200
	-1   0    0    1   
$EndComp
Text Label 1950 4300 2    60   ~ 0
IR1
Text Label 1950 4200 2    60   ~ 0
GND
Text Label 1950 4100 2    60   ~ 0
3V3
$Comp
L CONN_01X03 P4
U 1 1 5879BE92
P 1200 5850
F 0 "P4" H 1200 6050 50  0000 C CNN
F 1 "FRONT_RIGHT_IR" V 1300 5850 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 1200 5850 50  0001 C CNN
F 3 "" H 1200 5850 50  0000 C CNN
	1    1200 5850
	-1   0    0    1   
$EndComp
Text Label 1950 5950 2    60   ~ 0
IR4
Text Label 1950 5850 2    60   ~ 0
GND
Text Label 1950 5750 2    60   ~ 0
3V3
$Comp
L CONN_01X03 P2
U 1 1 5879C0EA
P 1200 4750
F 0 "P2" H 1200 4950 50  0000 C CNN
F 1 "SIDE_LEFT_IR" V 1300 4750 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 1200 4750 50  0001 C CNN
F 3 "" H 1200 4750 50  0000 C CNN
	1    1200 4750
	-1   0    0    1   
$EndComp
Text Label 1950 4850 2    60   ~ 0
IR2
Text Label 1950 4750 2    60   ~ 0
GND
Text Label 1950 4650 2    60   ~ 0
3V3
$Comp
L CONN_01X03 P3
U 1 1 5879C297
P 1200 5300
F 0 "P3" H 1200 5500 50  0000 C CNN
F 1 "SIDE_RIGHT_IR" V 1300 5300 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Straight_1x03" H 1200 5300 50  0001 C CNN
F 3 "" H 1200 5300 50  0000 C CNN
	1    1200 5300
	-1   0    0    1   
$EndComp
Text Label 1950 5400 2    60   ~ 0
IR3
Text Label 1950 5300 2    60   ~ 0
GND
Text Label 1950 5200 2    60   ~ 0
3V3
$Comp
L SW_PUSH SW4
U 1 1 5879F1E2
P 4300 1700
F 0 "SW4" H 4450 1810 50  0000 C CNN
F 1 "START_BUTTON" H 4300 1620 50  0000 C CNN
F 2 "Buttons_Switches_ThroughHole:SW_PUSH_6mm" H 4300 1700 50  0001 C CNN
F 3 "" H 4300 1700 50  0000 C CNN
	1    4300 1700
	1    0    0    -1  
$EndComp
$Comp
L R R8
U 1 1 5879F590
P 4800 1700
F 0 "R8" V 4880 1700 50  0000 C CNN
F 1 "10k" V 4800 1700 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4730 1700 50  0001 C CNN
F 3 "" H 4800 1700 50  0000 C CNN
	1    4800 1700
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR025
U 1 1 5879F86D
P 3900 1650
F 0 "#PWR025" H 3900 1500 50  0001 C CNN
F 1 "+3V3" H 3900 1790 50  0000 C CNN
F 2 "" H 3900 1650 50  0000 C CNN
F 3 "" H 3900 1650 50  0000 C CNN
	1    3900 1650
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR026
U 1 1 587A02F5
P 5050 1800
F 0 "#PWR026" H 5050 1550 50  0001 C CNN
F 1 "GND" H 5050 1650 50  0000 C CNN
F 2 "" H 5050 1800 50  0000 C CNN
F 3 "" H 5050 1800 50  0000 C CNN
	1    5050 1800
	1    0    0    -1  
$EndComp
$Comp
L SW_PUSH SW3
U 1 1 587A068D
P 4300 1100
F 0 "SW3" H 4450 1210 50  0000 C CNN
F 1 "SELECT_BUTTON" H 4300 1020 50  0000 C CNN
F 2 "Buttons_Switches_ThroughHole:SW_PUSH_6mm" H 4300 1100 50  0001 C CNN
F 3 "" H 4300 1100 50  0000 C CNN
	1    4300 1100
	1    0    0    -1  
$EndComp
$Comp
L R R7
U 1 1 587A0693
P 4800 1100
F 0 "R7" V 4880 1100 50  0000 C CNN
F 1 "10k" V 4800 1100 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 4730 1100 50  0001 C CNN
F 3 "" H 4800 1100 50  0000 C CNN
	1    4800 1100
	0    1    1    0   
$EndComp
$Comp
L +3V3 #PWR027
U 1 1 587A0699
P 3900 1050
F 0 "#PWR027" H 3900 900 50  0001 C CNN
F 1 "+3V3" H 3900 1190 50  0000 C CNN
F 2 "" H 3900 1050 50  0000 C CNN
F 3 "" H 3900 1050 50  0000 C CNN
	1    3900 1050
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR028
U 1 1 587A069F
P 5050 1200
F 0 "#PWR028" H 5050 950 50  0001 C CNN
F 1 "GND" H 5050 1050 50  0000 C CNN
F 2 "" H 5050 1200 50  0000 C CNN
F 3 "" H 5050 1200 50  0000 C CNN
	1    5050 1200
	1    0    0    -1  
$EndComp
Text Label 3900 1950 0    60   ~ 0
UB_START
Text Label 3900 1350 0    60   ~ 0
UB_SELECT
$Comp
L R R3
U 1 1 587A2320
P 1700 1550
F 0 "R3" V 1780 1550 50  0000 C CNN
F 1 "100" V 1700 1550 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1630 1550 50  0001 C CNN
F 3 "" H 1700 1550 50  0000 C CNN
	1    1700 1550
	0    1    1    0   
$EndComp
$Comp
L R R4
U 1 1 587A23EC
P 1700 1850
F 0 "R4" V 1780 1850 50  0000 C CNN
F 1 "100" V 1700 1850 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1630 1850 50  0001 C CNN
F 3 "" H 1700 1850 50  0000 C CNN
	1    1700 1850
	0    1    1    0   
$EndComp
$Comp
L R R5
U 1 1 587A24B8
P 1700 2150
F 0 "R5" V 1780 2150 50  0000 C CNN
F 1 "100" V 1700 2150 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1630 2150 50  0001 C CNN
F 3 "" H 1700 2150 50  0000 C CNN
	1    1700 2150
	0    1    1    0   
$EndComp
Text Label 3650 1450 2    60   ~ 0
RESET
Text Label 5950 4950 0    60   ~ 0
M_A01
Text Label 5950 5950 0    60   ~ 0
M_B01
Text Label 5950 5550 0    60   ~ 0
M_B02
Text Label 5950 5350 0    60   ~ 0
M_A02
Text Label 8450 5050 2    60   ~ 0
M_PWMA
Text Label 8450 5850 2    60   ~ 0
M_PWMB
Text Label 8450 5150 2    60   ~ 0
M_AIN2
Text Label 8450 5250 2    60   ~ 0
M_AIN1
Text Label 8450 5450 2    60   ~ 0
M_STBY
Text Label 8450 5650 2    60   ~ 0
M_BIN1
Text Label 8450 5750 2    60   ~ 0
M_BIN2
Text Notes 9400 700  0    60   ~ 0
MICROCONTROLLER CAPACITORS\n
Text Notes 9700 5000 0    60   ~ 0
DRIVER CAPACITORS\n
Text Notes 9400 2600 0    60   ~ 0
MICROCONTROLLER OSCILLATORS\n
Text Notes 6700 4600 0    60   ~ 0
DUAL MOTOR DRIVER
Text Notes 6600 750  0    60   ~ 0
STM32 MICROCONTROLLER
Text Notes 1250 750  0    60   ~ 0
USER LEDS
Text Notes 800  3800 0    60   ~ 0
SENSOR / MOTOR CONNECTORS
Text Notes 4250 6800 0    60   ~ 0
PROGRAMMING STUFF
Text Notes 3500 4600 0    60   ~ 0
POWER SUPPLY\n
Text Notes 3500 700  0    60   ~ 0
USER BUTTONS
$Comp
L +3V3 #PWR029
U 1 1 587A06CD
P 9950 5300
F 0 "#PWR029" H 9950 5150 50  0001 C CNN
F 1 "+3V3" H 9950 5440 50  0000 C CNN
F 2 "" H 9950 5300 50  0000 C CNN
F 3 "" H 9950 5300 50  0000 C CNN
	1    9950 5300
	1    0    0    -1  
$EndComp
$Comp
L LM7805 U1
U 1 1 587A0F96
P 3900 5750
F 0 "U1" H 4050 5554 50  0000 C CNN
F 1 "TR05S3V3" H 3900 5950 50  0000 C CNN
F 2 "Pin_Headers:Pin_Header_Angled_1x03" H 3900 5750 50  0001 C CNN
F 3 "" H 3900 5750 50  0000 C CNN
	1    3900 5750
	1    0    0    -1  
$EndComp
$Comp
L +3V3 #PWR030
U 1 1 587A133C
P 4550 5600
F 0 "#PWR030" H 4550 5450 50  0001 C CNN
F 1 "+3V3" H 4550 5740 50  0000 C CNN
F 2 "" H 4550 5600 50  0000 C CNN
F 3 "" H 4550 5600 50  0000 C CNN
	1    4550 5600
	1    0    0    -1  
$EndComp
$Comp
L GND #PWR031
U 1 1 587A1C89
P 3900 6100
F 0 "#PWR031" H 3900 5850 50  0001 C CNN
F 1 "GND" H 3900 5950 50  0000 C CNN
F 2 "" H 3900 6100 50  0000 C CNN
F 3 "" H 3900 6100 50  0000 C CNN
	1    3900 6100
	1    0    0    -1  
$EndComp
$Comp
L +BATT #PWR032
U 1 1 587A66DA
P 4350 4650
F 0 "#PWR032" H 4350 4500 50  0001 C CNN
F 1 "+BATT" H 4350 4790 50  0000 C CNN
F 2 "" H 4350 4650 50  0000 C CNN
F 3 "" H 4350 4650 50  0000 C CNN
	1    4350 4650
	1    0    0    -1  
$EndComp
Wire Wire Line
	6050 2700 5650 2700
Wire Wire Line
	6050 2800 5650 2800
Wire Wire Line
	8350 2000 8750 2000
Wire Wire Line
	8350 2100 8750 2100
Wire Wire Line
	6050 2000 5650 2000
Wire Wire Line
	6650 1400 6650 1000
Wire Wire Line
	6750 1400 6750 1000
Wire Wire Line
	7750 3700 7750 4100
Wire Wire Line
	8350 2200 8750 2200
Wire Wire Line
	7750 1400 7750 1000
Wire Wire Line
	6050 2600 5650 2600
Wire Wire Line
	6050 2400 5650 2400
Wire Wire Line
	6050 2500 5650 2500
Wire Wire Line
	3150 1350 3150 1500
Connection ~ 3150 1450
Wire Wire Line
	2900 1800 2900 1850
Wire Wire Line
	2900 1850 3150 1850
Wire Wire Line
	2900 1450 3650 1450
Wire Wire Line
	2900 1450 2900 1500
Wire Wire Line
	5300 7300 5300 7350
Wire Wire Line
	5800 7300 5800 7350
Wire Wire Line
	7050 1400 7050 1000
Wire Wire Line
	7350 3700 7350 4100
Wire Wire Line
	7650 3700 7650 4100
Wire Wire Line
	6150 7300 6500 7300
Wire Wire Line
	4600 7300 4950 7300
Wire Wire Line
	3450 7050 3700 7050
Wire Wire Line
	3450 7350 3700 7350
Wire Wire Line
	3450 7150 4050 7150
Wire Wire Line
	3450 7250 4050 7250
Wire Wire Line
	6050 2200 5650 2200
Wire Wire Line
	6050 2300 5650 2300
Wire Wire Line
	9350 2850 10250 2850
Wire Wire Line
	9350 3350 9950 3350
Wire Wire Line
	9350 3850 9950 3850
Wire Wire Line
	9350 4350 9950 4350
Wire Wire Line
	10250 4350 10750 4350
Wire Wire Line
	10750 4350 10750 4400
Wire Wire Line
	10550 4250 10550 4350
Connection ~ 10550 4350
Wire Wire Line
	10550 3950 10550 3850
Wire Wire Line
	10550 3850 10250 3850
Wire Wire Line
	10750 3250 10750 3450
Wire Wire Line
	10750 3350 10550 3350
Connection ~ 10750 3350
Wire Wire Line
	10550 2850 10750 2850
Wire Wire Line
	10750 2850 10750 2950
Wire Wire Line
	9500 1500 9500 1050
Wire Wire Line
	9800 1050 9800 1500
Wire Wire Line
	10100 1500 10100 1050
Wire Wire Line
	10400 1050 10400 1500
Wire Wire Line
	10700 1050 10700 1500
Wire Wire Line
	850  1250 1050 1250
Wire Wire Line
	1450 1250 1550 1250
Wire Wire Line
	1850 1550 2200 1550
Wire Wire Line
	850  1550 1050 1550
Wire Wire Line
	1450 1550 1550 1550
Wire Wire Line
	1850 1850 2200 1850
Wire Wire Line
	850  1850 1050 1850
Wire Wire Line
	1450 1850 1550 1850
Wire Wire Line
	1850 2150 2200 2150
Wire Wire Line
	850  2150 1050 2150
Wire Wire Line
	1450 2150 1550 2150
Connection ~ 850  1550
Connection ~ 850  1850
Connection ~ 850  2150
Connection ~ 3150 1850
Wire Wire Line
	3150 950  3150 1050
Wire Wire Line
	7950 4950 8450 4950
Wire Wire Line
	7950 5050 8450 5050
Wire Wire Line
	7950 5150 8450 5150
Wire Wire Line
	7950 5250 8450 5250
Wire Wire Line
	7950 5550 8450 5550
Wire Wire Line
	5950 5150 6450 5150
Wire Wire Line
	6450 5250 5950 5250
Wire Wire Line
	6450 5750 5950 5750
Wire Wire Line
	6450 5850 5950 5850
Wire Wire Line
	7950 5350 8450 5350
Wire Wire Line
	7950 5650 8450 5650
Wire Wire Line
	8450 5450 7950 5450
Wire Wire Line
	8450 5750 7950 5750
Wire Wire Line
	8450 5850 7950 5850
Wire Wire Line
	8450 5950 7950 5950
Wire Wire Line
	8450 6050 7950 6050
Wire Wire Line
	10600 5300 10600 5750
Wire Wire Line
	9950 5300 9950 5750
Wire Wire Line
	1400 4300 1950 4300
Wire Wire Line
	1400 6650 1950 6650
Wire Wire Line
	1400 6550 1950 6550
Wire Wire Line
	1400 6450 1950 6450
Wire Wire Line
	1400 6350 1950 6350
Wire Wire Line
	1400 6250 1950 6250
Wire Wire Line
	1400 7450 1950 7450
Wire Wire Line
	1400 7350 1950 7350
Wire Wire Line
	1400 7250 1950 7250
Wire Wire Line
	1400 7150 1950 7150
Wire Wire Line
	1400 7050 1950 7050
Wire Wire Line
	1400 6950 1950 6950
Wire Wire Line
	1950 4200 1400 4200
Wire Wire Line
	1950 4100 1400 4100
Wire Wire Line
	1400 5950 1950 5950
Wire Wire Line
	1950 5850 1400 5850
Wire Wire Line
	1950 5750 1400 5750
Wire Wire Line
	1400 4850 1950 4850
Wire Wire Line
	1950 4750 1400 4750
Wire Wire Line
	1950 4650 1400 4650
Wire Wire Line
	1400 5400 1950 5400
Wire Wire Line
	1950 5300 1400 5300
Wire Wire Line
	1950 5200 1400 5200
Wire Wire Line
	1400 6750 1950 6750
Wire Wire Line
	4950 1700 5050 1700
Wire Wire Line
	4600 1700 4650 1700
Wire Wire Line
	4950 1100 5050 1100
Wire Wire Line
	4600 1100 4650 1100
Wire Wire Line
	5950 4950 6450 4950
Wire Wire Line
	6450 5050 6350 5050
Wire Wire Line
	6350 5050 6350 4950
Connection ~ 6350 4950
Wire Wire Line
	6450 5350 5950 5350
Wire Wire Line
	6450 5450 6350 5450
Wire Wire Line
	6350 5450 6350 5350
Connection ~ 6350 5350
Wire Wire Line
	6450 5550 5950 5550
Wire Wire Line
	6450 5650 6350 5650
Wire Wire Line
	6350 5650 6350 5550
Connection ~ 6350 5550
Wire Wire Line
	6450 5950 5950 5950
Wire Wire Line
	6450 6050 6350 6050
Wire Wire Line
	6350 6050 6350 5950
Connection ~ 6350 5950
Wire Notes Line
	9000 500  9000 6500
Wire Notes Line
	9000 2300 11200 2300
Wire Notes Line
	9000 4750 11200 4750
Wire Notes Line
	2600 500  2600 7750
Wire Notes Line
	2600 3600 500  3600
Wire Wire Line
	4600 1700 4600 1950
Wire Wire Line
	4600 1950 3900 1950
Wire Wire Line
	4600 1100 4600 1350
Wire Wire Line
	4600 1350 3900 1350
Wire Wire Line
	3900 1100 4000 1100
Wire Wire Line
	3900 1050 3900 1100
Wire Wire Line
	5050 1100 5050 1200
Wire Wire Line
	3900 1650 3900 1700
Wire Wire Line
	3900 1700 4000 1700
Wire Wire Line
	5050 1700 5050 1800
Wire Wire Line
	5250 7300 5300 7300
Wire Wire Line
	5800 7300 5850 7300
Wire Wire Line
	3700 7350 3700 7450
Wire Wire Line
	3700 7050 3700 6950
Wire Notes Line
	6950 6550 2600 6550
Wire Notes Line
	5300 6550 5300 500 
Wire Notes Line
	2600 4300 9000 4300
Wire Wire Line
	3150 1800 3150 1950
Wire Notes Line
	2600 2300 5300 2300
Wire Wire Line
	8350 2400 8750 2400
Wire Wire Line
	8350 2300 8750 2300
Wire Wire Line
	3250 5700 3500 5700
Wire Wire Line
	4300 5700 4550 5700
Wire Wire Line
	4550 5700 4550 5600
Wire Wire Line
	3900 6000 3900 6100
Wire Wire Line
	8350 2600 8750 2600
Wire Wire Line
	8350 2700 8750 2700
Wire Wire Line
	6050 2900 5650 2900
Wire Wire Line
	6050 3000 5650 3000
Wire Wire Line
	6750 3700 6750 4100
Wire Wire Line
	6650 3700 6650 4100
Wire Wire Line
	6850 3700 6850 4100
Wire Wire Line
	6950 3700 6950 4100
Wire Wire Line
	7050 3700 7050 4100
Wire Wire Line
	7150 3700 7150 4100
Wire Wire Line
	7250 3700 7250 4100
Wire Wire Line
	6050 3100 5650 3100
Wire Wire Line
	7450 3700 7450 4100
Wire Wire Line
	7550 3700 7550 4100
Wire Wire Line
	8350 3100 8750 3100
Wire Wire Line
	8350 3000 8750 3000
Wire Wire Line
	8350 2900 8750 2900
Wire Wire Line
	8350 2800 8750 2800
Wire Wire Line
	6850 1400 6850 1000
Wire Wire Line
	6950 1400 6950 1000
Text Label 6850 1000 3    60   ~ 0
M_PWMA
Text Label 7350 1000 3    60   ~ 0
UB_SELECT
Wire Wire Line
	7450 1400 7450 1000
Text Label 7450 1000 3    60   ~ 0
M_STBY
Wire Wire Line
	6050 2100 5650 2100
Wire Wire Line
	8350 2500 8750 2500
Wire Wire Line
	7150 1400 7150 1000
Wire Wire Line
	7250 1400 7250 1000
Wire Wire Line
	7350 1400 7350 1000
Text Label 7150 1000 3    60   ~ 0
M_BIN2
Text Label 7250 1000 3    60   ~ 0
M_BIN1
Text Label 7650 1000 3    60   ~ 0
M_AIN2
Text Label 7550 1000 3    60   ~ 0
M_AIN1
Wire Wire Line
	7550 1000 7550 1400
Wire Wire Line
	7650 1000 7650 1400
Text Label 2200 1250 2    60   ~ 0
LED0
Wire Wire Line
	1850 1250 2200 1250
Wire Wire Line
	850  1250 850  3050
$Comp
L LED D6
U 1 1 5886AB08
P 1250 2500
F 0 "D6" H 1250 2600 50  0000 C CNN
F 1 "LED_4" H 1250 2400 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 1250 2500 50  0001 C CNN
F 3 "" H 1250 2500 50  0000 C CNN
	1    1250 2500
	1    0    0    -1  
$EndComp
$Comp
L LED D7
U 1 1 5886AB0E
P 1250 2800
F 0 "D7" H 1250 2900 50  0000 C CNN
F 1 "LED_5" H 1250 2700 50  0000 C CNN
F 2 "LEDs:LED-3MM" H 1250 2800 50  0001 C CNN
F 3 "" H 1250 2800 50  0000 C CNN
	1    1250 2800
	1    0    0    -1  
$EndComp
Text Label 2200 2500 2    60   ~ 0
LED4
Text Label 2200 2800 2    60   ~ 0
LED5
$Comp
L R R11
U 1 1 5886AB16
P 1700 2500
F 0 "R11" V 1780 2500 50  0000 C CNN
F 1 "100" V 1700 2500 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1630 2500 50  0001 C CNN
F 3 "" H 1700 2500 50  0000 C CNN
	1    1700 2500
	0    1    1    0   
$EndComp
$Comp
L R R12
U 1 1 5886AB1C
P 1700 2800
F 0 "R12" V 1780 2800 50  0000 C CNN
F 1 "100" V 1700 2800 50  0000 C CNN
F 2 "Resistors_SMD:R_0603_HandSoldering" V 1630 2800 50  0001 C CNN
F 3 "" H 1700 2800 50  0000 C CNN
	1    1700 2800
	0    1    1    0   
$EndComp
Wire Wire Line
	1850 2500 2200 2500
Wire Wire Line
	850  2500 1050 2500
Wire Wire Line
	1450 2500 1550 2500
Wire Wire Line
	1850 2800 2200 2800
Wire Wire Line
	850  2800 1050 2800
Wire Wire Line
	1450 2800 1550 2800
Connection ~ 850  2500
Connection ~ 850  2800
Text Label 6950 1000 3    60   ~ 0
M_PWMB
Text Label 7250 4100 1    60   ~ 0
IR1
Text Label 7150 4100 1    60   ~ 0
IR2
Text Label 7050 4100 1    60   ~ 0
IR3
Text Label 6950 4100 1    60   ~ 0
IR4
Text Label 8750 2600 2    60   ~ 0
ENC1B
Text Label 8750 2700 2    60   ~ 0
ENC1A
Text Label 5650 2900 0    60   ~ 0
ENC2A
Text Label 5650 3000 0    60   ~ 0
ENC2B
Text Label 8750 3000 2    60   ~ 0
LED0
Text Label 8750 2800 2    60   ~ 0
LED2
Text Label 8750 2900 2    60   ~ 0
LED1
Text Label 5650 2100 0    60   ~ 0
UB_START
Text Label 4550 5700 2    60   ~ 0
3V3
Text Label 6650 4100 1    60   ~ 0
LED3
Text Label 6750 4100 1    60   ~ 0
LED4
Text Label 6850 4100 1    60   ~ 0
LED5
Wire Wire Line
	4350 4650 4350 5100
$Comp
L CONN_01X02 P8
U 1 1 588D13FE
P 4850 4850
F 0 "P8" H 4850 5000 50  0000 C CNN
F 1 "CONN_01X02" V 4950 4850 50  0000 C CNN
F 2 "Socket_Strips:Socket_Strip_Straight_1x02" H 4850 4850 50  0001 C CNN
F 3 "" H 4850 4850 50  0000 C CNN
	1    4850 4850
	1    0    0    -1  
$EndComp
Wire Wire Line
	4350 4800 4650 4800
Connection ~ 4350 4800
$Comp
L GND #PWR033
U 1 1 588D1781
P 4600 5100
F 0 "#PWR033" H 4600 4850 50  0001 C CNN
F 1 "GND" H 4600 4950 50  0000 C CNN
F 2 "" H 4600 5100 50  0000 C CNN
F 3 "" H 4600 5100 50  0000 C CNN
	1    4600 5100
	1    0    0    -1  
$EndComp
Wire Wire Line
	4600 5100 4600 4900
Wire Wire Line
	4600 4900 4650 4900
$Comp
L SWITCH_INV SW2
U 1 1 588D2100
P 3750 5000
F 0 "SW2" H 3550 5150 50  0000 C CNN
F 1 "SWITCH_INV" H 3600 4850 50  0000 C CNN
F 2 "custom-lib:TR05S3V3" H 3750 5000 50  0001 C CNN
F 3 "" H 3750 5000 50  0000 C CNN
	1    3750 5000
	1    0    0    -1  
$EndComp
Wire Wire Line
	3250 5000 3250 5700
Wire Wire Line
	4350 5100 4250 5100
NoConn ~ 4250 4900
Text Label 3900 6100 0    60   ~ 0
GND
Wire Wire Line
	9650 5750 9650 5650
Wire Wire Line
	9650 5650 9950 5650
Connection ~ 9950 5650
Wire Wire Line
	10300 5750 10300 5650
Wire Wire Line
	10300 5650 10600 5650
Connection ~ 10600 5650
NoConn ~ 8750 2300
NoConn ~ 8750 2400
NoConn ~ 8750 2500
NoConn ~ 8750 3100
NoConn ~ 5650 3100
NoConn ~ 7450 4100
NoConn ~ 7550 4100
$EndSCHEMATC
