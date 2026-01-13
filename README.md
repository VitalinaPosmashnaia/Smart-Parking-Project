STM32 Smart Parking System

A scalable smart parking system based on STM32F407G-DISC1 microcontrollers using a Master-Slave architecture over I2C.

The system consists of independent Sensor Nodes (Slaves) that monitor parking spots using Ultrasonic Sensors and a Central Unit (Master) that aggregates data and displays available spots on an LCD.

# Features
Distributed Architecture: Uses I2C communication to connect multiple sensor boards to one main controller.
Real-time Monitoring: Uses TIM1 Input Capture with HC-SR04 sensors for precise distance measurement (microsecond accuracy).
 Visual Feedback:
    Local: Red/Green LEDs at each parking spot indicate availability.
    Central: LCD 1602 display shows the total count of free spots.
Robust I2C Communication: Implements error handling and auto-recovery for slave devices.

# Hardware Required

Microcontrollers: STM32F407G-DISC1 (x2)
Sensors: HC-SR04 Ultrasonic Sensors
Display: LCD 1602 
Indicators: Green and Red LEDs
Connectivity:
     Breadboard and Jumper wires

# Pin Configuration

# Master Board
I2C Bus - PA8 - SCL 
I2C Bus - PC9 - SDA

# Slave Board 

I2C Bus - PA8 - I2C3 SCL 
I2C Bus - PC9 - I2C3 SDA 
Sensor 1 -  PA3 (Trig), PE9 (Echo) - Parking Spot 1 
Sensor 2 - PB1 (Trig), PE11 (Echo) - Parking Spot 2 
Sensor 3 - PC8 (Trig), PE13 (Echo) - Parking Spot 3 
LED group 1 - PB0(Green), PB12(Red) - Status Indicators 
LED group 2 - PB4(Green), PB5(Red) - Status Indicators 
LED group 3 - PB13(Green), PB14(Red) - Status Indicators 

# How It Works

1.  Distance Measurement:
    The Slave board uses TIM1 in Input Capture mode to measure the pulse width from the HC-SR04 Echo pin. The system calculates the distance:
    Distance (cm) = Pulse_Width (us) * 0.017
2.  Logic:
      If distance < 20cm: Spot is OCCUPIED (Red LED ON).
      If distance >= 20cm: Spot is FREE (Green LED ON).
3.  Communication:
    The Master board polls each Slave via I2C addresses (e.g., 0x30, 0x31) periodically. The Slaves respond with the number of free spots they detected.
4.  Display:
    The Master sums up the data and updates the LCD screen.

    <img width="1504" height="526" alt="image" src="https://github.com/user-attachments/assets/6f657d9a-3a33-4fd1-97e6-bb74f06ff645" />


# Software Setup

1.  IDE: STM32CubeIDE (Version 1.16.0 or newer).
2.  Firmware Library: STM32Cube FW_F4 V1.28.0.

# Installation Steps

1.  Open STM32CubeIDE and import the projects (Master and Slave are separate projects).
2.  For Slave Nodes:
     Open main.c in the Slave project.
     Set the unique I2C address in MX_I2C3_Init (e.g., 0x30 for Node 1, 0x31 for Node 2).
3.  Build and Flash the code to the respective boards.

Check the wiring of SDA/SCL and ensure Pull-up resistors are connected.
Verify TIM1 Prescaler is set to 167 and Input Capture Polarity is correct.

# Authors

Vitalina Posmashnaia 
