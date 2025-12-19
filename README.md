# PIC Tetris 2 You 🎄 - PIC18F4520

Two-player Tetris system built on the **PIC18F4520** microcontroller. Integrating multiple peripheral modules, including SPI, UART, PWM, ADC, and Timers, to create a fully functional gaming experience complete with custom-built mechanical controllers and an alcohol-sensing mode.    
Keeping the Christmas spirit alive!

![Picture1](https://github.com/user-attachments/assets/80c694cb-18d3-402a-bd22-c251a2808f7b)

Demo Video (In Traditional Chinese): [YouTube Demo Link](https://www.youtube.com/watch?v=6w2MI4Ak4ps)

## Features

* **Real-time Dual Battle:** Two PIC18F4520 units communicate via **UART**. Clearing lines on one device sends "garbage lines" to the opponent.
* **Master/Slave Synchronization:** A Master-Slave architecture ensures both games start simultaneously with synchronized scrolling titles.
* **Custom Hardware Interface:** - Dual MAX7219 8x8 LED Dot Matrix displays per player (16x8 total resolution).
* Custom 3D-printed controllers with Gateron Yellow mechanical switches.


* **Dynamic Audio:** - Integrated buzzer playing the classic Tetris Theme and Christmas carols.
* Volume control via potentiometer.


* **Alcohol Sensing Mode:** Features an MQ-3 Alcohol Sensor that triggers after every Tetris game ends, displaying alcohol concentration levels (LOW/MID/HIGH) on the LED matrix.

## Hardware Components
* **Microcontroller:** 2x Microchip PIC18F4520
* **Display:** 4x MAX7219 8x8 LED Dot Matrix Modules
* **Sensor:** MQ-3 Alcohol Gas Sensor
* **Audio:** Passive Buzzer + Potentiometer (for volume)
* **Inputs:** 5x Gateron Yellow Mechanical Switches (per player)
* **Chassis:** 3D Printed PLA Case (Designed in SolidWorks)

## System Architecture

### 1. Communication (UART)

* **Master:** Handles game mode switching, syncs start signals ('S'), and sends scrolling text offsets.
* **Interaction:** Both units send characters ('1'-'4') to trigger garbage lines on the opposing screen.

### 2. Display (SPI)

* High-speed communication with MAX7219 modules to refresh the 16x8 game board.

### 3. Audio (PWM & Timers)

* **Timer0:** Controls note duration.
* **PWM (Timer2):** Controls frequency (pitch) to play polyphonic-like melodies.

### 4. Alcohol Detection (ADC)

* Reads analog voltage from RA0 to determine intoxication levels, mapped to 8 visual levels on the LED matrix.

## Controls

The custom gamepad is mapped to **PORTB (RB0 - RB4)**:

* **RB0:** Rotate Clockwise
* **RB1:** Rotate Counter-Clockwise
* **RB2:** Move Right
* **RB3:** Move Left
* **RB4:** Hold / Start Game (Master)

## Development Environment

* **IDE:** MPLAB X IDE
* **Language:** C (XC8 Compiler)
* **Design Tools:** SolidWorks 2018 (3D Modeling)
* **Hardware:** Bambu Lab A1 3D Printer


---
> Created as a Final Project for the Microcomputer Systems course.    
> By 石維廉、曾立呈、蔡宗賢、石浚邑

**Merry Christmas!** 🎅🧤
