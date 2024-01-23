# Thermometer Final Project

## Project Overview
The Thermometer project is implemented in C for an AVR microcontroller. It utilizes various modules and peripherals to create a functional thermometer with additional features.

## Features
1. Temperature Sensing: The project uses the DS18B20 temperature sensor to measure ambient temperature.
2. User Interface: The system includes a user interface with local and remote display options, controlled by buttons.
3. LED Indicators: LEDs indicate the temperature status (cool, warm, hot) and a buzzer provides additional feedback.
4. Serial Communication: The project transmits temperature values over USART to an external device.
5. Rotary Encoder: A rotary encoder is used to adjust threshold values for temperature control.

## Hardware Components
- **Microcontroller:** AVR
- **Temperature Sensor:** DS18B20
- **Display:** LCD
- **LEDs:** Red and Green LEDs
- **Buzzer:** Audible feedback
- **Rotary Encoder:** Interface for adjusting threshold values

## Setup and Initialization
1. **Inputs Initialization:**
   - Temperature sensor and encoder pins are configured as inputs with pull-up resistors enabled.
   - Buttons for local and remote display are also configured with pull-up resistors.

2. **Outputs Initialization:**
   - PWM output for servo motor control.
   - LEDs and buzzer outputs for indicating temperature status.
   - USART module for serial communication.

3. **USART Configuration:**
   - Baud rate, data bits, and other USART configurations are set.

4. **Timers Initialization:**
   - Timer modules control the buzzer, LED flashing, and PWM/servo motor.

5. **Interrupts:**
   - Pin change interrupts are used for button presses.
   - Timer and USART interrupts handle specific functionalities.

6. **Peripheral Initialization:**
   - LCD and DS18B20 sensor are initialized.

## Operation
1. The LCD displays an initial splash screen with project information.
2. Rotary encoder values and state are determined for initial setup.
3. Temperature conversion is initiated, and initial EEPROM values are read.
4. The main loop continuously:
   - Reads and converts temperature from the DS18B20 sensor.
   - Adjusts the servo motor based on temperature.
   - Transmits temperature values over USART.
   - Controls LEDs and buzzer based on temperature status.
   - Allows adjusting threshold values with the rotary encoder.
   - Switches between local and remote display modes.

## Usage and Interaction
- **Buttons:**
  - Pressing the local button switches to local display mode.
  - Pressing the remote button switches to remote display mode.

- **Rotary Encoder:**
  - Adjusts threshold values for temperature control.

- **Serial Communication:**
  - Temperature values are transmitted over USART.
