# esp32pumpkin

This project is inspired by Bitluni’s *Screaming Pumpkin*, found [here on YouTube](https://www.youtube.com/watch?v=VfbikvNPwM0&t=330s). This pumpkin uses the ESP32 microcontroller to create an animated flame effect and plays audio when motion is detected.

To convert audio files to C headers, I used Bitluni’s utility, available [here](https://bitluni.net/wp-content/uploads/2018/01/Audio2Header.html).

---

## Table of Contents
- [Overview](#overview)
- [Features](#features)
- [Components](#components)
- [Hardware Setup](#hardware-setup)
- [Software Setup](#software-setup)
- [How It Works](#how-it-works)
- [Usage](#usage)
- [Credits](#credits)
- [License](#license)

---

## Overview
`esp32pumpkin` is a DIY Halloween project that turns a pumpkin into a spooky decoration with LED flickering flame effects and a motion-activated scream. It uses the ESP32’s Real-Time Operating System (FreeRTOS) to run multiple tasks, ensuring seamless audio playback and LED effects.

## Features
- **LED Flame Effect**: Mimics a realistic flickering flame using RGB LEDs.
- **Motion Detection**: Detects motion and triggers audio playback.
- **Audio Playback**: Plays a spooky sound effect when motion is detected.
- **Customizable Sound and Light Effects**: Easily modify the LED and audio effects.

## Components
- **ESP32** microcontroller (e.g., ESP32-S2)
- **LED Strip** (e.g., WS2812B or NeoPixel, 10 LEDs)
- **PIR Motion Sensor** (for motion detection)
- **Speaker and Amplifier** (for audio output)
- **Power Supply**: 5V power source compatible with the ESP32 and LEDs
- **Connecting Wires** and **Breadboard** (optional)

## Hardware Setup
Here’s the wiring setup for the components:

1. **LED Strip**: Connect the data pin of the LED strip to a GPIO pin on the ESP32 (e.g., GPIO 12). Power and ground connect to the ESP32’s 5V and GND.
2. **PIR Sensor**: Connect the PIR sensor’s output to another GPIO pin on the ESP32 (e.g., GPIO 14).
3. **Speaker and Amplifier**: Connect the amplifier’s input to the ESP32’s I2S output pins. Connect the speaker to the amplifier’s output.

### Pin Configuration Example
| Component       | ESP32 Pin       |
| --------------- | --------------- |
| LED Data        | GPIO 12         |
| PIR Sensor Out  | GPIO 14         |
| I2S (Audio Out) | GPIO 25, GPIO 26 (if using I2S) |
| GND             | GND             |
| VCC (LED Strip & PIR) | 5V (with external power if needed) |

## Software Setup
1. **Install ESP-IDF**: Follow the [official ESP-IDF setup guide](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/) to install the development framework.
2. **Clone the Project**: Download or clone this repository to your local machine.
3. **Configure Project**:
   ```bash
   idf.py menuconfig
