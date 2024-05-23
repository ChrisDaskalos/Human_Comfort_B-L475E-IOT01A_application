# Sensor Data Acquisition and Activity Classification on STM32L475 IoT Node

This repository contains an example program to demonstrate sensor data acquisition, activity classification, and comfort data checking on the STM32L475 IoT node. The program utilizes accelerometer, gyroscope, temperature, and humidity sensors to classify activities such as running, walking, and detecting falls, as well as monitoring environmental comfort based on temperature and humidity.

## Table of Contents
- Features
- Hardware Requirements
- Software Requirements
- Getting Started
- Configuration
- Functionality Overview
- Usage
- Contributing
- License

## Features
- Accelerometer and gyroscope data acquisition
- Activity classification (running, walking, fall detection, idle, sleeping)
- Environmental comfort checking based on temperature and humidity
- Optional data encryption using AES for secure data transmission
- Configurable serial output for debugging and data visualization

## Hardware Requirements
- STM32L475 IoT node
- USB cable for programming and serial communication

## Software Requirements
- Mbed OS
- Mbed CLI or an IDE with Mbed support (e.g., Mbed Studio)
- Mbed TLS library for encryption (optional)

## Getting Started

### Clone the Repository
```sh
git clone https://github.com/yourusername/STM32L475-IoT-Sensor-Activity.git
cd STM32L475-IoT-Sensor-Activity
