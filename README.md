# AI-Based Smart Respiratory Monitoring and Assistance System for COPD Patients

A university engineering prototype that monitors SpO₂, body temperature,
and respiratory rate while providing automated respiratory-assistance actions.

> [!IMPORTANT]
> This is an educational prototype and is not a certified medical device.
> It must not be used for diagnosis, treatment, or real patient care.

## Features

- MAX30102-based SpO₂ monitoring
- DS18B20 temperature monitoring
- Piezoelectric respiratory sensor with TL071 signal conditioning
- NI USB-6001 and LabVIEW respiratory-signal acquisition
- Automatic air-pump relay control
- L298N-controlled bed-elevation motor
- ESP32-hosted real-time web dashboard
- AI-assisted COPD exacerbation risk assessment
- LabVIEW-to-ESP32 actuator communication
- Communication timeout and actuator fail-safe logic

## System Architecture

MAX30102 + DS18B20 → ESP32 → Website / LabVIEW  
Piezo Sensor → TL071 Filter → NI DAQ → LabVIEW  
LabVIEW → ESP32 → Relay + L298N Motor Driver

## Hardware

- ESP32 development board
- MAX30102 pulse-oximeter sensor
- DS18B20 temperature sensor
- Piezoelectric disc
- TL071CP signal-conditioning circuit
- NI USB-6001 DAQ
- Two-channel relay module
- DC air pump
- L298N motor driver
- 12 V DC gear motor

## Software

- Arduino IDE
- LabVIEW
- NI-DAQmx
- HTML, CSS and JavaScript
- ESP32 Wi-Fi libraries
- FastAPI/Python, where applicable

## Repository Structure

- `firmware/` – ESP32 source code
- `labview/` – LabVIEW VI files
- `dashboard/` – Web dashboard
- `docs/` – Project proposal, diagrams and documentation
- `images/` – Hardware and dashboard screenshots

## ESP32 Pin Connections

| Function | ESP32 pin |
|---|---:|
| MAX30102 SDA | GPIO 21 |
| MAX30102 SCL | GPIO 22 |
| DS18B20 data | GPIO 4 |
| Relay IN1 | GPIO 5 |
| L298N IN3 | GPIO 18 |
| L298N IN4 | GPIO 19 |
| L298N ENB | GPIO 23 |

## Serial Communication

ESP32 sends:

`SpO2,Temperature`

Example:

`96,36.75`

LabVIEW sends:

`RelayCommand,MotorCommand`

Example:

`1,0`

## Authors
Danul Renuja Palliyaguru

## Disclaimer

This project is intended solely for education, research and demonstration.
Clinical thresholds and AI-generated assessments must not replace qualified
medical advice.
