# CDAC_Project
RTOS-Based Safety Monitoring Helmet.
⛑️ RTOS-Based Smart Helmet for Real-Time Safety Monitoring
📌 Overview

This project is designed to enhance personal and worker safety by providing real-time accident detection and emergency alerts. It uses an STM32 microcontroller with RTOS to monitor motion and live location continuously and send alerts during emergencies.

⚙️ Features

Accident and fall detection using MPU6050

Live GPS location tracking (NEO-7M)

Emergency push-button for manual alerts

RTOS-based real-time task handling

Cloud monitoring using ThingsBoard

UART-based communication with ESP module

🛠️ Hardware Used

STM32 Microcontroller

MPU6050 (Accelerometer & Gyroscope)

NEO-7M GPS Module

ESP WiFi Module / ESP32

Emergency Push Button

🔧 Working

The MPU6050 monitors movement to detect accidents, while the GPS provides live location data. Sensor and GPS data are managed using RTOS and sent via UART to an ESP module, which uploads the information to the ThingsBoard cloud for real-time monitoring and alerts.

🚀 Applications

Smart helmets

Worker safety systems

Personal emergency alert systems
