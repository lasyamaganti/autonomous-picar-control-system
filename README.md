# Autonomous PiCar Control System

An autonomous vehicle project integrating **real-time sensor feedback, motor control, and computer vision** using a Raspberry Pi. Working with a teammate, I developed and tested a PiCar capable of regulating its speed, tracking visual targets, detecting obstacles, and navigating autonomously.

## How It Works

- Calculated wheel speed in real time using photoresistor sensor data
- Implemented PI/PID-based feedback control for speed regulation
- Used PWM to dynamically control motor speed
- Integrated ultrasonic sensing for obstacle detection and distance-based stopping
- Used OpenCV and HSV filtering to detect and navigate toward colored targets
- Evaluated system performance using response time, steady-state error, and FFT analysis

## Technologies & Concepts

**Python • Raspberry Pi • OpenCV • PID Control • Computer Vision • Sensor Integration • PWM • FFT • Autonomous Systems**

## What I Learned

This project gave me hands-on experience integrating **hardware, software, and feedback control into a complete autonomous system**. I gained experience with real-time sensor processing, control-system tuning, computer vision, and testing how individual components interact within a larger system.

## Project Report

The full report documents the system design, control methods, testing, and performance analysis. The original Raspberry Pi source code is no longer available, so this repository serves as documentation of the project.

## Future Improvements

- Incorporate derivative control to improve system response
- Improve computer vision and edge detection for navigation
- Increase robustness under changing lighting and environmental conditions
