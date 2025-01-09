Mecanum Car Project

This repository contains all files and documentation related to the Mecanum Car, developed as the final project for the Bachelor's program in Mechatronics Design & Innovation (5th semester). The project focuses on the design, implementation, and testing of a functional Mecanum Car, meeting mechanical and electronic requirements.
Project Overview

The goal of this project was to create a functional Mecanum Car, featuring omnidirectional movement capabilities. The project involved mechanical design, electronic development, and software programming, completed within the constraints of a single semester.
Key Features

    Mechanical Design:
        Lightweight aluminum chassis for durability and stability.
        Protective housing for all electronic components.
        Mecanum wheels for omnidirectional movement.
    Electronic System:
        Powered by a Makita drill battery.
        Controlled by an STM32F401VET6 microcontroller.
        Equipped with a gyro sensor (MPU6050) for acceleration measurements.
        Communication modules including XBee and IR receiver.
    Software:
        Support for Xbox controller-based steering.
        Debug functions integrated into hardware buttons.
        Modular programming for future enhancements.

Repository Contents

    Hardware Design: CAD files for the car's mechanical components (base plate, motor mounts, bumpers, etc.).
    Electronics: Circuit schematics and PCB layouts designed using Altium Designer.
    Firmware: Code for controlling the Mecanum Car, written in C++ and compatible with STM32 microcontrollers.
    Documentation: Detailed reports, technical specifications, and user guides.

Getting Started
Prerequisites

    Development Tools:
        Altium Designer for PCB design.
        Visual Studio Code for firmware development.
    Hardware:
        Mecanum wheels, stepper motors, Makita battery, STM32 microcontroller, and various sensors.

Installation

    Clone the repository:

    git clone https://github.com/smariacher/mecanum_car.git

    Open the project files in the respective software tools.
    Compile the firmware using the STM32 development environment.

Current Status

    Mechanics: Fully implemented and functional.
    Electronics: PCB assembled and operational.
    Software: Basic movement implemented; advanced features in progress.

Future Work

    Enhance motor drivers for improved power delivery.
    Add advanced features such as a display for sensor data and further automation.
    Refine programming and debugging functionalities.

Contributors

    Julian Fritzer
    Patrick Monthaler
    Simon Mariacher
