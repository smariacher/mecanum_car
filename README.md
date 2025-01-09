Mecanum Car Project

This repository contains all files and documentation related to the Mecanum Car, developed over two semesters (4th and 5th semester) as part of the Bachelor's degree program in Mechatronics Design & Innovation. The goal of the project was to design, implement, and test a fully functional Mecanum Car that meets both mechanical and electronic requirements.
Project Overview

The project's objective was to develop a functional Mecanum Car with omnidirectional movement capabilities. The focus was on mechanical design, electronic development, and software integration. The work spanned two semesters to allow iterative improvement and extension.
Key Features
Mechanical Design

    Lightweight and robust aluminum chassis for high durability and stability.
    Protective housing for all electronic components.
    Use of Mecanum wheels for omnidirectional movement.

Electronic System

    Power supply via a Makita power drill battery with an integrated voltage converter.
    Control unit based on an STM32F401VET6 microcontroller, offering sufficient reserves for future expansions.
    Integration of a gyro sensor (MPU6050) for acceleration data acquisition.
    Communication modules (XBee, infrared receiver) for future remote control options.
    Wiring of LEDs for front (white) and rear (red) lighting.

Software

    Control via Xbox controller.
    Debugging functionalities through hardware buttons.
    Modular programming of basic functions with potential for further extension.

Extensions

    Design and production of a custom controller PCB.
    Installation of a fan for cooling the PCB and motor drivers.
    Wiring and mounting of LEDs for vehicle lighting.

Repository Contents

    Mechanical Designs: CAD files of vehicle components (base plate, motor mounts, bumpers, etc.).
    Electronics: Schematics and PCB layouts created with Altium Designer.
    Firmware: Control software for the Mecanum Car, written in C++ for the STM32 microcontroller.
    Documentation: Final reports, technical specifications, and user manuals.

Getting Started
Prerequisites

    Development Environment:
        Altium Designer for PCB design.
        Visual Studio Code for firmware development.
    Hardware:
        Mecanum wheels, stepper motors, Makita battery, STM32 microcontroller, gyro sensor, and other components.

Installation

    Clone the repository:

    git clone https://github.com/smariacher/mecanum_car.git

    Open the project files in the corresponding software tools.

    Compile the firmware using the STM32 development environment.

Current Status

    Mechanics: The chassis has been fully implemented and functions flawlessly.
    Electronics: The PCB and motor control systems are fully operational.
    Software: Basic motion functionalities have been implemented; advanced features are under development.

Future Work

    Optimization of motor drivers for improved performance.
    Integration of a display for sensor data visualization.
    Expansion of automation and control functionalities.

Contributors

    Julian Fritzer
    Patrick Monthaler
    Simon Mariacher


## Source code
#### General Overview
Inside the `source_code` folder you can find the main code, the python controller and the example folder. The main code is meant to be flashed to the car and is the current working version. The python controller can be used to control the car via a computer using for example an Xbox-Controller or just keyboard inputs. Inside the `examples` folder you can find examples for XBEE-connection to the car, driving the stepper motors and more. There is also the old source code 

#### Main source code
The main source code has been thoroughly documented and can be flashed to the car via the exposed USB-B port on the side. For this you will need `Platform IO` in `Visual Studio Code`. 

#### Python Controller
To use the python controller, you will need to connect an XBEE via USB to your PC and configure the script to use the right COM port. You will also need to install `pygame`. When connecting a controller the code will automatically detect it when starting up and use the controller as input. When no controller is connected it will use the keyboard buttons WASD+EQ for movement and f+b for front and backlight respectively. The space bar is set to be used as an emergency stop function, where all motors will be disabled (meaning the will move freely) so that the car comes to a smooth stop. After emergency stopping you have to reset the car via the connected external button. Using a controller the car is moved through both analog sticks, and the A-Button is used as emergency stop (when using a XBOX-Controller).

#### Wireless protocol
The car's controller listens to UART2 which is connected to the XBEE. On receiving a message an interrupt function is called, which either changes the wheel speeds, toggles front or backlight or intitiates emergency stop. The message protocol to controll the wheel speeds is simple and consists of 20 characters, 5 for each wheel where the step frequency for the wheel is defined. As an example you can send a message using a tool like `PuTTY` and send `01000010000100001000` which will move every wheel at 1000 speed. Of course this protocol is limited to a max speed of 99999 or -9999. This should be upgraded later on.
