Mecanum Car Project

Dieses Repository enthält alle Dateien und Dokumentationen zum Mecanum Car, das über zwei Semester (4. und 5. Semester) im Rahmen des Bachelorstudiengangs Mechatronik Design & Innovation entwickelt wurde. Ziel des Projekts war die Konstruktion, Umsetzung und Erprobung eines funktionsfähigen Mecanum Cars, das sowohl mechanischen als auch elektronischen Anforderungen gerecht wird.
Projektübersicht

Das Projektziel war die Entwicklung eines funktionalen Mecanum Cars mit omnidirektionaler Bewegungsfähigkeit. Der Fokus lag auf der mechanischen Konstruktion, der elektronischen Entwicklung und der Softwareintegration. Die Arbeit erstreckte sich über zwei Semester, um eine iterative Verbesserung und Erweiterung zu ermöglichen.
Hauptmerkmale

    Mechanische Konstruktion:
        Leichtes und stabiles Aluminiumchassis für hohe Belastbarkeit und Stabilität.
        Schutzgehäuse für alle elektronischen Komponenten.
        Verwendung von Mecanum-Rädern für omnidirektionale Bewegung.
    Elektronisches System:
        Spannungsversorgung über einen Makita-Akkuschrauberakku mit integriertem Spannungswandler.
        Steuerung durch einen STM32F401VET6-Mikrocontroller mit ausreichend Reserven für zukünftige Erweiterungen.
        Integration eines Gyro-Sensors (MPU6050) zur Erfassung von Beschleunigungsdaten.
        Kommunikationsmodule (XBee, Infrarot-Empfänger) für zukünftige Fernsteuerungsoptionen.
        Verkabelung der LEDs zur Beleuchtung vorne (weiß) und hinten (rot).
    Software:
        Steuerung mittels Xbox-Controller.
        Debugging-Funktionen mit Hardware-Tasten.
        Modular programmierte Grundfunktionen mit Erweiterungspotenzial.
    Erweiterungen:
        Design und Fertigung einer separaten Platine für einen Controller.
        Installation eines Lüfters zur Kühlung der Platine und der Motor-Treiber.
        Verkabelung und Montage von LEDs zur Fahrzeugbeleuchtung.

Repository-Inhalte

    Mechanische Designs: CAD-Dateien der Fahrzeugkomponenten (Grundplatte, Motorhalter, Stoßfänger, etc.).
    Elektronik: Schaltpläne und PCB-Layouts, erstellt mit Altium Designer.
    Firmware: Steuerungssoftware für das Mecanum Car, programmiert in C++ für den STM32-Mikrocontroller.
    Dokumentation: Abschlussberichte, technische Spezifikationen und Benutzeranleitungen.

Einstieg
Voraussetzungen

    Entwicklungsumgebung:
        Altium Designer für das PCB-Design.
        Visual Studio Code zur Firmware-Entwicklung.
    Hardware:
        Mecanum-Räder, Schrittmotoren, Makita-Akku, STM32-Mikrocontroller, Gyro-Sensor und diverse weitere Komponenten.

Installation

    Klonen Sie das Repository:

    git clone https://github.com/smariacher/mecanum_car.git

    Öffnen Sie die Projektdateien in den entsprechenden Software-Tools.
    Kompilieren Sie die Firmware mit der STM32-Entwicklungsumgebung.

Aktueller Status

    Mechanik: Das Gehäuse wurde vollständig umgesetzt und funktioniert einwandfrei.
    Elektronik: Die Platine und die Motorsteuerung sind vollständig funktional.
    Software: Die grundlegenden Bewegungsfunktionen sind implementiert; fortgeschrittene Funktionen befinden sich in Arbeit.

Zukünftige Arbeit

    Optimierung der Motor-Treiber für bessere Leistung.
    Integration eines Displays zur Darstellung von Sensordaten.
    Erweiterung der Automatisierungs- und Steuerungsfunktionen.

Mitwirkende

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
