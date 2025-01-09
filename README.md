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
