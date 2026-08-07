# Bill of Materials — Nova Robot

This document lists all components required to build Nova.

> **Estimated Cost**
> - Single-Arm Nova: ~$150 USD
> - Full Dual-Arm Nova: ~$300 USD
>
> Prices vary by region and supplier.

---

## Electronics

| Component | Quantity | Notes |
|-----------|----------|-------|
| ESP32 | 1 | Main robot controller |
| ESP32-CAM | 1 | Camera module for AI vision |
| Analog Joystick Module | 1 | Arm control input |

---

## Servo Motors

| Component | Quantity | Notes |
|-----------|----------|-------|
| 35KG Servo | 1 | |
| 25KG Servo | 2 | Four 25KG servos may also be used instead of the 35KG + 25KG combination |
| 15KG Servo | 1 | |

> **Note:** Four 25KG servos may be substituted as an alternative configuration.
>
> **Not required:** Heat-set inserts, PCA9685, or any external servo driver board.

---

## Power

| Component | Quantity | Notes |
|-----------|----------|-------|
| DC Power Supply | 1 | Standard 3D printer-style power supply |
| Buck Converters | Multiple | Configure output voltage to match the requirements of each servo and electronic component |

---

## Hardware & Accessories

| Component | Quantity | Notes |
|-----------|----------|-------|
| Small Bluetooth Speaker | 1 | For text-to-speech audio output |
| Jumper Wires | As needed | |
| Servo Mounting Screws | As needed | Typically included with the servos |
| Rear Cover Screws | As needed | |

---

## 3D Printing

| Component | Quantity | Notes |
|-----------|----------|-------|
| PLA Filament | ~1063 g | For all structural parts |
| All 3D-Printed Parts | 1 set | STL files are in the `/STL` folder |

All structural parts are designed to be printed on a standard FDM 3D printer.
See the `/STL` folder for all printable files.

---

## STL Files Included

| File | Description |
|------|-------------|
| nova head.stl | Head shell |
| nova head cover .stl | Head cover |
| nova neck.stl | Neck joint |
| nova body (1).stl | Main body |
| nova body cover  .stl | Body rear cover |
| arm (2).stl | Arm link |
| claw hand (3).stl | Hand / claw |
| claw still.stl | Static claw part |
| clawpart.stl | Claw mechanism part |
| controller.stl | Controller enclosure |
| controler cover.stl | Controller cover |

---

> For build instructions, wiring, and assembly, see the full YouTube build guide:
> **https://youtube.com/@danielhany-o4g?si=fyO8ZAEhHYOVosV6**
