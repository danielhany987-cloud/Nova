# Nova — Open-Source Desktop Humanoid Robot

Nova is a free and open-source desktop humanoid robot designed to make robotics more affordable and accessible for students, makers, and educators around the world.

---

## Why Nova Was Created

Robotics is one of the most exciting fields in the world — but the cost and complexity of existing platforms put it out of reach for most people.

I started building Nova at **15 years old** and completed it at **16**. My goal was simple: build a real humanoid robot that almost anyone can afford to make, using parts that are easy to find, software that is completely free, and a design that anyone can learn from.

Nova is completely free, not sold commercially, and open to anyone who wants to build, modify, or learn from it — under the MIT License.

---

## What You Will Learn

Building Nova covers a wide range of real engineering skills:

- Robotics fundamentals
- Electronics and wiring
- Servo motor control
- ESP32 programming (Arduino IDE)
- ESP32-CAM computer vision
- AI-assisted robotics (YOLOv4-tiny object detection)
- CAD and mechanical design
- 3D printing

> **How the AI works:** Object detection runs on a standard computer using Python and OpenCV. The ESP32 handles robot control separately. This split architecture keeps the hardware cost low — no expensive GPU or dedicated AI chip required.

---

## Main Features

- **Joystick-controlled robot arm** — acceleration-limited velocity control with smooth servo motion
- **AI object detection** — YOLOv4-tiny running via OpenCV on a PC, streamed from the ESP32-CAM
- **Text-to-speech announcements** — Nova speaks detected object names aloud
- **Wave animation** — a pre-programmed greeting wave sequence
- **Servo center utility** — moves all servos to 90° for calibration and assembly
- **3D-printed body** — most structural parts are printed in PLA (~1063 g total)
- **Modular design** — build a single arm (~$150) or the full dual-arm version (~$300)

---

## Approximate Cost

| Version | Estimated Cost |
|---------|---------------|
| Single-Arm Nova | ~$150 USD |
| Full Dual-Arm Nova | ~$300 USD |

Prices vary by region and supplier. See the Bill of Materials for the full component list.

---

## Bill of Materials

| Component | Notes |
|-----------|-------|
| ESP32 | Main robot controller |
| ESP32-CAM | Camera module for AI vision |
| Analog Joystick Module | Arm control input |
| 1 x 35KG Servo | |
| 2 x 25KG Servos | Four 25KG servos may also be used instead |
| 1 x 15KG Servo | |
| Standard DC Power Supply | 3D printer-style |
| Multiple Buck Converters | Configure voltage to match servo and electronics requirements |
| Small Bluetooth Speaker | |
| Jumper Wires | |
|Electrolytic Capacitor||
|Ceramic Disc Capacitor||
| Servo Mounting Screws | Included with the servos |
| Rear Cover Screws | |
| ~1063 g PLA Filament | For all 3D-printed structural parts |
| All 3D-Printed Parts | STL files included in the `/STL` folder |

> See [Bill_of_Materials.md](Bill_of_Materials.md) for the full component list.

---

## Folder Structure

```
nova-robot/
├── Arduino/
│   ├── Nova_Arm/            Main joystick arm controller (ESP32)
│   ├── Nova_Arm_Wave/       Wave animation sequence (ESP32)
│   ├── Nova_Arm_Center/     Servo center calibration utility (ESP32)
│   └── ESP32_CAM/           Camera streaming firmware (ESP32-CAM)
├── Python_AI/
│   └── main.py              AI object detection + speech (runs on PC)
├── STL/                     All 3D-printable parts
├── Bill_of_Materials.md
├── README.md
├── .gitignore
└── LICENSE
```

---

## YouTube Build Guide

The complete build guide is on YouTube, including:

- 3D printing all parts
- Wiring and electronics
- ESP32 programming
- ESP32-CAM setup
- AI software setup
- Assembly
- Calibration
- Final demonstration

**Watch the full build guide: https://youtube.com/@danielhany-o4g?si=fyO8ZAEhHYOVosV6**

> Assembly instructions, wiring diagrams, and setup steps are covered in the video rather than in this repository.

---

## License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

You are free to use, copy, modify, and distribute this project for any purpose, including personal, educational, and commercial use, as long as the original license notice is included.

---

## Credits

Designed and built by **Daniel Hany**.

Started at age 15 — completed at age 16.

If you build Nova, share it. The goal is to make robotics accessible to everyone.
