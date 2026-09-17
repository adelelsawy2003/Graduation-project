# Autonomous Target Detection & Laser Pointing Robot

Graduation Project — Mechatronics, Robotics & Automation Engineering, Tanta University.

A mobile robot that detects a colored target with an onboard camera and points a laser at it by driving a 4-DOF robotic arm through closed-loop visual servoing. The platform also carries gas and magnetic sensors for environment monitoring.

## System architecture

The system runs on three independent controllers:

| Module | Hardware | Role |
| --- | --- | --- |
| `raspberry_pi/` | Raspberry Pi 4 + IMX219 camera | Runs YOLO11n on NCNN, detects and verifies the target, computes the pixel error vector and streams it over UART |
| `robot_arm/` | Arduino + 4x servo + laser module | Receives joint angles over serial, moves the servos gradually, drives the laser, and falls back to a safe pose on signal loss |
| `mobile_base/` | Arduino Mega + 4x DC motors (mecanum) | Bluetooth-driven mecanum platform with gas sensor, magnetic sensor, buzzer and LED alarms |

Data flow: camera → Raspberry Pi (detection, HSV verification, error vector) → serial → arm controller (joint angles → servos + laser).

## Detection pipeline (Raspberry Pi)

1. Capture a 640x480 BGR frame from the IMX219 through Picamera2.
2. Resize to 512x512, normalize, and run an NCNN forward pass on YOLO11n.
3. Decode the raw candidate tensor and filter by confidence threshold.
4. Apply IoU-based Non-Maximum Suppression to remove overlapping boxes.
5. Scale the surviving boxes back to the original frame resolution.
6. Run a second-stage HSV verification — any box with less than 15% pink pixels is rejected. This deterministic check eliminates false positives that pass the neural network.
7. Require the target in 3 consecutive frames before locking, which suppresses single-frame jitter.
8. Compute the X and Y error relative to the frame center and transmit them.
9. On target loss, issue a STOP command to the actuators.

Run with:

```bash
python3 target_detection.py --conf 0.75
```

`model.ncnn.param` and `model.ncnn.bin` must be present in the working directory. Dependencies: ncnn, opencv-python, numpy, pyserial, picamera2.

## Arm controller

Parses comma-separated joint angles from serial, maps them into servo space with `constrain()`, and steps each servo one degree every 80 ms. Stepping this way keeps the motion smooth without ever calling a blocking `delay()`, so serial reception is never starved.

A 2-second watchdog guards the arm: if the serial stream stops, the joints return to their neutral pose and the laser switches off.

## Mobile base

Mecanum drive with six motion primitives — forward, backward, strafe left/right, and rotate left/right — selected by single-character Bluetooth commands (F, B, L, R, A, D, S). The gas sensor drives both the buzzer and a status LED; the magnetic sensor drives the buzzer.

## Hardware

- Raspberry Pi 4
- IMX219 camera module
- Arduino Mega 2560 — mobile base
- Arduino — robot arm controller
- 4x DC motors with mecanum wheels
- 4x servo motors
- Laser module
- Gas sensor, magnetic sensor, buzzer, LED
- Bluetooth module

## Repository layout

```
.
├── raspberry_pi/
│   └── target_detection.py     # Vision pipeline and serial transmission
├── robot_arm/
│   └── robot_arm.ino           # 4-DOF servo arm + laser + watchdog
└── mobile_base/
    └── mobile_base.ino         # Mecanum drive + sensor alarms
```

Each .ino file sits in a folder matching its name so the Arduino IDE opens it directly.

Comments inside the Arduino sketches are in Arabic, as written during development.
