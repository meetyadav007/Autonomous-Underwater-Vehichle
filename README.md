# Autonomous Underwater Vehicle (AUV)

A six-thruster underwater vehicle built by a student team. A Raspberry Pi 4 does
the vision work and an Arduino Mega reads the sensors and drives the thrusters.
This repository holds the vehicle's code and its design report.

## What I worked on

I handled the sensor interfacing and the thruster communication: reading the
MPU6050 IMU and the MS5837 pressure sensor, and driving the six Blue Robotics
T200 thrusters through their ESCs from the Arduino. The code in this repository
is the Arduino gate-pass sketch and the Raspberry Pi vision script described below.

## Hardware

| Part | Used for |
|---|---|
| Raspberry Pi 4B (8 GB) | Vision and high-level logic |
| Arduino Mega 2560 | Sensors and thruster control |
| Raspberry Pi Camera Module 3 | Underwater vision |
| MPU6050 IMU | Orientation and yaw |
| MS5837-02BA pressure sensor | Depth |
| TF-Luna LiDAR | Distance to the wall in front, used by the gate sketch |
| 6 x Blue Robotics T200 thrusters + ESCs | Propulsion |
| Hydrophone + preamplifier | Acoustic pinger work |
| Li-ion battery + buck converter | Power |

The Pi is tethered to a laptop over Ethernet (see below for why).

## What is in this repository

| Path | What it is |
|---|---|
| `Software/finalcode.ino` | Arduino sketch for a gate mission. It arms the ESCs, measures the wall distance, sways sideways until the distance jumps (the gate's left and right edges), re-centres between them, drives forward through the gap, then turns 180 degrees using IMU yaw. |
| `Software/finalAUvprocessing.py` | Raspberry Pi script. It finds a ball in the camera image with a Hough circle transform and sends timed text commands (`DESCEND`, `FORWARD`, `BALL`, `PITCH_UP`, ...) over serial to the Arduino. |
| `Documentation/AUV_Report.docx` | Design report: mechanical design, Ansys studies, electronics, and the control and acoustic-localisation approach. |
| `Hardware/` | Placeholders for mechanical and electronics files. |

### Where things stand

- The gate sketch and the Pi script are not joined up yet. The sketch does not
  read serial commands, so the Pi script's commands are not acted on.
- The sketch's only closed-loop control is a simple proportional yaw hold from
  the gyro. The vertical thrusters are not used by it.
- The report was written at the design stage, in the future tense. A few details
  in it differ from what we finally built (for example it lists a USB camera and
  a four-hydrophone array, while the vehicle used the Pi Camera 3 and a single
  hydrophone).
- The GCC-PHAT / TDOA pinger-localisation method and the thruster allocation
  matrix are described in the report. They are design work, not code in this
  repository.

## What went wrong in the water

On paper this looked simple. On the bench everything behaved. In the water,
almost nothing did:

- **The Pi kept dropping its wireless link.** We stopped fighting it and ran an
  Ethernet cable to the vehicle.
- **Parts overheated inside the enclosure.** We added fans.
- **Some of the low-cost ESCs burned out.** We had picked them to stay within
  budget.
- **Getting all six thrusters to push the right way took longer than anything
  else.** Direction and mapping mistakes only showed up once the vehicle was
  actually in the water.

The rule I took from this: if it has only worked on the bench, it hasn't
really worked yet.

## Running the code

**Arduino** (Mega 2560): install the `MPU6050` library (i2cdevlib); `Wire`,
`Servo` and `SoftwareSerial` ship with the IDE. Open `Software/finalcode.ino`
and upload. Serial monitor at 115200 baud.

**Raspberry Pi**: needs `picamera2`, `opencv-python`, `numpy` and `pyserial`.
The script expects the Arduino on `/dev/ttyACM0`.

```
python3 Software/finalAUvprocessing.py
```

## Repository layout

```text
Autonomous-Underwater-Vehicle/
├── README.md
├── Documentation/
│   └── AUV_Report.docx
├── Hardware/
│   ├── Electronics/
│   └── Mechanical/
└── Software/
    ├── finalAUvprocessing.py
    └── finalcode.ino
```
