─── README.md START ───
# 🤖 HERMES — Autonomous Secure Document Delivery Robot


> Air-gapped | LiDAR-SLAM | RFID-Authenticated | Offline | ROS 2


![Status](https://img.shields.io/badge/Status-In%20Development-orange)
![ROS2](https://img.shields.io/badge/ROS2-Humble%20Hawksbill-blue)
![License](https://img.shields.io/badge/License-MIT-green)


## 📌 Project Overview


HERMES is a fully autonomous mobile robot designed and built as a final-year
Mechatronics Engineering project at Ahmadu Bello University Zaria, Nigeria.


It solves a real institutional problem: the manual transport of sensitive
paper documents (exam papers, HR files, legal contracts) through open
corridors by human staff — a process that is both inefficient and insecure.


**What makes HERMES different from commercial AMRs:**
Commercial delivery robots depend on cloud APIs and Wi-Fi for navigation.
These can be remotely hijacked. HERMES operates with ZERO wireless
connectivity — fully air-gapped. Navigation is offline (LiDAR + SLAM).
Vault access requires a physical RFID card. If power fails, the vault stays
locked (fail-secure solenoid deadbolt).


## 🏗️ System Architecture


| Layer         | Technology                      | Hardware          |
|---------------|---------------------------------|-------------------|
| Navigation    | ROS 2 + SLAM Toolbox + Nav2     | Raspberry Pi 4    |
| Motor Control | PID + Differential Drive        | Arduino Mega 2560 |
| Mapping       | 2D LiDAR (360°)                 | RPLIDAR A1        |
| Security      | RFID Auth + Solenoid Vault      | RC522 + 12V Lock  |
| Vision        | MobileNet-SSD (TensorFlow Lite) | USB Webcam        |
| Power         | 12V 10Ah Li-Ion + Buck Conv.    | LM2596 Modules    |


## 🔐 Security Philosophy


- **Air-gapped**: No Wi-Fi, no Bluetooth, no cloud connection
- **Fail-secure**: Vault stays locked on power failure or crash
- **RFID-only access**: Payload unlocks ONLY on authorized badge scan
- **Offline dispatch**: Destinations entered via onboard physical keypad
- **Audit trail**: Vision node logs all human interactions during transit


## 📦 Repository Structure


```
hermes-amr-robot/
├── docs/          # Research proposal, architecture docs, BOM
├── firmware/      # Arduino Mega C++ firmware (PID, RFID, solenoid)
├── ros2_ws/       # ROS 2 workspace (SLAM, Nav2, vision node)
├── hardware/      # Chassis specs, wiring diagrams, power calculations
├── scripts/       # Environment setup and simulation launch scripts
└── media/         # Photos, diagrams, demo videos
```


## 🛠️ Hardware Bill of Materials


| Component           | Model                | Role                    |
|---------------------|----------------------|-------------------------|
| SBC                 | Raspberry Pi 4 (4GB) | ROS 2 / SLAM / Vision   |
| Microcontroller     | Arduino Mega 2560    | PID / RFID / Solenoid   |
| LiDAR               | SLAMTEC RPLIDAR A1   | 360° Mapping            |
| Motor Driver        | BTS7960 43A H-Bridge | Drive Control           |
| Authentication      | RC522 RFID + Keypad  | Vault Access Control    |
| Locking Mechanism   | 12V Solenoid + MOSFET| Fail-Secure Vault       |
| Power               | 12V 10Ah Li-Ion      | System Power            |


See [docs/component_list.md](docs/component_list.md) for full BOM with prices.


## 🗺️ Operational States


0. **Idle** — Docked at charging station
1. **Secure Dispatch** — Sender loads document, vault locks
2. **Air-Gapped Assignment** — Destination entered via keypad
3. **Autonomous Transit** — LiDAR SLAM + DWA obstacle avoidance
4. **Arrival & Prompt** — LCD prompts recipient for RFID badge
5. **Authentication & Release** — RFID verified, vault unlocks 10 sec
6. **Return Home** — Robot navigates back to dispatch station


## 🚀 Getting Started


### Prerequisites
- Ubuntu 22.04 LTS
- ROS 2 Humble Hawksbill
- Python 3.10+, colcon, slam_toolbox, Nav2, TensorFlow Lite


### Software Setup
```bash
git clone https://github.com/[your-username]/hermes-amr-robot.git
cd hermes-amr-robot
bash scripts/setup_environment.sh
cd ros2_ws && colcon build
source install/setup.bash
```


### Run Simulation
```bash
bash scripts/run_simulation.sh
```


## 📄 Research Proposal


The full 28-page academic proposal is available in [docs/HERMES_Proposal.pdf](docs/HERMES_Proposal.pdf).
Supervised by Dr. H.I. Hassan, Dept. of Mechanical Engineering, ABU Zaria.


## 🗓️ Project Timeline


| Phase                              | Weeks   | Status      |
|------------------------------------|---------|-------------|
| Literature Review & Proposal       | 1–2     | ✅ Complete |
| Component Procurement              | 3–4     | ⏳ Pending  |
| Hardware Assembly & Wiring         | 4–5     | ⏳ Pending  |
| ROS 2 / SLAM / Nav2 Config         | 6–7     | ⏳ Pending  |
| RFID, Keypad, Vision & PID Coding  | 8–9     | ⏳ Pending  |
| System Testing & Office Trials     | 10–12   | ⏳ Pending  |


## 👤 Author


**Muhammad Salman Oluwatoyin**  
Final Year Mechatronics Engineering Student  
Ahmadu Bello University, Zaria — U19MN1004  
Supervisor: Dr. H.I. Hassan  
[LinkedIn](https://linkedin.com/in/[your-profile])


## 📜 License


MIT License — see [LICENSE](LICENSE) for details.
─── README.md END ───
