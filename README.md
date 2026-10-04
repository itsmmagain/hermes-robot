# Project HERMES

A low-cost autonomous mobile robot that delivers documents between two offices along a marked line, with visual destination verification and RFID-controlled cargo access. Built on a single ESP32 as a final-year Mechatronics Engineering project at Ahmadu Bello University, Zaria, Nigeria.

## Features

- **Line following** with a HuskyLens AI camera, using a cascaded controller: an outer PD vision loop (about 30 ms) sets left and right wheel speed targets, and an inner PI encoder loop (50 ms) with feed-forward tracks them.
- **Visual destination verification.** When the line is lost for more than 10 frames, the camera switches from Line Tracking to Tag Recognition and looks for the AprilTag of the commanded destination (3.5 s timeout, then back to line tracking).
- **Location-bound RFID security.** The cargo compartment (solenoid lock) opens only for a valid tag scanned at the right station. Authentication runs locally on the ESP32 and does not depend on Wi-Fi.
- **Obstacle stop** with an HC-SR04 ultrasonic sensor (30 cm threshold).
- **Telegram control and notifications** (`/send_to_exam`, `/send_to_hod`, `/status`, `/menu`).
- **OTA updates** with a safety interlock that forces the motors off while flashing.
- Eight-state finite state machine; Telegram polling is paused during transit so network latency does not disturb the control loop.

## Hardware

| Part | Model |
|---|---|
| Controller | ESP32 dev board with 30-pin expansion board |
| Vision | HuskyLens (Kendryte K210) |
| Obstacle sensor | HC-SR04 |
| RFID | RC522 (13.56 MHz) with two station tags |
| Motor driver | L298N |
| Motors | TT geared motors, 1:90, Hall encoders (12 PPR), 65 mm wheels |
| Lock | 12 V normally-closed solenoid with 1-channel relay |
| Power | 3S Li-ion pack with BMS, LM2596 buck converter |

### Pin map (from the firmware)

| Function | GPIO |
|---|---|
| L298N ENA / IN1 / IN2 | 13 / 12 / 14 |
| L298N IN3 / IN4 / ENB | 27 / 26 / 25 |
| RC522 SDA (SS) / RST | 5 / 22 |
| Solenoid relay | 32 |
| Buzzer | 4 |
| Ultrasonic TRIG / ECHO | 33 / 35 |
| Encoders (left A / right A) | 34 / 36 |
| HuskyLens UART (Serial2 RX / TX) | 16 / 17 |

The HuskyLens must have AprilTag IDs 1 (Exam Office) and 2 (HOD Office) learned.

## Getting started

1. Install the Arduino IDE and the ESP32 board package.
2. Install these libraries: HUSKYLENS, MFRC522, UniversalTelegramBot, ArduinoJson (ArduinoOTA, WiFi and SPI come with the ESP32 package).
3. In `firmware/hermes_firmware/`, copy `secrets.example.h` to `secrets.h` and fill in your Wi-Fi details, Telegram bot token and chat IDs, and the UIDs of your two RFID tags. Scan each tag once with the serial monitor open at 115200 baud to read its UID.
4. Open `hermes_firmware.ino`, select your ESP32 board, and upload.

`secrets.h` is git-ignored. Never commit real credentials.

## Results (15-run benchmark)

| Function | Success |
|---|---|
| Loading authentication | 15 / 15 |
| Delivery notification | 15 / 15 |
| Line-following completion | 14 / 15 |
| Obstacle avoidance | 14 / 15 |
| Destination verification | 14 / 15 |
| Retrieval authentication | 14 / 15 |

Mean delivery time over the 14 comparable runs: 55.8 s (SD 11.5 s). Component cost: ₦245,000. Three anomalies were recorded and are discussed in the thesis. One route, two endpoints, and fifteen runs only, so treat these as a baseline, not a reliability guarantee.

## Documentation

The full design, calculations, and test results are described in the project thesis.

## License

MIT. See `LICENSE`.

## Author

Muhammad Salman Oluwatoyin, Department of Mechatronics Engineering, Ahmadu Bello University, Zaria.
