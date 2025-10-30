# SafeStep

SafeStep is an ESP32-based assistive stick that reads GPS (TinyGPS++), an MPU6050 IMU for fall detection, an ultrasonic sensor for proximity vibration feedback, controls a DFPlayer MP3 module, writes logs to an SD card, and can send SMS via a SIM module. It also has an camera mounted in the front to detect currency denominations and announce to user with the help of AI.

## Main sketches:
- [com_v2.ino](./com_v2.ino) - primary SafeStep sketch (GPS, MPU6050, ultrasonic, SD, SIM).
- [aiCodeWithMP3.ino](./aiCodeWithMP3.ino) - alternate sketch using ESP32-CAM + Edge Impulse FOMO with DFPlayer audio playback.

## Contents
- [com_v2.ino](./com_v2.ino)
- [aiCodeWithMP3.ino](./aiCodeWithMP3.ino)
- [Smart_Stick_v2_inferencing.h](./Smart_Stick_v2_inferencing/src/Smart_Stick_v2_inferencing.h)

## Hardware (high level)
#### Components:
- ESP32 development board
- GPS module
- MPU6050
- HC-SR04 ultrasonic sensor
- Vibration motor
- DFPlayer Mini MP3 module
- GPRS/SIM900 module
- microSD card module
- Push button
- ESP32-CAM

#### Pins used in sketches:
- Ultrasonic trig: 33, echo: 4
- Vibration: 32
- GPS Serial2 pins: 16 (RX), 17 (TX)
- MP3 serial: pins 13/14 in the camera sketch
- SIM serial: 25 (RX), 26 (TX)
- MPU6050 I2C: SDA 21, SCL 22
- SD CS: 5
- Button: 27

## Software prerequisites (Arduino IDE recommended)
1. Install the Arduino IDE and add ESP32 board support (Espressif Systems).
2. Install required libraries with Library Manager:
    - TinyGPSPlus
    - Adafruit MPU6050
    - DFRobotDFPlayerMini
    - TinyGSM
    - EloquentESP32cam
    - Put the [Smart_Stick_v2_inferencing Library](./Smart_Stick_v2_inferencing/) in Arduino libraries location
3. Open either [com_v2.ino](./com_v2.ino) or [aiCodeWithMP3.ino](./aiCodeWithMP3.ino) in the Arduino IDE, select the correct ESP32 board and COM port, then compile and upload.

## Wiring and configuration notes
- Identify which serial ports are available on your ESP32. The sketch uses:
  - `Serial2` for GPS with pins 16 (RX) and 17 (TX).
  - HardwareSerial(1) for MP3 (pins 13/14).
  - HardwareSerial(0) for SIM (pins 25/26 are used to initialize it).
  Adjust pins and HardwareSerial instances if using different pins/peripherals.
- SD card CS is set to GPIO 5. Change if your module uses another CS pin.
- Confirm voltage levels: SIM, GPS, MP3 and SD modules usually need 3.3V/5V compatibility.

## Running
1. Install libraries and select correct board.
2. Make any pin adjustments to match your wiring.
3. Open [com_v2.ino](./com_v2.ino) for ESP32 board and [aiCodeWithMP3.ino](./aiCodeWithMP3.ino) for ESP32 cam.
4. Compile and upload.
5. Connect power and ground.