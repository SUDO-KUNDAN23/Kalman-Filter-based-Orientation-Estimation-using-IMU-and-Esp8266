

# 🚀 Rocket Flight Controller using ESP8266, MPU6050, HMC5883L & BMP280 with PCA9685 Servo Control

This project implements a **rocket flight controller** using an **ESP8266** microcontroller with **MPU6050 (Accelerometer + Gyroscope)**, **HMC5883L Magnetometer**, and **BMP280 Barometric Sensor** for real-time **roll, pitch, yaw, and altitude estimation**. The system controls **servo motors via PCA9685** to enable **Thrust Vector Control (TVC)** for stable rocket flight.

---

## 📌 Features

* **Sensor Fusion (Kalman Filter)** → Combines MPU6050 + HMC5883L + BMP280 for accurate roll, pitch, yaw, and altitude estimation.
* **Thrust Vector Control (TVC)** → PCA9685 drives multiple servos for nozzle control.
* **Wireless Telemetry** → ESP8266 provides Wi-Fi-based telemetry (Web server / MQTT ready).
* **Real-Time Display** → Optional OLED (SSD1306) for onboard monitoring.
* **Expandable Design** → Can be integrated with GPS and LoRa for advanced telemetry.

---

## 🛠️ Hardware Requirements

* ESP8266 (NodeMCU / Wemos D1 Mini)
* MPU6050 (Accelerometer + Gyroscope)
* HMC5883L (Magnetometer)
* BMP280 (Barometric Pressure + Altitude Sensor)
* PCA9685 (16-Channel PWM Servo Driver)
* Servo Motors (for Fin control)
* Power Supply (12V, high current for servos)

---

## 🔌 Circuit Connections

```
ESP8266          Sensor/Driver
-------          ----------------
3V3      --->    VCC (MPU6050, HMC5883L, BMP280, PCA9685)
GND      --->    GND
D1 (SCL) --->    SCL (All I2C sensors + PCA9685)
D2 (SDA) --->    SDA (All I2C sensors + PCA9685)
VIN (5V) --->    V+ (Servo Power for PCA9685)
```

⚠️ **Important**: Servos must be powered separately with a common GND to avoid brownouts.


## 📥 Installation

1. Clone this repository:

   ```bash
   git clone https://github.com/your-username/rocket-flight-controller.git
   cd rocket-flight-controller
   ```

2. Install required Arduino libraries:

   * [Adafruit PCA9685 Servo Driver](https://github.com/adafruit/Adafruit-PWM-Servo-Driver-Library)
   * [Adafruit BMP280](https://github.com/adafruit/Adafruit_BMP280_Library)
   * [Adafruit HMC5883L](https://github.com/adafruit/Adafruit_HMC5883_Unified)
   * [Adafruit MPU6050](https://github.com/adafruit/Adafruit_MPU6050)
   * Wire.h, ESP8266WiFi.h (default)

3. Update WiFi / telemetry settings in `main.ino`.

4. Upload code to ESP8266.

---

## 🎮 Control & Telemetry

* **Servo Control** → PCA9685 adjusts nozzle angle for roll, pitch, yaw stabilization.
* **Telemetry Options** → Web dashboard / MQTT broker.
* **Onboard Display** → Roll, pitch, yaw, and altitude shown on SSD1306 OLED (optional).

---


## 📸 Demo

![Imag_alt](https://github.com/SUDO-KUNDAN23/Kalman-Filter-based-Orientation-Estimation-using-IMU-and-Esp8266/blob/3082ac19f8e28da455b46333ad47a36980f1a821/video_20250611_151812%20-%20frame%20at%200m0s.jpg)
---

📸 Video


https://github.com/user-attachments/assets/2286a8e6-76da-4c17-9963-7d51ec54a8ba



## 🚀 Future Improvements

* GPS integration for trajectory tracking.
* LoRa-based long-range telemetry.
* Automatic PID tuning for TVC control.
* Full ROS (Robot Operating System) integration.

---

## 📝 License

This project is licensed under the **MIT License** – free to use and modify.

---

## 👨‍💻 Author

Developed by Kundan Kumar Munda
🔗 [GitHub](https://github.com/your-username) | [LinkedIn](https://www.linkedin.com/in/kundan-kumar-munda-47b978286?utm_source=share&utm_campaign=share_via&utm_content=profile&utm_medium=android_app)

---

Do you want me to also include a **starter Arduino code snippet** (ESP8266 reading MPU6050, HMC5883L, BMP280 and driving PCA9685 servos) in this README so others can directly test it?
