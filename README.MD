# 🌦️ Weather Station Project (Alpha Version)

## 🚀 Overview
This project is a **weather station prototype** using the **ESP32** microcontroller. It reads and displays **environmental data** from multiple sensors on an **I2C LCD display**.

---

## 🛠️ Hardware Components
- **ESP32** microcontroller
- **BMP180** sensor: temperature, pressure, altitude
- **DHT11** sensor: temperature, humidity
- **MQ-135** gas sensor: air quality
- **16x2 I2C LCD** display

---

## 🔌 Wiring Summary
- **BMP180** (I2C): SDA (GPIO 21), SCL (GPIO 22), VCC (3.3V), GND
- **DHT11**: DATA (GPIO 4), VCC (3.3V), GND
- **MQ-135**: A0 (GPIO 34), VCC (5V), GND
- **16x2 LCD**: SDA (GPIO 21), SCL (GPIO 22), VCC (3.3V or 5V), GND

---

## ⚙️ Features
✅ **Sensor readings**:
- BMP180: temperature, pressure, altitude
- DHT11: temperature, humidity
- MQ-135: raw air quality analog value

✅ **LCD screen cycling**:
- Displays different sensor data in **5-second intervals**.

✅ **Smooth user experience**:
- Non-blocking loop using `millis()` for screen updates.
- Clear, concise data presentation on the LCD.

---

## 📦 Libraries Used
- **Wire.h** (I2C communication)
- **LiquidCrystal_I2C.h** (LCD display driver)
- **Adafruit_BMP085.h** (BMP180 sensor)
- **DHT.h** (DHT11 sensor)

---

## 🔧 Future Improvements
- Add a **more accurate air quality conversion** for MQ-135 readings.
- Add **local altitude calibration** for BMP180 altitude accuracy.
- Implement **data logging** or **wireless data upload** (e.g., to a web server or cloud).
- Integrate a **web interface** to display readings.
- Optionally re-integrate the **1.9” segment e-paper module**.

---

## ⚠️ Notes
- This is an **alpha version**: meant for prototyping and experimentation.
- Final implementation may involve **calibration** and **sensor accuracy validation**.

---

