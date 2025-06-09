# 🌦️ ESP32 Weather Station Project (Alpha Version)

A DIY weather station using the **ESP32** that measures and displays environmental data on a **16x2 I2C LCD**. It’s a modular, extendable project integrating temperature, pressure, humidity, air quality, and UV sensors!

---

## 🚀 Features

✅ **BMP180**: Temperature, pressure, and calibrated altitude readings  
✅ **DHT11**: Humidity and temperature readings  
✅ **MQ-135**: Air quality monitoring (raw analog output)  
✅ **GUVA-S12SD**: UV intensity measurement, calibrated to approximate UV Index  
✅ **16x2 I2C LCD**: Cycles through sensor readings every 5 seconds  
✅ **Sea-Level Pressure Calibration**: Uses a reference weather station to calibrate BMP180 readings for accurate altitude  
✅ **Expandable**: Clean code structure for adding more sensors or display modules!

## 📈 LCD Display Cycling

The LCD cycles through four screens:

1️⃣ Temperature & Pressure  
2️⃣ Altitude & Air Quality (MQ-135)  
3️⃣ Humidity & DHT11 Temperature  
4️⃣ UV Intensity (raw & approximate UV Index)

---

## 🛠️ Future Improvements

- Upgrade **DHT11** to a more precise sensor (e.g., **DHT22** or **BME280**)
- Add **data logging** (e.g., SD card or cloud)
- Create a **web interface** for remote monitoring
- Reintegrate **1.9" segment e-paper display** as a second display module

---

## 🔧 Hardware

| Component          | Description / Notes                     |
|--------------------|------------------------------------------|
| **ESP32**          | Main microcontroller                     |
| **BMP180**         | Pressure/altitude/temperature sensor     |
| **DHT11**          | Humidity and temperature sensor          |
| **MQ-135**         | Gas/air quality sensor (analog output)   |
| **GUVA-S12SD**     | UV sensor (analog output)                |
| **16x2 I2C LCD**   | User interface for sensor data display   |

---

## 🔌 Wiring

| Sensor/Module     | ESP32 Pin              |
|--------------------|------------------------|
| **BMP180 (I2C)**   | SDA (GPIO 21), SCL (GPIO 22) |
| **DHT11**          | DATA (GPIO 4)         |
| **MQ-135**         | A0 (GPIO 34)          |
| **GUVA-S12SD**     | OUT (GPIO 36)         |
| **16x2 LCD (I2C)** | SDA (GPIO 21), SCL (GPIO 22) |

✅ Power via 3.3 V and GND for sensors and display.

---

## 📦 Libraries Used

- `Wire.h` – I2C communication  
- `LiquidCrystal_I2C.h` – LCD driver  
- `Adafruit_BMP085.h` – BMP180 driver  
- `DHT.h` – DHT11 driver  

---


## ⚙️ Calibration and Usage

### Sea-Level Pressure Calibration
If you know the **pressure** at a reference altitude (from a local weather station), use:

\[
P_0 = P \times \left(1 - \frac{0.0065 \times h}{T + 0.0065 \times h + 273.15} \right)^{-5.257}
\]

Example in code:
```cpp
float localSeaLevelPressure = calibrateSeaLevelPressure(pressureAtAltitude, referenceAltitude, referenceTemp);
