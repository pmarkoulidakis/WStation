#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP085.h>
#include <DHT.h>

// LCD setup
LiquidCrystal_I2C lcd(0x27, 16, 2);
unsigned long lastUpdate = 0;
int screenIndex = 0;
const int updateInterval = 4000; // 5 seconds

// BMP180 setup
Adafruit_BMP085 bmp;
float localSeaLevelPressure;
// MQ-135 setup
#define MQ135_PIN 34  // Analog pin for MQ-135
#define RL 10000  // MQ-135 10k Ohm load resistor

//DHT Sensor
#define DHT_PIN   4     // Digital pin for DHT11
#define DHT_TYPE  DHT11 // DHT11 sensor
DHT dht(DHT_PIN, DHT_TYPE);

//UV sensor
#define UV_PIN 36

void setup() {
  Serial.begin(115200);


  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();

  // Initialize BMP180
  if (!bmp.begin()) {
    lcd.setCursor(0, 0);
    lcd.print("BMP180 error");
    Serial.println("Could not find BMP180 sensor.");
    while (1) {}
  }
// Use data from a well know local area calibrated weather station
  float referencePressure = 1011.31; // hPa
  float referenceAltitude = 40.0;   // meters
  float referenceTemp = 31.4;       // Celsius
  
  localSeaLevelPressure = calibrateSeaLevelPressure(referencePressure, referenceAltitude, referenceTemp);
  Serial.print("Calibrated Sea-Level Pressure: ");
  Serial.print(localSeaLevelPressure, 1);
  Serial.println(" hPa");
  
  lcd.setCursor(0, 0);
  lcd.print("BMP180 OK!");

}

void loop() {
  // Read temperature, pressure, and altitude
  float temp = bmp.readTemperature();               // Celsius
  float pressure = bmp.readPressure() / 100.0;      // hPa
  float altitude = bmp.readAltitude(localSeaLevelPressure);       // Altitude in meters, assuming sea-level pressure of 1013.25 hPa

  float humidity = dht.readHumidity();
  float dhtTemp = dht.readTemperature();
  float dewPoint = calculateDewPoint(dhtTemp, humidity);
  
  int analogValue = analogRead(MQ135_PIN);
  float voltage = analogValue * (3.3 / 4095.0);
  float Rs = (3.3 - voltage) / voltage * RL;

  int uvRaw = analogRead(UV_PIN);
  // Convert raw ADC to voltage (3.3V ref, 12-bit ADC)
  float uvVoltage = uvRaw * (3.3 / 4095.0);

  // Approximate UV Index
  float uvIndex = (uvVoltage - 0.99) / 0.06;
  if (uvIndex < 0) uvIndex = 0; // No negative index

  // Display on LCD
  if (millis() - lastUpdate >= updateInterval) {
    lcd.clear();

    switch (screenIndex) {
      case 0:
        lcd.setCursor(0, 0);
        lcd.print("Temp:");
        lcd.print(temp, 1);
        lcd.print((char)223); // degree symbol
        lcd.print("C");

        lcd.setCursor(0, 1);
        lcd.print("Pressure:");
        lcd.print(pressure, 0);
        lcd.print("hPa");
        break;

      case 1:
        lcd.setCursor(0, 0);
        lcd.print("Altitude:");
        lcd.print(altitude, 1);
        lcd.print("m");

        lcd.setCursor(0, 1);
        lcd.print("Air Q Raw:");
        lcd.print(analogValue);
        break;

      case 2:
        lcd.setCursor(0, 0);
        lcd.print("Humidity:");
        if (isnan(humidity)) {
          lcd.print("Error");
        } else {
          lcd.print(humidity, 0);
          lcd.print("%");
        }

        lcd.setCursor(0, 1);
        lcd.print("DHT T:");
        if (isnan(dhtTemp)) {
          lcd.print("Error");
        } else {
          lcd.print(dhtTemp, 1);
          lcd.print((char)223);
          lcd.print("C");
        }
        break;

        case 3:
          lcd.setCursor(0, 0);
          lcd.print("UV Index:");
          lcd.print(uvIndex, 1);

          lcd.setCursor(0, 1);
          lcd.print("Voltage:");
          lcd.print(uvVoltage, 2);
          lcd.print(" V");
          break;
          
        case 4:
          lcd.setCursor(0, 0);
          lcd.print("Dew Pnt: ");
          if (isnan(dewPoint)) {
            lcd.print("Error");
          } else {
            lcd.print(dewPoint);
            lcd.print((char)223);
            lcd.print("C");
          }
        break;

    }

    // Cycle to the next screen
    screenIndex++;
    if (screenIndex > 4) screenIndex = 0;

    lastUpdate = millis();
  }


}

// Function to calculate dew point in Celsius
float calculateDewPoint(float dhtTemp, float humidity) {
  // Constants for Magnus formula
  const float a = 17.27;
  const float b = 237.7; // Celsius

  // Calculate alpha
  float alpha = ((a * dhtTemp) / (b + dhtTemp)) + log(humidity / 100.0);

  // Calculate dew point
  float dewPoint = (b * alpha) / (a - alpha);

  return dewPoint;
}

float calibrateSeaLevelPressure(float referencePressure, float referenceAltitude, float referenceTemp) {
  const float lapseRate = 0.0065;
  float tempK = referenceTemp + 273.15;

  float seaLevelPressure = referencePressure *
                            pow(1 - (lapseRate * referenceAltitude) / (tempK + lapseRate * referenceAltitude), -5.257);

  return seaLevelPressure;
}