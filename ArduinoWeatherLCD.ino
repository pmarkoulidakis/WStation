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

// MQ-135 setup
#define MQ135_PIN 34  // Analog pin for MQ-135
#define RL 10000  // MQ-135 10k Ohm load resistor

//DHT Sensor
#define DHT_PIN   4     // Digital pin for DHT11
#define DHT_TYPE  DHT11 // DHT11 sensor
DHT dht(DHT_PIN, DHT_TYPE);

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

  lcd.setCursor(0, 0);
  lcd.print("BMP180 OK!");

}

void loop() {
  // Read temperature, pressure, and altitude
  float temp = bmp.readTemperature();               // Celsius
  float pressure = bmp.readPressure() / 100.0;      // hPa
  float altitude = bmp.readAltitude();       // Altitude in meters, assuming sea-level pressure of 1013.25 hPa

  float humidity = dht.readHumidity();
  float dhtTemp = dht.readTemperature();

  int analogValue = analogRead(MQ135_PIN);
  float voltage = analogValue * (3.3 / 4095.0);
  float Rs = (3.3 - voltage) / voltage * RL;

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

      // Add more cases if you want more screens
    }

    // Cycle to the next screen
    screenIndex++;
    if (screenIndex > 2) screenIndex = 0;

    lastUpdate = millis();
  }


}
