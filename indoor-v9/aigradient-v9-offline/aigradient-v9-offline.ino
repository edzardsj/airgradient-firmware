/*
Important: This code is only for the DIY PRO / AirGradient ONE PCB Version 9 with the ESP-C3 MCU.

It is a high quality sensor showing PM2.5, CO2, TVOC, NOx, Temperature and Humidity on a small display and LEDbar and can send data over Wi-Fi.

The code needs the following libraries installed:
"U8g2" by oliver tested with version 2.34.22
"Sensirion I2C SGP41" by Sensation Version 1.0.0
"Sensirion Gas Index Algorithm" by Sensation Version 3.2.2
"PMS Library" by Markusz Kacki version 1.1.0
"S8_UART" by Josep Comas Version 1.0.1
"arduino-sht" by Johannes Winkelmann Version 1.2.5
"Adafruit NeoPixel" by Adafruit Version 1.12.0

Configuration:
Please set in the code below the configuration parameters.

Select "LOLIN C3 Mini" Board

CC BY-SA 4.0 Attribution-ShareAlike 4.0 International License

*/

#include "PMS.h"
#include <HardwareSerial.h>
#include <Wire.h>
#include "s8_uart.h"
#include <HTTPClient.h>
#include <Adafruit_NeoPixel.h>
#include <EEPROM.h>
#include "SHTSensor.h"
#include <SensirionI2CSgp41.h>
#include <NOxGasIndexAlgorithm.h>
#include <VOCGasIndexAlgorithm.h>
#include <U8g2lib.h>

#define DEBUG false

#define I2C_SDA 7
#define I2C_SCL 6

HTTPClient client;

Adafruit_NeoPixel pixels(11, 10, NEO_GRB + NEO_KHZ800);
SensirionI2CSgp41 sgp41;
VOCGasIndexAlgorithm voc_algorithm;
NOxGasIndexAlgorithm nox_algorithm;
SHTSensor sht;

PMS pms1(Serial0);

PMS::DATA data1;

S8_UART* sensor_S8;
S8_sensor sensor;

// time in seconds needed for NOx conditioning
uint16_t conditioning_s = 10;

// for persistent saving and loading
int addr = 4;
byte value;

// Display bottom right
U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// start at 0; incl. start, excl. end
int tmp_start = 0;
int tmp_end = 3;
int hum_start = 4;
int hum_end = 7;
int co2_start = 8;
int co2_end = 11;

// PM2.5 in US AQI (default ug/m3)
boolean inUSAQI = false;

// Display Position
boolean displayTop = true;

// use RGB LED Bar
boolean useRGBledBar = true;

int loopCount = 0;

unsigned long currentMillis = 0;

const int oledInterval = 5000;
unsigned long previousOled = 0;

const int tvocInterval = 1000;
unsigned long previousTVOC = 0;
int TVOC = -1;
int NOX = -1;

const int co2Interval = 5000;
unsigned long previousCo2 = 0;
int Co2 = 600;

const int pmInterval = 5000;
unsigned long previousPm = 0;
int pm25 = -1;
int pm01 = -1;
int pm10 = -1;

const int tempHumInterval = 5000;
unsigned long previousTempHum = 0;
float temp;
int hum;


void setup() {
  if (DEBUG) {
    Serial.begin(115200);
    // see https://github.com/espressif/arduino-esp32/issues/6983
    Serial.setTxTimeoutMs(0);  // <<<====== solves the delay issue
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  pixels.begin();
  pixels.clear();
  pixels.show();

  Serial1.begin(9600, SERIAL_8N1, 0, 1);
  Serial0.begin(9600);
  u8g2.begin();

  showInfoLines("Warming Up", "Serial Number:", String(getNormalizedMac()));
  sgp41.begin(Wire);
  delay(300);

  sht.init(Wire);
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_HIGH);
  // sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
  delay(300);

  // init Watchdog
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);

  sensor_S8 = new S8_UART(Serial1);
}  // setup()

void loop() {
  currentMillis = millis();
  updateTempHum();
  updateCo2();
  updatePm();
  updateTVOC();
  updateUI();
}  // loop()

void updateTVOC() {
  uint16_t error;
  char errorMessage[256];
  uint16_t defaultRh = 0x8000;
  uint16_t defaultT = 0x6666;
  uint16_t srawVoc = 0;
  uint16_t srawNox = 0;
  uint16_t defaultCompensationRh = 0x8000;  // in ticks as defined by SGP41
  uint16_t defaultCompensationT = 0x6666;   // in ticks as defined by SGP41
  uint16_t compensationRh = 0;              // in ticks as defined by SGP41
  uint16_t compensationT = 0;               // in ticks as defined by SGP41

  delay(1000);

  compensationT = static_cast< uint16_t >((temp + 45) * 65535 / 175);
  compensationRh = static_cast< uint16_t >(hum * 65535 / 100);

  if (conditioning_s > 0) {
    error = sgp41.executeConditioning(compensationRh, compensationT, srawVoc);
    conditioning_s--;
  } else {
    error = sgp41.measureRawSignals(compensationRh, compensationT, srawVoc,
                                    srawNox);
  }

  if (currentMillis - previousTVOC >= tvocInterval) {
    previousTVOC += tvocInterval;
    if (error) {
      TVOC = -1;
      NOX = -1;
    } else {
      TVOC = voc_algorithm.process(srawVoc);
      NOX = nox_algorithm.process(srawNox);
    }
  }
}

void updateCo2() {
  if (currentMillis - previousCo2 >= co2Interval) {
    previousCo2 += co2Interval;
    Co2 = sensor_S8->get_co2();
  }
}

void updatePm() {
  if (currentMillis - previousPm >= pmInterval) {
    previousPm += pmInterval;
    if (pms1.readUntil(data1, 2000)) {
      pm01 = data1.PM_AE_UG_1_0;
      pm25 = data1.PM_AE_UG_2_5;
      pm10 = data1.PM_AE_UG_10_0;
    } else {
      pm01 = -1;
      pm25 = -1;
      pm10 = -1;
    }
  }
}

void updateTempHum() {
  if (currentMillis - previousTempHum >= tempHumInterval) {
    previousTempHum += tempHumInterval;

    if (sht.readSample()) {
      temp = sht.getTemperature();
      hum = sht.getHumidity();
    } else {
      Serial.print("Error in readSample()\n");
      temp = -10001;
      hum = -10001;
    }
  }
}

void updateUI() {
  if (currentMillis - previousOled >= oledInterval) {
    updateDisplay();
    setRGBledTempColor(temp);
    setRGBledHumColor(hum);
    setRGBledCO2color(Co2);
    pixels.show();
  }
}

void showInfoLines(String ln1, String ln2, String ln3) {
  char buf[9];
  u8g2.firstPage();
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_t0_16_tf);
    u8g2.drawStr(1, 10, String(ln1).c_str());
    u8g2.drawStr(1, 30, String(ln2).c_str());
    u8g2.drawStr(1, 50, String(ln3).c_str());
  } while (u8g2.nextPage());
}

void updateDisplay() {
  char buf[9];
  u8g2.firstPage();
  u8g2.firstPage();
  do {

    u8g2.setFont(u8g2_font_t0_16_tf);

    if (temp > -10001) {
      sprintf(buf, "%.1f°C", temp);
    } else {
      sprintf(buf, "-°C");
    }
    u8g2.drawUTF8(1, 10, buf);

    if (hum >= 0) {
      sprintf(buf, "%d%%", hum);
    } else {
      sprintf(buf, " -%%");
    }
    if (hum > 99) {
      u8g2.drawStr(97, 10, buf);
    } else {
      u8g2.drawStr(105, 10, buf);
      // there might also be single digits, not considered, sprintf might actually support a leading space
    }

    u8g2.drawLine(1, 13, 128, 13);
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawUTF8(1, 27, "CO2");
    u8g2.setFont(u8g2_font_t0_22b_tf);
    if (Co2 > 0) {
      sprintf(buf, "%d", Co2);
    } else {
      sprintf(buf, "%s", "-");
    }
    u8g2.drawStr(1, 48, buf);
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawStr(1, 61, "ppm");
    u8g2.drawLine(45, 15, 45, 64);
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawStr(48, 27, "PM2.5");
    u8g2.setFont(u8g2_font_t0_22b_tf);

    if (inUSAQI) {
      if (pm25 >= 0) {
        sprintf(buf, "%d", PM_TO_AQI_US(pm25));
      } else {
        sprintf(buf, "%s", "-");
      }
      u8g2.drawStr(48, 48, buf);
      u8g2.setFont(u8g2_font_t0_12_tf);
      u8g2.drawUTF8(48, 61, "AQI");
    } else {
      if (pm25 >= 0) {
        sprintf(buf, "%d", pm25);
      } else {
        sprintf(buf, "%s", "-");
      }
      u8g2.drawStr(48, 48, buf);
      u8g2.setFont(u8g2_font_t0_12_tf);
      u8g2.drawUTF8(48, 61, "ug/m³");
    }

    u8g2.drawLine(82, 15, 82, 64);
    u8g2.setFont(u8g2_font_t0_12_tf);
    u8g2.drawStr(85, 27, "TVOC:");
    if (TVOC >= 0) {
      sprintf(buf, "%d", TVOC);
    } else {
      sprintf(buf, "%s", "-");
    }
    u8g2.drawStr(85, 39, buf);
    u8g2.drawStr(85, 53, "NOx:");
    if (NOX >= 0) {
      sprintf(buf, "%d", NOX);
    } else {
      sprintf(buf, "%s", "-");
    }
    u8g2.drawStr(85, 63, buf);

  } while (u8g2.nextPage());
}

void resetWatchdog() {
  Serial.println("Watchdog reset");
  digitalWrite(2, HIGH);
  delay(20);
  digitalWrite(2, LOW);
}


String getNormalizedMac() {
  String mac = WiFi.macAddress();
  mac.replace(":", "");
  mac.toLowerCase();
  return mac;
}

void setRGBledCO2color(int co2Value) {
  if (co2Value >= 300 && co2Value < 800) setRGBledColor(0x00FF00, co2_start, co2_end);
  if (co2Value >= 800 && co2Value < 1000) setRGBledColor(0x80FF00, co2_start, co2_end);
  if (co2Value >= 1000 && co2Value < 1200) setRGBledColor(0xFFFF00, co2_start, co2_end);
  if (co2Value >= 1200 && co2Value < 1500) setRGBledColor(0xFF8000, co2_start, co2_end);
  if (co2Value >= 1500 && co2Value < 2000) setRGBledColor(0xFF0000, co2_start, co2_end);
  if (co2Value >= 2000 && co2Value < 3000) setRGBledColor(0x660000, co2_start, co2_end);
  if (co2Value >= 3000 && co2Value < 10000) setRGBledColor(0x990099, co2_start, co2_end);
}

void extractRGB(uint32_t color, uint8_t& red, uint8_t& green, uint8_t& blue) {
    red = (color >> 16) & 0xFF;
    green = (color >> 8) & 0xFF;
    blue = color & 0xFF;
}

uint32_t getFeelGoodColor(float value, float low_bad, float low_good, float high_good, float high_bad, uint32_t low_color, uint32_t good_color, uint32_t high_color) {
  if (value <= low_bad) {
      return low_color;
  } else if (value >= high_bad) {
      return high_color;
  } else if (value >= low_good && value <= high_good) {
      return good_color;
  }

  uint8_t good_r, good_g, good_b;
  uint8_t bad_r, bad_g, bad_b;
  float normalized;

  extractRGB(good_color, good_r, good_g, good_b);

  if (value < low_good) { // value between low_good and low_bad
      extractRGB(low_color, bad_r, bad_g, bad_b);
      normalized = (low_good - value) / (low_good - low_bad);
  } else { // value between high_good and high_bad
      extractRGB(high_color, bad_r, bad_g, bad_b);
      normalized = (value - high_good) / (high_bad - high_good);
  }

  // interpolate the r, g and b values 
  uint8_t r = (uint8_t) (bad_r * normalized + good_r * (1 - normalized));
  uint8_t g = (uint8_t) (bad_g * normalized + good_g * (1 - normalized));
  uint8_t b = (uint8_t) (bad_b * normalized + good_b * (1 - normalized));

  return ((r << 16) | (g << 8) | b);
}

void setRGBledHumColor(int humValue) {
  setRGBledColor(getFeelGoodColor((float) humValue, 35, 45, 55, 70, 0xFF8000, 0xFFFFFF, 0x00FFFF), hum_start, hum_end);
}

void setRGBledTempColor(float tempValue) {
  setRGBledColor(getFeelGoodColor(tempValue, 16, 21, 23, 28, 0x0000FF, 0x00FF00, 0xFF0000), tmp_start, tmp_end);
}

void setRGBledColor(uint32_t color, int start, int end) {
  if (useRGBledBar) {
    for (int i = start; i < end; i++) {
      pixels.setPixelColor(i, color);
      delay(30);
      pixels.show();
    }
  }
}

// Calculate PM2.5 US AQI
int PM_TO_AQI_US(int pm02) {
  if (pm02 <= 12.0) return ((50 - 0) / (12.0 - .0) * (pm02 - .0) + 0);
  else if (pm02 <= 35.4) return ((100 - 50) / (35.4 - 12.0) * (pm02 - 12.0) + 50);
  else if (pm02 <= 55.4) return ((150 - 100) / (55.4 - 35.4) * (pm02 - 35.4) + 100);
  else if (pm02 <= 150.4) return ((200 - 150) / (150.4 - 55.4) * (pm02 - 55.4) + 150);
  else if (pm02 <= 250.4) return ((300 - 200) / (250.4 - 150.4) * (pm02 - 150.4) + 200);
  else if (pm02 <= 350.4) return ((400 - 300) / (350.4 - 250.4) * (pm02 - 250.4) + 300);
  else if (pm02 <= 500.4) return ((500 - 400) / (500.4 - 350.4) * (pm02 - 350.4) + 400);
  else return 500;
};