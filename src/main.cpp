#include <EEPROM.h>
#include <M5Stack.h>
#include <SimpleKalmanFilter.h>
#include <adafruit_mlx90614.h>
#include <math.h>

#define READ_RATE 64
#define MAX_DATA_POINT READ_RATE
#define DISPLAY_REFRESH_RATE 30

float e_mea = 10;
float e_est = 10;
float q = 0.2;

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
SimpleKalmanFilter kf;

int pointer = 0;
float dataPoint[MAX_DATA_POINT] = {};

void setup() {
  M5.begin();                // Init M5Core.
  M5.Power.begin();          // Init Power module.
  M5.Lcd.fillScreen(WHITE);  // Set the screen background.
  EEPROM.begin(1);

  delay(1000);

  Wire.begin();  // Joing I2C bus
  mlx.begin();
  M5.Lcd.fillScreen(BLACK);

  q = EEPROM.readFloat(0);
  kf = SimpleKalmanFilter(e_mea, e_est, q);
}

int refreshTime = millis();

void loop() {
  M5.update();

  if (M5.BtnA.isPressed()) {
    q -= 0.01;
    if (q < 0) q = 0.1;
    kf = SimpleKalmanFilter(e_mea, e_est, q);
    EEPROM.writeFloat(0, q);
    EEPROM.commit();
  }

  if (M5.BtnB.isPressed()) {
    q = 0.2;
    kf = SimpleKalmanFilter(e_mea, e_est, q);
    EEPROM.writeFloat(0, q);
    EEPROM.commit();
  }

  if (M5.BtnC.isPressed()) {
    q += 0.01;
    if (q >= 1) q = 1;
    kf = SimpleKalmanFilter(e_mea, e_est, q);
    EEPROM.writeFloat(0, q);
    EEPROM.commit();
  }

  float readingTemp = mlx.readObjectTempC();

  float estimated_value = kf.updateEstimate(readingTemp);

  dataPoint[pointer++] = readingTemp;

  pointer %= MAX_DATA_POINT;

  float upperBoundTemp = 0;
  for (int i = 0; i < MAX_DATA_POINT; i++) {
    upperBoundTemp = max(upperBoundTemp, dataPoint[i]);
  }

  float lowerBoundTemp = 999;
  for (int i = 0; i < MAX_DATA_POINT; i++) {
    lowerBoundTemp = min(lowerBoundTemp, dataPoint[i]);
  }

  if (millis() > refreshTime) {
    M5.Lcd.setCursor(10, 10);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print("Upper Bound Temp.");
    M5.Lcd.setCursor(20, 34);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(RED, BLACK);
    M5.Lcd.printf("%3.2f c", upperBoundTemp);

    M5.Lcd.setCursor(10, 78);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print("Kalman Filter Temp.");
    M5.Lcd.setCursor(20, 108);
    M5.Lcd.setTextSize(6);
    M5.Lcd.setTextColor(ORANGE, BLACK);
    M5.Lcd.printf("%3.2f c", estimated_value);

    M5.Lcd.setCursor(10, 176);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.print("Lower Bound Temp.");
    M5.Lcd.setCursor(20, 200);
    M5.Lcd.setTextSize(3);
    M5.Lcd.setTextColor(CYAN, BLACK);
    M5.Lcd.printf("%3.2f c", lowerBoundTemp);

    M5.Lcd.setCursor(230, 205);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("T=%2.0f c", mlx.readAmbientTempC());

    M5.Lcd.setCursor(230, 220);
    M5.Lcd.setTextSize(2);
    M5.Lcd.setTextColor(WHITE, BLACK);
    M5.Lcd.printf("Q=%1.2f ", q);

    refreshTime = millis() + (1000.0 / DISPLAY_REFRESH_RATE);
  }

  delay(1000 / READ_RATE);
}