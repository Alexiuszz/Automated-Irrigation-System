#include <Adafruit_GFX.h>       // include Adafruit graphics library
#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library
#include <Fonts\FreeMonoBoldOblique12pt7b.h>
#include <Fonts\FreeMono9pt7b.h>

#define TFT_CS   15 //D2     // TFT CS  pin is connected to NodeMCU pin D2 but it is assigned as GPIO4 
#define TFT_RST  4 //D3     // TFT RST pin is connected to NodeMCU pin D3 but it is assigned as GPIO0
#define TFT_DC   2 //D4     // TFT DC  pin is connected to NodeMCU pin D4 but it is assigned as GPIO2
// initialize ILI9341 TFT library with hardware SPI module
// SCK (CLK) ---> NodeMCU pin D5 (GPIO14)
// MOSI(DIN) ---> NodeMCU pin D7 (GPIO13)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

#define tankPump 22
#define irrPump 17
#define pingPin 14 // Trigger Pin of Ultrasonic Sensor
#define echoPin 13 // Echo Pin of Ultrasonic Sensor

const int16_t tankHeight = 108;
int waterLevel;

int lastClearScreen = 0;
int lastRefresh = 0;

int tankPumpState = 0;
int IrrPumpState = 0;
void setup() {
  Serial.begin(115200); // open serial port, set the baud rate to 9600 bps
  pinMode(tankPump, OUTPUT);
  pinMode(irrPump, OUTPUT);
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(tankPump, LOW);
  digitalWrite(irrPump, LOW);

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(50, 30); // ROW, collumn
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.setFont(&FreeMonoBoldOblique12pt7b);
  tft.println("IRRIGAR");
  tft.setCursor(0, 70);
  tft.setFont(&FreeMonoBoldOblique12pt7b);
  tft.setTextColor(ILI9341_YELLOW);  tft.setTextSize(1);
  tft.println("Your solution to irrigation");
  delay(1000);
  tft.fillScreen(ILI9341_BLACK);
}
void loop() {
  if (millis() - lastRefresh > 500) {
    getWaterLevel();
    if (waterLevel < 50 ) {
      digitalWrite(tankPump, HIGH);
      Serial.println("Tank Pump ON");
      if (tankPumpState == 0) {
        updateTank(0, 30);
      }
      tankPumpState = 1;
    }
    else if (waterLevel > 104) {
      digitalWrite(tankPump, LOW);
      Serial.println("Tank Pump OFF");
      if (tankPumpState == 1) {
        updateTank(0, 30);
      }
      tankPumpState = 0;
    }
    lastRefresh = millis();
  }
}

void getWaterLevel () {
  long duration, cm, cms;
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(2);

  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);

  Serial.print(cm);
  Serial.print("cm: Distance to water Level");
  Serial.println();

  waterLevel = tankHeight - cm;
  if (millis() - lastClearScreen > 1000) {
    updateWL(0, 100);
    lastClearScreen = millis();
  }
  Serial.print(waterLevel);
  Serial.print("cm: water Level");
  Serial.println();

}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void updateTank(int x, int y) {
  tft.fillRect (x, y, 200, 50, ILI9341_YELLOW);
  tft.setCursor(x, y + 20);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(1);
  if (tankPumpState == 1) {
    tft.println("Tank Pump OFF");
  } else {
    tft.println("Tank Pump ON");
  }
}

void updateWL(int x, int y) {
  tft.fillRect (x, y, 240, 30, ILI9341_YELLOW);
  tft.setCursor(x, y + 20);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(1);
  tft.print("Water Level: ");
  tft.println(waterLevel);
}
