#include <Adafruit_GFX.h>       // include Adafruit graphics library
#include <Adafruit_ILI9341.h>   // include Adafruit ILI9341 TFT library
#include <Fonts\FreeMonoBoldOblique12pt7b.h>
#include <Fonts\FreeMono9pt7b.h>
#include <esp_now.h>
#include <WiFi.h>

#define TFT_CS   15 //D2     // TFT CS  pin is connected to NodeMCU pin D2 but it is assigned as GPIO4 
#define TFT_RST  4 //D3     // TFT RST pin is connected to NodeMCU pin D3 but it is assigned as GPIO0
#define TFT_DC   2 //D4     // TFT DC  pin is connected to NodeMCU pin D4 but it is assigned as GPIO2
// initialize ILI9341 TFT library with hardware SPI module
// SCK (CLK) ---> NodeMCU pin D5 (GPIO14)
// MOSI(DIN) ---> NodeMCU pin D7 (GPIO13)
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);

// some principal color definitions
// RGB 565 color picker at https://ee-programming-notepad.blogspot.com/2016/10/16-bit-color-generator-picker.html
#define WHITE       0xFFFF
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define GREY        0x2108
#define SCALE0      0xC655                                                  // accent color for unused scale segments                                   
#define SCALE1      0x5DEE                                                  // accent color for unused scale segments
#define TEXT_COLOR  0xFFFF                                                  // is currently white 

// circular scale color scheme
#define RED2RED 0
#define GREEN2GREEN 1
#define BLUE2BLUE 2
#define BLUE2RED 3
#define GREEN2RED 4
#define RED2GREEN 5
#define RED2BLUE 6

#define tankPump 22
#define irrPump 17
#define pingPin 14 // Trigger Pin of Ultrasonic Sensor
#define echoPin 13 // Echo Pin of Ultrasonic Sensor

const int16_t tankHeight = 60;
int waterLevel;
const int waterFilled = 55;
const int waterLow = 20;

int lastClearScreen = 0;
int lastRefresh = 0;

int tankPumpState = 0;
int irrPumpState = 0;

// display positions
int gaugeposition_x = 0;                                                 // these two variables govern the position
int gaugeposition_y = 0;

//
//Data struct to be received
typedef struct struct_message {
  int id;
  int x;
  bool y;
} struct_message;


struct_message myData;

struct_message board1;
struct_message board2;


struct_message boardsStruct[2] = {board1, board2};

void OnDataRecv(const uint8_t * mac_addr, const uint8_t *incomingData, int len) {
  char macStr[18];
  //Serial.print("Packet received from: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //Serial.println(macStr);
  memcpy(&myData, incomingData, sizeof(myData));
  //Serial.printf("Board ID %u: %u bytes\n", myData.id, len);

  boardsStruct[myData.id - 1].x = myData.x;
  boardsStruct[myData.id - 1].y = myData.y;
  //Serial.printf("x value: %d \n", boardsStruct[myData.id - 1].x);
  //Serial.printf("y value: %d \n", boardsStruct[myData.id - 1].y);
  //Serial.println();
}


void setup() {
  Serial.begin(115200); // open serial port, set the baud rate to 9600 bps

  WiFi.mode(WIFI_STA);


  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  pinMode(tankPump, OUTPUT);
  pinMode(irrPump, OUTPUT);
  pinMode(pingPin, OUTPUT);
  pinMode(echoPin, INPUT);

  digitalWrite(tankPump, HIGH);
  digitalWrite(irrPump, HIGH);

  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_WHITE);
  tft.setCursor(120, 60);
  tft.setTextColor(ILI9341_BLUE);  tft.setTextSize(2);
  tft.setFont(&FreeMonoBoldOblique12pt7b);
  tft.println("IRRIGAR");
  tft.setFont(&FreeMonoBoldOblique12pt7b);
  tft.setCursor(0, 90);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(1);
  tft.println("Your solution to irrigation");
  delay(2000);
  tft.fillScreen(ILI9341_BLACK);
}
void loop() {
  //  int board1X = boardsStruct[0].x;
  //  int board1Y = boardsStruct[0].y;
  //  int board2X = boardsStruct[1].x;
  //  int board2Y = boardsStruct[1].y;
  int nodeData[] = [boardsStruct[0].x, boardsStruct[0].y, boardsStruct[1].x, boardsStruct[1].y]
                   //  getNodeData();

  if (millis() - lastRefresh > 1000) {
    Serial.print("Sensor 1: ");
    Serial.println(nodeData[0]);
    Serial.print("Valve 1: ");
    Serial.println(nodeData[1]);
    Serial.print("Sensor 2: ");
    Serial.println(nodeData[2]);
    Serial.print("Valve 2: ");
    Serial.println(nodeData[3]);

    getWaterLevel();
    controlTankPump();
    controlIrrPump(board1Y, board2Y);
    updateSensors(0, 120, nodeData[0], nodeData[1]);
    updateValves(0, 160, nodeData[2], nodeData[3]);

    lastRefresh = millis();
  }
}


void getWaterLevel () {
  long duration, cm;
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(2);

  duration = pulseIn(echoPin, HIGH);
  cm = microsecondsToCentimeters(duration);

  waterLevel = tankHeight - cm;
  if (millis() - lastClearScreen > 2000) {
    updateWL(0, 10);
    lastClearScreen = millis();
  }
  Serial.print(waterLevel);
  Serial.print("cm: water Level");
  Serial.println();
}


void controlTankPump() {
  if (waterLevel < 30 ) {
    digitalWrite(tankPump, LOW);
    Serial.println("Tank Pump ON");
    if (tankPumpState == 0) {
      updatePump(0, 40, tankPumpState, "Tank ");
    }
    tankPumpState = 1;
  }
  else if (waterLevel > waterFilled) {
    digitalWrite(tankPump, HIGH);
    Serial.println("Tank Pump OFF");
    if (tankPumpState == 1) {
      updatePump(0, 40, tankPumpState, "Tank ");
    }
    tankPumpState = 0;
  }
}


void controlIrrPump(int valve1, int valve2) {
  if (waterLevel < waterLow) {
    digitalWrite(irrPump, HIGH);
    Serial.println("Irrigation Pump OFF");
    if (irrPumpState == 1) {
      updatePump(0, 80, irrPumpState, "Irrigation ");
    }
    irrPumpState = 0;
    return;
  }

  if (!valve1 || !valve2) {
    digitalWrite(irrPump, LOW);
    Serial.println("Irrigation Pump ON");
    if (irrPumpState == 0) {
      updatePump(0, 80, irrPumpState, "Irrigation ");
    }
    irrPumpState = 1;
  } else {
    digitalWrite(irrPump, HIGH);
    Serial.println("Irrigation Pump OFF");
    if (irrPumpState == 1) {
      updatePump(0, 80, irrPumpState, "Irrigation ");
    }
    irrPumpState = 0;
  }
}


long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}


void updatePump(int x, int y, int pumpState, String pump) {
  tft.fillRect (x, y, 280, 50, ILI9341_YELLOW);
  tft.setCursor(x, y + 20);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(1);
  if (pumpState == 1) {
    tft.print(pump);
    tft.println("Pump OFF");
  } else {
    tft.print(pump);
    tft.println("Pump ON");
  }
}


void updateWL(int x, int y) {
  tft.fillRect (x, y, 240, 30, ILI9341_YELLOW);
  tft.setCursor(x, y + 20);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(1);
  tft.print("Water Level: ");
  tft.println(waterLevel);
}


void updateValves(int x, int y, int valve1, int valve2) {
  tft.fillRect (x, y, 300, 30, ILI9341_YELLOW);
  tft.setCursor(x, y + 20);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(1);
  //valve 1
  tft.print("V 1: ");
  if (!valve1) {
    tft.print("ON ");
    Serial.println("Valve 1 ON");
  }
  else {
    tft.print("OFF ");
    Serial.println("Valve 1 OFF");
  }
  //valve 2
  tft.print("V 2: ");
  if (!valve2) {
    tft.println("ON");
    Serial.println("Valve 2 ON");
  }
  else {
    tft.print("OFF");
    Serial.println("Valve 2 OFF");
  }
}


void updateSensors(int x, int y, int sensor1, int sensor2) {
  tft.fillRect (x, y, 260, 30, ILI9341_YELLOW);
  tft.setCursor(x, y + 20);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(1);
  tft.print("S1: ");
  tft.print(sensor1);

  tft.print(" S2: ");
  tft.println(sensor2);
}
