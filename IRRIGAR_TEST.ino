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
int gaugeposition_x = 20;                                                 // these two variables govern the position
int gaugeposition_y = 0;

int radius = 40;

int circleRadius = 25;
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
  tft.setTextColor(ILI9341_BLUE);  tft.setTextSize(3);
  //  tft.setFont(&FreeMonoBoldOblique12pt7b);
  tft.println("IRRIGAR");
  //  tft.setFont(&FreeMonoBoldOblique12pt7b);
  tft.setCursor(20, 90);
  tft.setTextColor(ILI9341_BLACK);  tft.setTextSize(2);
  tft.println("Your solution to irrigation");
  delay(2000);
  setRingMeter();
}
void loop() {
  int board1X = boardsStruct[0].x;
  int board1Y = boardsStruct[0].y;
  int board2X = boardsStruct[1].x;
  int board2Y = boardsStruct[1].y;
  int nodeData[4] = {board1X, board1Y, board2X, board2Y};


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
    controlIrrPump(nodeData[1], nodeData[3]);
    updateSensors((gaugeposition_x + 100), (gaugeposition_y + 25), (gaugeposition_x + 200), (gaugeposition_y + 25), nodeData[0], nodeData[2]);
    updateValves(5, 150, 180, nodeData[1], nodeData[3]);

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
    updateWL(gaugeposition_x + 3, gaugeposition_y + 25);
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
      updatePump(5, 80, tankPumpState);
    }
    tankPumpState = 1;
  }
  else if (waterLevel > waterFilled) {
    digitalWrite(tankPump, HIGH);
    Serial.println("Tank Pump OFF");
    if (tankPumpState == 1) {
      updatePump(5, 80, tankPumpState);
    }
    tankPumpState = 0;
  }
}


void controlIrrPump(int valve1, int valve2) {
  if (waterLevel < waterLow) {
    digitalWrite(irrPump, HIGH);
    Serial.println("Irrigation Pump OFF");
    if (irrPumpState == 1) {
      updatePump(140, 145, irrPumpState);
    }
    irrPumpState = 0;
    return;
  }

  if (!valve1 || !valve2) {
    digitalWrite(irrPump, LOW);
    Serial.println("Irrigation Pump ON");
    if (irrPumpState == 0) {
      updatePump(140, 145, irrPumpState);
    }
    irrPumpState = 1;
  } else {
    digitalWrite(irrPump, HIGH);
    Serial.println("Irrigation Pump OFF");
    if (irrPumpState == 1) {
      updatePump(140, 145, irrPumpState);
    }
    irrPumpState = 0;
  }
}


long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

void setRingMeter() {
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor (YELLOW, BLACK);
  tft.setCursor (20, 0); tft.setTextSize (2); tft.print("Water");
  tft.setCursor (110, 0); tft.setTextSize (2); tft.print("SoilM1");
  tft.setCursor (210, 0); tft.setTextSize (2); tft.print("SoilM2");

  tft.setCursor (5, 180); tft.setTextSize (2); tft.print("VALVE1: ");
  tft.setCursor (150, 180); tft.setTextSize (2); tft.print("VALVE2: ");


  tft.setCursor (5, 130); tft.setTextSize (2); tft.print("Tank: ");
  tft.setCursor (125, 130); tft.setTextSize (2); tft.print("Irrigation: ");
}

void updatePump(int x, int circle, int pumpState) {
  if (pumpState == 1) {
    tft.setTextColor (WHITE, RED);
    tft.fillCircle(x + circle, 135, 22, ILI9341_RED);
    tft.setCursor (x + circle - 15 , 130); tft.setTextSize (2); tft.print("OFF");
  } else {
    tft.setTextColor (WHITE, GREEN);
    tft.fillCircle(x + circle, 135, 22, ILI9341_GREEN);
    tft.setCursor (x + circle - 10, 130); tft.setTextSize (2); tft.print("ON");
  }
}


void updateWL(int x, int y) {
  ringMeter (waterLevel, 0, 60, x, y, radius, "cm", RED2BLUE);
}


void updateValves(int x, int x1, int y, int valve1, int valve2) {
  //valve 1
  if (!valve1) {
    tft.setTextColor (WHITE, GREEN);
    tft.fillCircle(x + 105, y + 5, 22, ILI9341_GREEN);
    tft.setCursor (x + 95, y); tft.setTextSize (2); tft.print("ON");
    Serial.println("Valve 1 ON");
  }
  else {
    tft.setTextColor (WHITE, RED);
    tft.fillCircle(x + 105, y + 5, 22, ILI9341_RED);
    tft.setCursor (x + 90, y); tft.setTextSize (2); tft.print("OFF");
    Serial.println("Valve 1 OFF");
  }
  //valve 2
  if (!valve2) {
    tft.setTextColor (WHITE, GREEN);
    tft.fillCircle(x1 + 105, y + 5, 22, ILI9341_GREEN);
    tft.setCursor (x1 + 95, y); tft.setTextSize (2); tft.print("ON");
    Serial.println("Valve 2 ON");
  }
  else {
    tft.setTextColor (WHITE, RED);
    tft.fillCircle(x1 + 105, y + 5, 22, ILI9341_RED);
    tft.setCursor (x1 + 90, y); tft.setTextSize (2); tft.print("OFF");
    Serial.println("Valve 2 OFF");
  }
}


void updateSensors(int x, int y, int x1, int y1, int sensor1, int sensor2) {
  ringMeter (sensor1, 0, 100, (gaugeposition_x + 100), (gaugeposition_y + 25), radius, "%", RED2GREEN);
  ringMeter (sensor2, 0, 100, (gaugeposition_x + 200), (gaugeposition_y + 25), radius, "%", RED2GREEN);

}


int ringMeter(int value, int vmin, int vmax, int x, int y, int r, char *units, byte scheme) {
  // Minimum value of r is about 52 before value text intrudes on ring
  // drawing the text first is an option

  x += r; y += r;                                                             // calculate coordinates of center of ring
  int w = r / 3;                                                              // width of outer ring is 1/4 of radius
  int angle = 150;                                                            // half the sweep angle of the meter (300 degrees)
  int v = map(value, vmin, vmax, -angle, angle);                              // map the value to an angle v
  byte seg = 3;                                                               // segments are 3 degrees wide = 100 segments for 300 degrees
  byte inc = 6;                                                               // draw segments every 3 degrees, increase to 6 for segmented ring
  int colour = BLUE;                                                          // variable to save "value" text color from scheme and set default


  for (int i = -angle + inc / 2; i < angle - inc / 2; i += inc)               // draw color blocks every increment degrees
  {
    float sx = cos((i - 90) * 0.0174532925);                                 // calculate pair of coordinates for segment start
    float sy = sin((i - 90) * 0.0174532925);
    uint16_t x0 = sx * (r - w) + x;
    uint16_t y0 = sy * (r - w) + y;
    uint16_t x1 = sx * r + x;
    uint16_t y1 = sy * r + y;

    float sx2 = cos((i + seg - 90) * 0.0174532925);                          // salculate pair of coordinates for segment end
    float sy2 = sin((i + seg - 90) * 0.0174532925);
    int x2 = sx2 * (r - w) + x;
    int y2 = sy2 * (r - w) + y;
    int x3 = sx2 * r + x;
    int y3 = sy2 * r + y;

    if (i < v)
    { // fill in coloured segments with 2 triangles
      switch (scheme)
      {
        case 0: colour = RED; break;                                     // fixed color
        case 1: colour = GREEN; break;                                   // fixed color
        case 2: colour = BLUE; break;                                    // fixed colour
        case 3: colour = rainbow(map(i, -angle, angle, 0, 127)); break;  // full spectrum blue to red
        case 4: colour = rainbow(map(i, -angle, angle, 70, 127)); break; // green to red (high temperature etc)
        case 5: colour = rainbow(map(i, -angle, angle, 127, 63)); break; // red to green (low battery etc)
        case 6: colour = rainbow(map(i, -angle, angle, 127, 0)); break; // red to blue (low battery etc)
        default: colour = BLUE; break;                                   // fixed color
      }
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, colour);
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, colour);
    }
    else                                                                     // fill in blank segments
    {
      tft.fillTriangle(x0, y0, x1, y1, x2, y2, BLACK );    //SCALE1            // color of the unoccupied ring scale
      tft.fillTriangle(x1, y1, x2, y2, x3, y3, BLACK );           //SCALE0     // color of the unoccupied ring scale
    }
  }

  char buf [10];                                                              // convert value to a string
  byte len = 2; if (value > 999) len = 4;
  dtostrf (value, len, 0, buf);
  buf[len] = ' '; buf[len] = 0;                                               // add blanking space and terminator, helps to centre text too!

  tft.setTextSize (2);
  if (value > 9)
  {
    tft.setTextColor (colour, BLACK);
    tft.setCursor (x - 13, y - 10); tft.setTextSize (2);
    tft.print (buf);
  }

  if (value < 10)
  {
    tft.setTextColor (colour, BLACK);
    tft.setCursor (x - 13, y - 10); tft.setTextSize (2);
    tft.print (buf);
  }

  tft.setTextColor (WHITE, BLACK);
  tft.setCursor (x - 5, y + 10); tft.setTextSize (2);                        // units position relative to scale
  tft.print (units);                                                          // units display = celsius
  return x + r;                                                               // calculate and return right hand side x coordinate
}

// ################################################################################################################################
// Return a 16 bit rainbow colour
// ################################################################################################################################

unsigned int rainbow(byte value) {                                             // value is expected to be in range 0-127
  // value is converted to a spectrum color from 0 = blue through to 127 = red
  byte red = 0;                                                             // red is the top 5 bits of a 16 bit colour value
  byte green = 0;                                                          // green is the middle 6 bits
  byte blue = 0;                                                          // blue is the bottom 5 bits
  byte quadrant = value / 32;

  if (quadrant == 0)
  {
    blue = 31;
    green = 2 * (value % 32);
    red = 0;
  }
  if (quadrant == 1)
  {
    blue = 31 - (value % 32);
    green = 63;
    red = 0;
  }
  if (quadrant == 2)
  {
    blue = 0;
    green = 63;
    red = value % 32;
  }
  if (quadrant == 3)
  {
    blue = 0;
    green = 63 - 2 * (value % 32);
    red = 31;
  }
  return (red << 11) + (green << 5) + blue;
}

// ################################################################################################################################
// Return a value in range -1 to +1 for a given phase angle in degrees
// ################################################################################################################################

float sineWave(int phase) {

  return sin(phase * 0.0174532925);
}
