#include <SPI.h>
#include <TFT_eSPI.h>       // CHANGE in SETUP
#include <max6675.h>
#include <TimerEvent.h>
#include <analogWrite.h>
#include <ESP32Encoder.h>
#include "bmp.h"
#include "profile.h"

TFT_eSPI tft = TFT_eSPI(135, 240); // Invoke custom library
MAX6675 thermocouple(12, 13, 15);
ESP32Encoder encoder;

#define fan     2
#define heater  17

#define r       25
#define g       26
#define b       27

#define CLK     32
#define DT      33
#define SW      39

#define btn1    38
#define btn2    37

const int encInterval = 100;
const int tempInterval = 300;
const int plotInterval = 2000;

// Create two TimerEvent instances
TimerEvent enc;
TimerEvent temp;
TimerEvent plot;

double setPoint, ambTemp = 40;
double error, lastError, PID_value;
unsigned long startTime, currentTime, previousTime, elapsedTime;

double kP = 10, kI = 0.5, kD = 0.3;
double pTerm, iTerm, dTerm;

int fanRate = 100;
int offset = 0;

int tempRead;
int count, ID, lastCount, plotter;
bool home = true, menu, start, setPaste, pointer = true;
bool setPid, setProfile, setFan, setTemp, setBuzz;
bool reflowProcess, startTimer;
bool heaterState, fanState;

void setup() {
  Serial.begin(115200);
  pinMode(btn1, INPUT);
  pinMode(btn2, INPUT);
  pinMode(SW, INPUT);

  pinMode(r, OUTPUT);
  pinMode(g, OUTPUT);
  pinMode(b, OUTPUT);
  rgb(1, 1, 1);
  getProfiles();
  pinMode(fan, OUTPUT);
  digitalWrite(fan, HIGH);

  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setSwapBytes(true);
  //tft.pushImage(0, 0,  240, 135, splash);
  //delay(3000);
  tft.setTextSize(2);
  tft.fillScreen(TFT_BLACK);

  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(CLK, DT);

  enc.set(encInterval, encFunc);
  temp.set(tempInterval, tempFunc);
  plot.set(plotInterval, plotFunc);
}

String heaterStat() {
  if (heaterState)
    return "ON";
  else
    return "OFF";
}
String fanStat() {
  if (fanState)
    return "ON";
  else
    return "OFF";
}

uint16_t temp2clr(int degree, int lo, int hi) {
  uint8_t Tr, Tg, Tb;
  Tr = map(degree, lo, hi, 0, 255);
  Tg = 0;
  Tb = map(degree, lo, hi, 255, 0);
  return tft.color565(Tr, Tg, Tb);
}


void loop() {
  temp.update();
  enc.update();
  if (home) {
    tft.setTextColor(TFT_WHITE);
    tft.setTextDatum(CC_DATUM);
    tft.drawString("TEMP", tft.width() / 2 - 90, tft.height() / 13);
    tft.drawString("HEATER", tft.width() / 2 , tft.height() / 13);
    tft.drawString("FAN", tft.width() / 2 + 90, tft.height() / 13);
    int padding = tft.textWidth("999", 2);
    tft.setTextColor(temp2clr(tempRead, 30, profile[ID].peak), TFT_BLACK);
    tft.setTextPadding(padding);
    tft.drawString(String(tempRead), tft.width() / 2 - 90, tft.height() / 4);
    tft.setTextColor(TFT_CYAN, TFT_BLACK);
    tft.drawString(heaterStat(), tft.width() / 2 , tft.height() / 4);
    tft.setTextColor(TFT_ORANGE, TFT_BLACK);
    tft.drawString(fanStat(), tft.width() / 2 + 90 , tft.height() / 4);
    tft.setTextPadding(0);

    tft.setTextColor(TFT_DARKGREEN, TFT_BLACK);
    tft.drawString("Paste:", tft.width() / 2 - 80, tft.height() / 2 - 5);
    tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
    tft.drawString(profile[ID].name, tft.width() / 2 + 35, tft.height() / 2 - 5);
    tft.setTextColor(TFT_DARKCYAN, TFT_BLACK);
    tft.drawString(profile[ID].meta, tft.width() / 2 - 5, tft.height() / 2 + 20);

    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("SETUP", tft.width() / 2 - 80, tft.height() - 15);
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.drawString("START", tft.width() / 2 + 80, tft.height() - 15);

    if (pointer) {
      if (count == 0) {
        tft.drawRect(0, tft.height() / 2 - 19, tft.width(), 25, TFT_WHITE);
        tft.drawRect(5, tft.height() - 29, 70, 25, TFT_BLACK);
        tft.drawRect(tft.width() / 2 + 45, tft.height() - 29, 70, 25, TFT_BLACK);
      }
      if (count == 1) {
        tft.drawRect(0, tft.height() / 2 - 19, tft.width(), 25, TFT_BLACK);
        tft.drawRect(5, tft.height() - 29, 70, 25, TFT_WHITE);
        tft.drawRect(tft.width() / 2 + 45, tft.height() - 29, 70, 25, TFT_BLACK);
      }
      if (count == 2) {
        tft.drawRect(0, tft.height() / 2 - 19, tft.width(), 25, TFT_BLACK);
        tft.drawRect(5, tft.height() - 29, 70, 25, TFT_BLACK);
        tft.drawRect(tft.width() / 2 + 45, tft.height() - 29, 70, 25, TFT_WHITE);
      }
      if (count > 2) {
        encoder.clearCount();
      }
    }

    if (digitalRead(SW) == HIGH && pointer) {
      if (count == 0) {
        setPaste = true;
        pointer = false;
      }
      if (count == 1) {
        home = false;
        menu = true;
        tft.fillScreen(TFT_BLACK);
      }
      if (count == 2) {
        home = false;
        start = true;
        tft.fillScreen(TFT_BLACK);
      }
      encoder.clearCount();
      delay(500);
    }

    if (digitalRead(SW) == HIGH && setPaste) {
      setPaste = false;
      pointer = true;
      encoder.clearCount();
      delay(500);
    }

    if (setPaste) {
      if (count >= numProfile) {
        encoder.clearCount();
      }
      else if (count >= 0) ID = count;
    }
    if (digitalRead(btn1) == LOW) {
      heaterState = !heaterState;
      if (heaterState) analogWrite(heater, 255);
      else analogWrite(heater, 0);
      delay(500);

    }
    if (digitalRead(btn2) == LOW) {
      fanState = !fanState;
      digitalWrite(fan, !fanState);
      delay(500);
    }
  }

  if (menu) {
    tft.setTextColor(TFT_GREENYELLOW, TFT_BLACK);
    tft.setTextDatum(CL_DATUM);
    tft.drawString("<--", 5, tft.height() / 2 - 50);
    tft.drawString("PID", 10, tft.height() / 2 - 25);
    tft.drawString("PRO", 10, tft.height() / 2);
    tft.drawString("FILE", 10, tft.height() / 2 + 25);

    if (count <= 4) tft.drawString("FAN ", 10, tft.height() / 2 + 50);
    if (count == 5) tft.drawString("TEMP", 10, tft.height() / 2 + 50);
    if (count == 6) tft.drawString("BUZZ", 10, tft.height() / 2 + 50);

    tft.drawFastVLine(tft.width() / 2 - 55, 0, tft.height(), TFT_WHITE);
    if (count != 3) tft.drawFastHLine(tft.width() / 2 - 55, tft.height() / 2 - 39, tft.width(), TFT_WHITE);
    else  tft.drawFastHLine(tft.width() / 2 - 55, tft.height() / 2 - 39, tft.width(), TFT_BLACK);

    if (pointer) {
      if (lastCount != count) {
        tft.drawRect(0, tft.height() / 2 - 62, 65, 25, TFT_BLACK);
        tft.drawRect(0, tft.height() / 2 - 36, 65, 25, TFT_BLACK);
        tft.drawRect(0, tft.height() / 2 - 13, 65, 50, TFT_BLACK);
        tft.drawRect(0, tft.height() / 2 + 37, 65, 25, TFT_BLACK);

        tft.fillRect(tft.width() / 2 - 54, 0, 180, tft.height(), TFT_BLACK);
        lastCount = count;
      }
      if (count == 0) {
        tft.drawRect(0, tft.height() / 2 - 62, 65, 25, TFT_WHITE);
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        tft.drawString("Return", tft.width() / 2 - 50, tft.height() / 2 - 50);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("REFLOWER", tft.width() / 2 - 5, tft.height() / 2);
        tft.drawString("V.1.2.0", tft.width() / 2 - 5, tft.height() / 2 + 25);
      }
      if (count == 1) {
        tft.drawRect(0, tft.height() / 2 - 36, 65, 25, TFT_WHITE);
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        tft.drawString("Tune PID", tft.width() / 2 - 50, tft.height() / 2 - 50);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("kP:", 75, tft.height() / 2 - 25);
        tft.drawString("kI:", 75, tft.height() / 2);
        tft.drawString("kD:", 75, tft.height() / 2 + 25);
        if (!setPid) {
          tft.setTextColor(TFT_WHITE, TFT_BLACK);
          tft.drawString(String(kP), tft.width() / 2 - 5, tft.height() / 2 - 25);
          tft.drawString(String(kI), tft.width() / 2 - 5, tft.height() / 2);
          tft.drawString(String(kD), tft.width() / 2 - 5, tft.height() / 2 + 25);
        }
      }
      if (count == 2) {
        tft.drawRect(0, tft.height() / 2 - 13, 65, 50, TFT_WHITE);
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        tft.drawString("Reflow Profile", tft.width() / 2 - 50, tft.height() / 2 - 50);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.drawString("ID    :" + String(ID + 1), 75, tft.height() / 2 - 25);
        tft.drawString("SOAK  :" + String(profile[ID].soak) + String((char)247) + "C", 75, tft.height() / 2);
        tft.drawString("REFLOW:" + String(profile[ID].reflow) + String((char)247) + "C", 75, tft.height() / 2 + 25);
        tft.drawString("PEAK  :" + String(profile[ID].peak) + String((char)247) + "C", 75, tft.height() / 2 + 50);
      }
      if (count == 3) {
        tft.drawRect(0, tft.height() / 2 - 13, 65, 50, TFT_WHITE);
        tft.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
        tft.drawString("HEATUP:" + String(profile[ID].heatup_time) + " sec", 75, tft.height() / 2 - 50);
        tft.drawString("SOAK  :" + String(profile[ID].soak_time) + " sec", 75, tft.height() / 2 - 25);
        tft.drawString("RAMPUP:" + String(profile[ID].rampup_time) + " sec", 75, tft.height() / 2);
        tft.drawString("PEAK  :" + String(profile[ID].peak_time) + " sec", 75, tft.height() / 2 + 25);
        tft.drawString("RAMPDN:" + String(profile[ID].rampdn_time) + " sec", 75, tft.height() / 2 + 50);
      }
      if (count == 4) {
        tft.drawRect(0, tft.height() / 2 + 37, 65, 25, TFT_WHITE);
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        tft.drawString("Control Fan", tft.width() / 2 - 50, tft.height() / 2 - 50);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("OFF", 75, tft.height() / 2 - 25);
        tft.drawString("ON", 75, tft.height() / 2);
        tft.drawString("AUTO", 75, tft.height() / 2 + 25);
        tft.drawString("FAN RATE:", 75, tft.height() / 2 + 50);

        tft.drawString(String(fanRate) + "%", tft.width() / 2 + 60, tft.height() / 2 + 50);
      }
      if (count == 5) {
        tft.drawRect(0, tft.height() / 2 + 37, 65, 25, TFT_WHITE);
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        tft.drawString("Temp Setting", tft.width() / 2 - 50, tft.height() / 2 - 50);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("Deg C", 75, tft.height() / 2 - 25);
        tft.drawString("Deg F", 75, tft.height() / 2);
        tft.drawString("OFFSET:", 75, tft.height() / 2 + 25);

        tft.drawString(String(offset) + String((char)247) + "C", tft.width() / 2 + 45, tft.height() / 2 + 25);
      }
      if (count == 6) {
        tft.drawRect(0, tft.height() / 2 + 37, 65, 25, TFT_WHITE);
        tft.setTextColor(TFT_SKYBLUE, TFT_BLACK);
        tft.drawString("Control Buzzer", tft.width() / 2 - 50, tft.height() / 2 - 50);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawString("AUTO", 75, tft.height() / 2 - 20);
        tft.drawString("ONCE", 75, tft.height() / 2 + 5);
        tft.drawString("OFF", 75, tft.height() / 2 + 30);
      }
      if (count > 6) {
        encoder.clearCount();
      }
    }
    if (digitalRead(SW) == HIGH && pointer) {
      pointer = false;
      setPid = setProfile = setFan = setTemp =  setBuzz = false;
      if (count == 0) {
        home = true;
        pointer = true;
        menu = false;
        tft.fillScreen(TFT_BLACK);
      }
      if (count == 1) {
        setPid = true;
      }
      if (count == 2) {
        setProfile = true;
      }
      if (count == 3) {
        setFan = true;
      }
      if (count == 4) {
        setTemp = true;
      }
      if (count == 5) {
        setBuzz = true;
      }
      encoder.clearCount();
      delay(500);
    }
  }

  if (start) {
    tft.setTextColor(TFT_RED, TFT_BLACK);
    tft.setTextDatum(CC_DATUM);
    int padding = tft.textWidth("999", 2);
    tft.setTextPadding(padding);
    tft.drawString(String(profile[ID].reflow), tft.width() / 2 - 102, tft.height() / 2 - 50);
    tft.setTextColor(temp2clr(tempRead, 30, profile[ID].peak), TFT_BLACK);
    tft.drawString(String(tempRead), tft.width() / 2 - 102, tft.height() / 2 - 25);
    tft.setTextPadding(0);

    if (heaterState) {
      tft.fillRect(5, tft.height() / 2 - 5, 30, 30, TFT_GOLD);
      tft.setTextColor(TFT_BLACK);
      tft.drawString("H", tft.width() / 2 - 100, tft.height() / 2 + 10);
    }
    else {
      tft.drawRect(5, tft.height() / 2 - 5, 30, 30, TFT_WHITE);
      tft.setTextColor(TFT_GOLD);
      tft.drawString("H", tft.width() / 2 - 100, tft.height() / 2 + 10);
    }
    if (fanState) {
      tft.fillRect(5, tft.height() / 2 + 30, 30, 30, TFT_MAGENTA);
      tft.setTextColor(TFT_BLACK);
      tft.drawString("F", tft.width() / 2 - 100, tft.height() / 2 + 45);
    }
    else {
      tft.drawRect(5, tft.height() / 2 + 30, 30, 30, TFT_WHITE);
      tft.setTextColor(TFT_MAGENTA);
      tft.drawString("F", tft.width() / 2 - 100, tft.height() / 2 + 45);
    }

    // Graph
    int y0 = tft.height() - 15;
    int x0 = tft.width() / 2 - 60;

    tft.drawFastVLine(x0, 0, y0, TFT_WHITE);                   // x Axis = 120
    tft.drawFastHLine(x0, y0, x0 + 120, TFT_WHITE);            // y Axis = 180

    tft.setTextSize(1);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);

    for (int x = 18; x >= 0; x--) {
      for (int y = 12; y >= 0; y--) {
        tft.drawPixel(x0 + x * 10, y * 10, TFT_DARKGREY);

        tft.drawFastVLine(x0 + x * 10, y0, 5, TFT_YELLOW);
        tft.drawString(String(x), x0 + x * 30, y0 + 10);

        tft.drawFastHLine(x0 - 5, y * 10, 5, TFT_GREEN);
        tft.drawString(String((12 - 2 * y) * 20), x0 - 13, y * 20);
      }
    }

    int x1 = x0 + profile[ID].heatup_time / 2;
    int y1 = y0 - profile[ID].soak / 2;
    int x2 = x1 + profile[ID].soak_time / 2;
    int y2 = y0 - profile[ID].reflow / 2;
    int x3 = x2 + profile[ID].rampup_time / 2;
    int y3 = y0 - profile[ID].peak / 2;
    int x4 = x3 + profile[ID].peak_time / 2;
    int y4 = y3;
    int x5 = x3 + profile[ID].rampdn_time / 2;
    int y5 = y0;

    tft.drawLine(x0, y0 - 20, x1, y1, TFT_BLUE);
    tft.drawLine(x1, y1, x2, y2, TFT_YELLOW);
    tft.drawLine(x2, y2, x3, y3, TFT_ORANGE);
    tft.drawLine(x3, y3, x4, y4, TFT_RED);
    tft.drawLine(x4, y4, x5, y5, TFT_GREEN);

    if (tempRead > ambTemp) {
      plot.update();
      reflowProcess = true;
    }

    if (digitalRead(SW) == HIGH) {
      reflowProcess = false;
      startTimer = false;
      start = false;
      home = true;
      plotter = 0;
      delay(500);
      tft.fillScreen(TFT_BLACK);
    }

    tft.setTextSize(2);
    if (digitalRead(btn1) == LOW) {
      heaterState = !heaterState;
      if (heaterState) analogWrite(heater, 255);
      else analogWrite(heater, 0);
      delay(500);
      tft.fillRect(5, tft.height() / 2 - 5, 30, 30, TFT_BLACK);
    }
    if (digitalRead(btn2) == LOW) {
      fanState = !fanState;
      digitalWrite(fan, !fanState);
      delay(500);
      tft.fillRect(5, tft.height() / 2 + 30, 30, 30, TFT_BLACK);
    }
  }
}

void encFunc() {
  count = encoder.getCount();
  if (count > 0) count = count;
  if (count < 0) count = -count;
}

void plotFunc() {
  int y0 = tft.height() - 15;
  int x0 = tft.width() / 2 - 60;
  tft.drawPixel(x0 + plotter, y0 - tempRead / 2, TFT_WHITE);
  plotter = plotter + 1;
}

void rgb(int red, int green, int blue) {
  digitalWrite(r, red);
  digitalWrite(g, green);
  digitalWrite(b, blue);
}

unsigned long previousMillis;

void tempFunc() {
  if (millis() - previousMillis >= 1000) {
    previousMillis = millis();
    tempRead = thermocouple.readCelsius();
  }

  if (reflowProcess) {
    if (!startTimer) {
      setPoint = ambTemp;
      startTime = previousTime = millis();
      startTimer = true;
      Serial.println("SetPoint PID TempRead");
    }

    currentTime = millis();
    elapsedTime = (currentTime - previousTime) / 300;

    int t1 = profile[ID].heatup_time * 1000;
    int t2 = t1 + (profile[ID].soak_time * 1000);
    int t3 = t2 + (profile[ID].rampup_time * 1000);
    int t4 = t3 + (profile[ID].peak_time * 1000);

    if (currentTime - startTime < t1) {
      setPoint += (float(profile[ID].soak - 0) / profile[ID].heatup_time);
      if (setPoint > profile[ID].soak) setPoint = profile[ID].soak;
    }
    if (currentTime - startTime > t1 && currentTime - startTime < t2) {
      setPoint += (float(profile[ID].reflow - profile[ID].soak) / profile[ID].soak_time);
      if (setPoint > profile[ID].reflow) setPoint = profile[ID].reflow;
    }
    if (currentTime - startTime > t2 && currentTime - startTime < t3) {
      setPoint += (float(profile[ID].peak - profile[ID].reflow) / profile[ID].rampup_time);
      if (setPoint > profile[ID].peak) setPoint = profile[ID].peak;
    }
    if (currentTime - startTime > t3 && currentTime - startTime < t4) {
      setPoint = profile[ID].peak;
    }
    if (currentTime - startTime > t4) {
      setPoint = ambTemp;
      digitalWrite(fan, LOW);

      if (tempRead < ambTemp + 10) {
        reflowProcess = false;
        digitalWrite(fan, HIGH);
        iTerm = 0;
        plotter = 0;
        lastError = 0;
        startTimer = false;
        start = false;
        home = true;
        delay(500);
        tft.fillScreen(TFT_BLACK);
      }
    }

    error = setPoint - tempRead;

    // Proportional
    pTerm = error;

    // Integral
    if (error > -3 && error < 3)
      iTerm += error;

    // Anti-wind-up
    iTerm = constrain(iTerm, 0, 100);

    // Derivative
    dTerm = (error - lastError) / elapsedTime;

    // Calculate PID
    PID_value = kP * pTerm + kI * iTerm + kD * dTerm;

    // Deadband
    PID_value = constrain(PID_value, 0, 255);

    analogWrite(heater, PID_value);

    Serial.print(setPoint);
    Serial.print(",");
    Serial.print(PID_value);
    Serial.print(",");
    Serial.println(tempRead);

    lastError = error;
    previousTime = currentTime;

  }
}
