#include <EEPROM.h>


const int START_ADDRESS = 80;
const int END_MARKER = 42;

int pointsLogged = 0;

enum State {
  NONE,
  STANDBY,
  HEAT_AUTO,
  TIMER_ON
};

struct LogData {
  int hours;
  int minutes;
  State state;
  int solarTemp;
  int waterSensor;
  int waterTemp;
  bool pumpOn;
  bool solarOn;
};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  printLogData();
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(100);
}



void printLogData() {
  for (int i = 0; i < 1000; i++) {
    int address = START_ADDRESS + i * sizeof(LogData);

    int marker;
    EEPROM.get(address, marker);
    if (marker == END_MARKER) break;

    LogData data;
    EEPROM.get(address, data);

    String stateString;
    switch (data.state) {
      case STANDBY: stateString = "Standby"; break;
      case HEAT_AUTO: stateString = "Heat A."; break;
      case TIMER_ON: stateString = "Timer On"; break;
      case NONE: stateString = "OFF"; break;
    }

    Serial.print(data.hours);
    Serial.print(":");
    Serial.print(data.minutes);
    Serial.print("  State: ");
    Serial.print(stateString);
    Serial.print("  Solar temp: ");
    Serial.print(data.solarTemp);
    Serial.print("  Water sensor: ");
    Serial.print(data.waterSensor);
    Serial.print("  Water temp: ");
    Serial.print(data.waterTemp);
    Serial.print("  Pump on: ");
    Serial.print(data.pumpOn);
    Serial.print("  Solar on: ");
    Serial.println(data.solarOn);
  }
  Serial.println("");
}
