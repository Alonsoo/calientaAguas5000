#include <TimeLib.h>
#include <Wire.h>
#include <DS1307RTC.h>
#include <EEPROM.h>
#include "Nextion.h"


const int EEPROM_CHUNK_SIZE = 32;

const int TIMER_BUTTON_PIN = 4;
const int WATER_TEMP_SENSOR_PIN = 1;
const int SOLAR_TEMP_SENSOR_PIN = 0;
const int SOLAR_VALVE_PIN = 2;
const int PUMP_PIN = 3;

enum OperationMode {
  OFF,
  AUTO,
  TIMER
};

enum State {
  NONE,
  //DECIDE,
  STANDBY,
  //PROBE,
  HEAT_AUTO,
  TIMER_ON
};


// This struct makes it easier to connect variables to simple numeric controls on the settings page on the display and store them
struct NumericSetting {
  NexNumber *nexNumber;
  int eeAddress;

  NumericSetting(NexNumber* nexN, int addrIndex) {
    nexNumber = nexN;
    eeAddress = addrIndex * EEPROM_CHUNK_SIZE;
  }

  void setVal(int newValue) {
    //value = newValue;
    nexNumber->setValue(newValue);
    EEPROM.put(eeAddress, newValue);
  }

  int getVal() {
    //return value;
    int val;
    EEPROM.get(eeAddress, val);
    return (val);
  }
};


// These structs makes it easier to listen for changes in specific variables to trigger display updates
struct ChangeListener_base {
  void(* actionOnChange)();
  virtual bool changed() = 0;
  virtual void updateValue() = 0;
  ChangeListener_base(void(* action)()) {
    actionOnChange = action;
  }
};

template<typename T>
struct ChangeListener : ChangeListener_base {
  T* currentValue;
  T previousValue;
  ChangeListener(T* ptr, T val, void(* action)()) : ChangeListener_base(action) {
    currentValue = ptr;
    previousValue = val;
  }

  bool changed() {
    return previousValue != *currentValue;
  }
  void updateValue() {
    previousValue = *currentValue;
  }
};



// Temperature Waveform stuff

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

struct TimedReading {
  int val;
  long takenAt;
};


struct Grapher {
  static const int MIN_READING_INTERVAL = 5; //minutes

  // We will take water temperature readings at most every 5 minutes and want to store 12hrs of data so we will need at most (60 / 5) * 12 = 144 readings
  static const int MAX_READINGS = 60;//(60 / MIN_READING_INTERVAL) * 12;

  int readingsTaken = 0;
  TimedReading readings[MAX_READINGS];

  int channel;

  Grapher(int chnl) {
    channel = chnl;
  }

  void registerData(TimedReading reading) {
    long lastReadingTakenAt = readingsTaken > 0 ? readings[readingsTaken - 1].takenAt : 0;
    long ellapsedTime = now() - lastReadingTakenAt;

    if (ellapsedTime >= 1 || readingsTaken == 0) {
      //if (ellapsedTime > minToSec(MIN_READING_INTERVAL) || readingsTaken == 0) {
      if (readingsTaken < MAX_READINGS) {
        //Store new reading on next available space on array
        readings[readingsTaken] = reading;
        readingsTaken++;
      }
      else {
        //Shift array to the left and store new reading on the last spot
        for (int i = 0; i < (MAX_READINGS - 1); i++) {
          readings[i] = readings[i + 1];
        }
        readings[MAX_READINGS - 1] = reading;
      }
    }
  }

  void displayWaveform(long graphStartTime, long waveformTimeWidth, int waveformWidth) {
    if (readingsTaken == 0) return;

    long numDataPoints = ((readings[readingsTaken - 1].takenAt - graphStartTime) / (float) waveformTimeWidth) * waveformWidth;

    if (numDataPoints == 0) {
      Serial.println("No Data Points");
      return;
    }

    //Interpolate over current teperature readings to produce regular-intervaled data points
    int dataPoints[numDataPoints];
    for (int i = 0; i < numDataPoints; i++) {
      float relativePointTime = ((i / (float)waveformWidth) * waveformTimeWidth);

      //      Serial.print("relative poin: ");
      //      Serial.print((i / (float)waveformWidth) * waveformTimeWidth);
      //      Serial.print(" relative point time: ");
      //      Serial.print(relativePointTime);
      //      Serial.print(" realtive first reading at: ");
      //      Serial.println(readings[0].takenAt - graphStartTime);
      bool debug_found = false;
      if (readings[0].takenAt - graphStartTime >= relativePointTime) {
        //data point is from before any measurments were made, send 1
        dataPoints[i] = 1;
        debug_found = true;
        //Serial.println("before");
      }
      else {
        //data points lies between measuements, serach for closest ones and interpolate
        for (int j = 0; j < readingsTaken - 1; j++) {
          if (readings[j + 1].takenAt - graphStartTime >= relativePointTime) {
            //Interpolate dataPoint value at pointTime using closest temp readings
            float val = interpolateData(graphStartTime, relativePointTime, readings[j], readings[j + 1]);

            //Scale and clip value
            val = scaleData(val);
            val = min(max(val, 1), 136);
            dataPoints[i] = (int) val;

            debug_found = true;
            break;
          }
        }
      }
      if (!debug_found) {
        Serial.println("WTF");
      }
    }

    //parse dataPoints into string
    char dataString[2 * numDataPoints + 1];
    dataString[0] = 0;
    for (int i = 0; i < numDataPoints; i++) {
      strcat(dataString, (char*) &dataPoints[i]);
    }

    //Clear waveform channel instruction
    char clearString[20];
    sprintf(clearString, "cle 2,%d\xFF\xFF\xFF", channel);

    int digits = numDataPoints >= 100 ? 3 : (numDataPoints >= 10 ? 2 : 1);
    char instructionString[21 + digits];
    sprintf(instructionString, "addt 2,%d,%d\xFF\xFF\xFF", channel, numDataPoints);

    //Send dataPoints to nextion display
    Serial2.write(clearString);
    delay(50);
    Serial2.write(instructionString);
    delay(20);
    Serial2.write(dataString);
    delay(750);
  }

  virtual float interpolateData(long graphStartTime, float t, TimedReading before, TimedReading after) = 0;
  virtual int scaleData(float val) = 0;
};

struct TempGrapher : Grapher {
  static const int WAVEFORM_MIN_TEMP = 20;
  static const int WAVEFORM_MAX_TEMP = 40;
  static const int WAVEFORM_HEIGHT = 136; //px

  TempGrapher(int chnl) : Grapher(chnl) {}

  float interpolateData(long graphStartTime, float t, TimedReading before, TimedReading after) {
    float interp = (t - (before.takenAt - graphStartTime)) / (float) (after.takenAt - before.takenAt);
    //    Serial.print("interp");
    //    Serial.println(interp);
    return ((after.val - before.val) * interp) + before.val;
  }

  int scaleData(float val) {
    return fmap(val, WAVEFORM_MIN_TEMP, WAVEFORM_MAX_TEMP, 0, WAVEFORM_HEIGHT);
  }
};

struct PowerGrapher : Grapher {
  static const int ON_HEIGHT = 85;
  static const int OFF_HEIGHT = 51;

  PowerGrapher(int chnl) : Grapher(chnl) {}

  float interpolateData(long graphStartTime, float t, TimedReading before, TimedReading after) {
    return t - (before.takenAt - graphStartTime) < (after.takenAt - graphStartTime) - t ? before.val : after.val;
  }

  int scaleData(float val) {
    return val == 1 ? ON_HEIGHT : OFF_HEIGHT;
  }
};


struct Waveform {
  static const long WAVEFORM_TIME_INTERVAL = 10; // 10 seconds
  static const long WAVEFORM_TIME_WIDTH = 60; //60 seconds
  static const int WAVEFORM_WIDTH = 288; //px

  static const int NUM_GRAPHERS = 3;

  Grapher *graphers[NUM_GRAPHERS];

  void updateWaveform() {

    long earliestReading = 2147483647L; //maximum value for long
    long latestReading = 0;


    for (int i = 0; i < NUM_GRAPHERS; i++) {
      if (graphers[i]->readingsTaken > 0) {
        earliestReading = min(earliestReading, graphers[i]->readings[0].takenAt);
        latestReading = max(latestReading, graphers[i]->readings[graphers[i]->readingsTaken - 1].takenAt);
      }
    }

    if (earliestReading == 2147483647L) return; // stop if there arent any readings yet

    long graphStartTime;
    long earliestReadingRoundedDown = earliestReading - (earliestReading % WAVEFORM_TIME_INTERVAL); //round down to the nearest graph time interval

    if (latestReading - earliestReadingRoundedDown > WAVEFORM_TIME_WIDTH) {
      long graphEndTime = latestReading - (latestReading % WAVEFORM_TIME_INTERVAL) + WAVEFORM_TIME_INTERVAL; //round up to the nearest graph time interval
      graphStartTime = graphEndTime - WAVEFORM_TIME_WIDTH;
    }
    else {
      graphStartTime = earliestReadingRoundedDown;
    }

    // Display individual graphs
    for (Grapher *grapher : graphers) {
      grapher->displayWaveform(graphStartTime, WAVEFORM_TIME_WIDTH, WAVEFORM_WIDTH);
    }
  }

};


// Nextion pages
int currentPage;
NexPage page0 = NexPage(0, 0, NULL);
NexPage page1 = NexPage(1, 0, NULL);
NexPage page3 = NexPage(3, 0, NULL);
NexPage page4 = NexPage(4, 0, NULL);

// Main screen components
NexSlider sTargetTemp = NexSlider(0, 1, "sTargetTemp");
NexNumber nTargetTemp = NexNumber(0, 4, "nTargetTemp");
NexNumber nWaterTemp = NexNumber(0, 6, "nWaterTemp");
NexRadio rOff = NexRadio(0, 8, "rOff");
NexRadio rAuto = NexRadio(0, 9, "rAuto");
NexRadio rTimer = NexRadio(0, 10, "rTimer");
NexCheckbox cSolar = NexCheckbox(0, 17, "cSolar");
NexCheckbox cHeater = NexCheckbox(0, 18, "cHeater");

//Settings Page Components
NexButton bTempTolMinus = NexButton(1, 3, "bTempTolM");
NexButton bTempTolPlus = NexButton(1, 1, "bTempTolP");
NexButton bSolarTolMinus = NexButton(1, 6, "bSolarTolM");
NexButton bSolarTolPlus = NexButton(1, 8, "bSolarTolP");
NexButton bSolarOffsetMinus = NexButton(1, 10, "bSolarOffsetM");
NexButton bSolarOffsetPlus = NexButton(1, 12, "bSolarOffsetP");
NexButton bProbeTimeMinus = NexButton(1, 14, "bProbeTimeM");
NexButton bProbeTimePlus = NexButton(1, 16, "bProbeTimeP");
NexButton bStandbyTimeMinus = NexButton(3, 3, "bStandbyTimeM");
NexButton bStandbyTimePlus = NexButton(3, 5, "bStandbyTimeP");
NexButton bTimerOnTimeMinus = NexButton(3, 8, "bTimerOnTimeM");
NexButton bTimerOnTimePlus = NexButton(3, 10, "bTimerOnTimeP");

NexNumber nTempTol = NexNumber(1, 4, "nTempTol");
NexNumber nSolarTol = NexNumber(1, 7, "nSolarTol");
NexNumber nSolarOffset = NexNumber(1, 11, "nSolarOffset");
NexNumber nProbeTime = NexNumber(1, 15, "nProbeTime");
NexNumber nStandbyTime = NexNumber(3, 4, "nStandbyTime");
NexNumber nTimerOnTime = NexNumber(3, 9, "nTimerOnTime");

NexText tTime = NexText(3, 12, "tTime");

// Info Page Components
NexNumber nInfoWaterTemp = NexNumber(4, 23, "nInfoWaterTemp");
NexNumber nInfoSolarTemp = NexNumber(4, 26, "nInfoSolarTemp");
NexText tInfoState = NexText(4, 19, "tInfoState");
NexText tInfoPumpOn = NexText(4, 20, "tInfoPumpOn");
NexText tInfoSolarOn = NexText(4, 21, "tInfoSolarOn");
NexText tInfoHeaterOn = NexText(4, 22, "tInfoHeaterOn");

NexWaveform wInfoTemp = NexWaveform(4, 2, "wInfoTemp");
NexText tInfoTimeLabels[] = { NexText(4, 7, "t1"),
                              NexText(4, 8, "t2"),
                              NexText(4, 9, "t3"),
                              NexText(4, 10, "t4"),
                              NexText(4, 11, "t5"),
                              NexText(4, 12, "t6")
                            };


// Register objects to the touch event list.
NexTouch *nex_listen_list[] = {
  &page0,
  &page1,
  &page3,
  &page4,
  &sTargetTemp,
  &rOff,
  &rAuto,
  &rTimer,
  &cSolar,
  &cHeater,
  &bTempTolMinus,
  &bTempTolPlus,
  &bSolarTolMinus,
  &bSolarTolPlus,
  &bSolarOffsetMinus,
  &bSolarOffsetPlus,
  &bProbeTimeMinus,
  &bProbeTimePlus,
  &bStandbyTimeMinus,
  &bStandbyTimePlus,
  &bTimerOnTimeMinus,
  &bTimerOnTimePlus,
  NULL
};


OperationMode operationMode;
State state = STANDBY;

NumericSetting tempTolerance = NumericSetting(&nTempTol, 0); //{2, nTempTol, 0}; // For hysteresis
NumericSetting solarTempTolerance = NumericSetting(&nSolarTol, 1); //{2, nSolarTol, 1}; // For hysteresis
NumericSetting solarTempOffset = NumericSetting(&nSolarOffset, 2); //{2, nSolarOffset, 2}; // Solar wont turn on unless solar temperature is solarTempOffset degrees above water temp

const int SOLAR_TEMP_LOWER_THRESHOLD = 25;
//TODO: decide actual fault temperatures
const int TEMP_SENSOR_LOW_FAULT = -1;
const int TEMP_SENSOR_HIGH_FAULT = 49;

int targetTemp;
int waterTemp;
int solarTemp;

int maxTemp;
int minTemp;

bool sensorFault = false;

bool solarEnabled;
bool heaterEnabled;

bool solarOn = false;
bool pumpOn = false;
bool heaterOn = false;

NumericSetting probeTime = NumericSetting(&nProbeTime, 3); //{5, nProbeTime, 3}; // in seconds
//NumericSetting standbyTime = NumericSetting(&nStandbyTime, 4); //{10, nStandbyTime, 4};// in minutes
NumericSetting timerOnTime = NumericSetting(&nTimerOnTime, 5); // in hours

long waterTempRecordedAt = -1; // initialize to -1 to ensure water temp readig is interpreted as stale on the first loop since there isnt a water temperature reading yet
//long waterTempReadingLifetime = 8 * 1000UL; //10 * 60 * 1000UL; // 10 minutes
NumericSetting waterTempReadingLifetime = NumericSetting(&nStandbyTime, 4);

//bool timerStarted = false;
//long timerStartedAt;

NumericSetting *settings[] = {
  &tempTolerance,
  &solarTempTolerance,
  &solarTempOffset,
  &probeTime,
  &waterTempReadingLifetime,
  &timerOnTime
  //&standbyTime
};



// Value Change Listeners
struct ChangeListener<int> waterTempCL = ChangeListener<int>(&waterTemp, waterTemp, displayUpdateWaterTemp);
struct ChangeListener<int> solarTempCL = ChangeListener<int>(&solarTemp, solarTemp, displayUpdateSolarTemp);
struct ChangeListener<State> stateCL = ChangeListener<State>(&state, state, displayUpdateState);
struct ChangeListener<bool> pumpOnCL = ChangeListener<bool>(&pumpOn, pumpOn, displayUpdatePumpOn);
struct ChangeListener<bool> solarOnCL = ChangeListener<bool>(&solarOn, solarOn, displayUpdateSolarOn);
struct ChangeListener<bool> heaterOnCL = ChangeListener<bool>(&heaterOn, heaterOn, displayUpdateHeaterOn);

// Register objects to the change listen list
struct ChangeListener_base *changeListenList[] = {
  &waterTempCL,
  &solarTempCL,
  &stateCL,
  &pumpOnCL,
  &solarOnCL,
  &heaterOnCL
};


TempGrapher waterTempGrapher = TempGrapher(0);
TempGrapher solarTempGrapher = TempGrapher(1);
PowerGrapher solarOnGrapher = PowerGrapher(2);
Waveform waveform = Waveform();


void setup() {
  Serial.begin(9600);

  setSyncProvider(RTC.get);   // the function to get the time from the RTC
  if (timeStatus() != timeSet)
    Serial.println("Unable to sync with the RTC");
  else
    Serial.println("RTC has set the system time");

  pinMode(TIMER_BUTTON_PIN, INPUT_PULLUP);
  pinMode(SOLAR_VALVE_PIN, OUTPUT);
  pinMode(PUMP_PIN, OUTPUT);
  pinMode(13, INPUT);

  waveform.graphers[0] = &waterTempGrapher;
  waveform.graphers[1] = &solarTempGrapher;
  waveform.graphers[2] = &solarOnGrapher;

  nexInit();

  //attach ui component press/release callback functions
  page0.attachPush(updatePage0);
  page1.attachPush(updatePage1);
  page3.attachPush(updatePage3);
  page4.attachPush(updatePage4);

  sTargetTemp.attachPop(sTempCallback);
  sTargetTemp.attachPush(sTempCallback);
  rOff.attachPush(rOffPushCallback);
  rAuto.attachPush(rAutoPushCallback);
  rTimer.attachPush(rTimerPushCallback);
  cSolar.attachPush(cSolarPushCallback);
  cHeater.attachPush(cHeaterPushCallback);

  bTempTolMinus.attachPush(numericSettingMinusCallback, &tempTolerance);
  bTempTolPlus.attachPush(numericSettingPlusCallback, &tempTolerance);
  bSolarTolMinus.attachPush(numericSettingMinusCallback, &solarTempTolerance);
  bSolarTolPlus.attachPush(numericSettingPlusCallback, &solarTempTolerance);
  bSolarOffsetMinus.attachPush(numericSettingMinusCallback, &solarTempOffset);
  bSolarOffsetPlus.attachPush(numericSettingPlusCallback, &solarTempOffset);
  bProbeTimeMinus.attachPush(numericSettingMinusCallback, &probeTime);
  bProbeTimePlus.attachPush(numericSettingPlusCallback, &probeTime);
  bStandbyTimeMinus.attachPush(numericSettingMinusCallback, &waterTempReadingLifetime);
  bStandbyTimePlus.attachPush(numericSettingPlusCallback, &waterTempReadingLifetime);
  bTimerOnTimeMinus.attachPush(numericSettingMinusCallback, &timerOnTime);
  bTimerOnTimePlus.attachPush(numericSettingPlusCallback, &timerOnTime);

  setMaxTemp(45);
  setMinTemp(25);
  setTargetTemp(30, true);
  setOperationMode(OFF);
  setSolarEnabled(true);
  setHeaterEnabled(false);

  updatePage0(NULL);
  updatePage1(NULL);
  updatePage3(NULL);
  updatePage4(NULL);

  currentPage = 0;
  Serial2.write("page 0\xFF\xFF\xFF");
  delay(50);
}


void loop() {
  nexLoop(nex_listen_list);

  checkSensorFaults();
  updateSolarTemp();
  timerButtonListen();

  displayUpdateClock(false);

  heaterOn = heaterEnabled;

  static long timerStartedAt = 0;
  if (state != TIMER_ON) timerStartedAt = 0;


  switch (state) {
    case STANDBY:
      if (!waterTempReadingIsStale() && waterTemp < targetTemp - tempTolerance.getVal())
        state = HEAT_AUTO;
      else {
        solarOn = false;
        pumpOn = heaterOn;
      }
      break;

    case HEAT_AUTO:
      //run water and choose how to heat it until target temp. is reached
      if (solarEnabled) {
        if (waterTemp < solarTemp - solarTempOffset.getVal() - solarTempTolerance.getVal())
          solarOn = true;
        if (waterTemp > solarTemp - solarTempOffset.getVal())
          solarOn = false;
      }
      else {
        solarOn = false;
      }

      pumpOn = solarOn || heaterOn;

      if (!waterTempReadingIsStale() && waterTemp > targetTemp + tempTolerance.getVal())
        state = STANDBY;
      break;

    case TIMER_ON:
      //turn pump on and choose how to heat it
      solarOn = solarEnabled;
      pumpOn = true;

      if (timerStartedAt == 0)
        timerStartedAt = now();

      //Wait for timer to finish and set Op Mode to OFF
      //TODO: replace timer conversion on production
      //if(now() - timerStartedAt > hrToSec(timerOnTime.getVal())){
      if (now() - timerStartedAt > timerOnTime.getVal()) {
        setOperationMode(OFF);
        timerStartedAt = 0; //reset timer
      }
      break;

    default:
    case NONE:
      //turn everything off
      solarOn = false;
      pumpOn = false;
      //timerStarted = false;
      break;
  }


  //If conditions are right and water temperature reading is stale, then override pump to on so we can probe the water temperature later
  //TODO: RTC add nightime conditioning
  bool isNightime = hour() >= 20 || hour() < 7;
  if (operationMode == AUTO && solarTemp > SOLAR_TEMP_LOWER_THRESHOLD && waterTempReadingIsStale() && !isNightime)
    pumpOn = true;


  // If pump has been on long enough, update water temperature
  static long pumpTurnedOnAt = 0;
  static bool prevPumpOn = false;

  if (pumpOn && !prevPumpOn)
    pumpTurnedOnAt = now();

  long pumpOnFor = pumpOn ? now() - pumpTurnedOnAt : 0;

  //TODO: change time convertion for production
  if (pumpOn && pumpOnFor > probeTime.getVal())
    updateWaterTemp();

  prevPumpOn = pumpOn;

  //Control hardware
  digitalWrite(SOLAR_VALVE_PIN, solarOn);
  digitalWrite(PUMP_PIN, pumpOn);


  //Graph solarOn value
  TimedReading solarOnReading = {solarOn, now()};
  solarOnGrapher.registerData(solarOnReading);


  //Listen for changes to trigger display updates and update change listeners
  for (ChangeListener_base *cl : changeListenList) {
    if (cl->changed()) {
      cl->actionOnChange();
    }
    cl->updateValue();
  }


  // Update waveform
  //    static long waveformLastUpdatedAt = 0;
  //    long ellapsedTime = now() - waveformLastUpdatedAt;
  //    if ((ellapsedTime >= 5 || waveformLastUpdatedAt == 0) && currentPage == 4) {
  //      Serial.println("update");
  //      waveform.updateWaveform();
  //      waveformLastUpdatedAt = now();
  //
  //      Serial.println("done");
  //    }

  delay(50);
}


// Timer Button
const int BUTTON_BOUNCE_COOLDOWN = 100; //ms

void timerButtonListen() {
  static int prevVal = LOW;
  static long switchedOnAt = 0;

  int val = digitalRead(TIMER_BUTTON_PIN);
  if (val == HIGH && prevVal == LOW && (millis() - switchedOnAt) > BUTTON_BOUNCE_COOLDOWN) {
    if (operationMode == OFF) setOperationMode(TIMER);
    if (operationMode == TIMER) setOperationMode(OFF);
    switchedOnAt = millis();
    Serial.println("Button Pressed");
  }
  prevVal = val;
}



// Temperature Sensor Reading and Fault Checking

void updateWaterTemp() {
  waterTemp = readTempSensor(WATER_TEMP_SENSOR_PIN);
  waterTempRecordedAt = now();

  TimedReading reading = {waterTemp, now()};
  waterTempGrapher.registerData(reading);
  //When we upgrade waterTemp to float remember to always truncate it to one decimal place so sensor noise wont trigger display updates every frame
}

bool waterTempReadingIsStale() {
  long waterTempReadingAge = now() - waterTempRecordedAt;
  return waterTempReadingAge > waterTempReadingLifetime.getVal() || waterTempRecordedAt == -1;
}

void updateSolarTemp() {
  solarTemp = readTempSensor(SOLAR_TEMP_SENSOR_PIN);

  TimedReading reading = {solarTemp, now()};
  solarTempGrapher.registerData(reading);
  //When we upgrade solarTemp to float remember to always truncate it to one decimal place so sensor noise wont trigger display updates every frame
}

float readTempSensor(int pin) {
  int val = analogRead(pin);
  float volt = fmap(val, 0, 1023, 0, 5);
  float res =  (5 * 10000L) / volt - 10000L ;
  float temp = 238 - 23.1 * log(res);
  return temp;
}

void checkSensorFaults() {
  int wTemp = readTempSensor(1);
  int sTemp = readTempSensor(0);

  sensorFault = wTemp < TEMP_SENSOR_LOW_FAULT || wTemp > TEMP_SENSOR_HIGH_FAULT || sTemp < TEMP_SENSOR_LOW_FAULT || sTemp > TEMP_SENSOR_HIGH_FAULT;

  if (sensorFault && operationMode == AUTO)
    setOperationMode(OFF);

  static bool prevSensorFault = false;
  if (sensorFault != prevSensorFault) //Only update display when value changes, otherwise nextion lags
    displayUpdateSensorFault();
  prevSensorFault = sensorFault;
}



// Display page updates

void updatePage0(void *ptr) {
  currentPage = 0;
  nWaterTemp.setValue(waterTemp);
  nTargetTemp.setValue(targetTemp);
  sTargetTemp.setValue(targetTemp);
  rOff.setValue(operationMode == OFF);
  rAuto.setValue(operationMode == AUTO);
  rTimer.setValue(operationMode == TIMER);
  cSolar.setValue(solarEnabled);
  cHeater.setValue(heaterEnabled);
  displayUpdateSensorFault();
}

void updatePage1(void *ptr) {
  currentPage = 1;
  nTempTol.setValue(tempTolerance.getVal());
  nSolarTol.setValue(solarTempTolerance.getVal());
  nSolarOffset.setValue(solarTempOffset.getVal());
  nProbeTime.setValue(probeTime.getVal());
}

void updatePage3(void *ptr) {
  currentPage = 3;
  nStandbyTime.setValue(waterTempReadingLifetime.getVal());
  nTimerOnTime.setValue(timerOnTime.getVal());
  displayUpdateClock(true);
}

void updatePage4(void *ptr) {
  currentPage = 4;
  nInfoWaterTemp.setValue(waterTemp);
  nInfoSolarTemp.setValue(solarTemp);

  displayUpdatePumpOn();
  displayUpdateSolarOn();
  displayUpdateHeaterOn();
  displayUpdateState();

  //Clear waveform
  //  Serial2.write("cle 2,0\xFF\xFF\xFF");
  //  delay(50);
  //  Serial2.write("cle 2,1\xFF\xFF\xFF");
  //  delay(50);
  //  Serial2.write("cle 2,2\xFF\xFF\xFF");
  //  waveform.updateWaveform();
  //waveform stuff
  //waterTempGrapher.displayWaveform();
  //solarTempGrapher.displayWaveform();
}


void displayUpdateWaterTemp() {
  nWaterTemp.setValue(waterTemp);
  nInfoWaterTemp.setValue(waterTemp);
}

void displayUpdateSolarTemp() {
  nInfoSolarTemp.setValue(solarTemp);
}

void displayUpdateSensorFault() {
  if (sensorFault)
    Serial2.write("vis tCheckSensors,1\xFF\xFF\xFF");
  else
    Serial2.write("vis tCheckSensors,0\xFF\xFF\xFF");
}

void displayUpdateState() {
  String stateString;
  switch (state) {
    //case DECIDE: stateString = "Decide"; break;
    case STANDBY: stateString = "Standby"; break;
    //case PROBE: stateString = "Probe"; break;
    case HEAT_AUTO: stateString = "Heat A."; break;
    case TIMER_ON: stateString = "Timer On"; break;
    case NONE: stateString = "OFF"; break;
  }
  tInfoState.setText(stateString.c_str());
}

const int greenPco = 7716;
const int redPco = 57440;

void displayUpdatePumpOn() {
  tInfoPumpOn.setText(pumpOn ? "On" : "Off");
  tInfoPumpOn.Set_font_color_pco(pumpOn ? greenPco : redPco);
}

void displayUpdateSolarOn() {
  tInfoSolarOn.setText(solarOn ? "On" : "Off");
  tInfoSolarOn.Set_font_color_pco(solarOn ? greenPco : redPco);
}

void displayUpdateHeaterOn() {
  tInfoHeaterOn.setText(heaterOn ? "On" : "Off");
  tInfoHeaterOn.Set_font_color_pco(heaterOn ? greenPco : redPco);
}

void displayUpdateClock(bool force) {
  static int prevMinutes = 0;
  if (force || minute() != prevMinutes) {
    char text[6];
    sprintf(text, "%2d:%02d", hour(), minute());
    tTime.setText(text);
  }
  prevMinutes = minute();
}

// Radio button callbacks

void rOffPushCallback(void *ptr) {
  setOperationMode(OFF);
}

void rAutoPushCallback(void *ptr) {
  setOperationMode(AUTO);
}

void rTimerPushCallback(void *ptr) {
  setOperationMode(TIMER);
}



// Checkbox callbacks
void cSolarPushCallback(void *ptr) {
  uint32_t enabled;
  cSolar.getValue(&enabled);
  setSolarEnabled(enabled);
}

void cHeaterPushCallback(void *ptr) {
  uint32_t enabled;
  cHeater.getValue(&enabled);
  setHeaterEnabled(enabled);
}



// Temperature slider callbacks

/*When slider is pressed moved or released update target temperature to slider value*/
void sTempCallback(void *ptr) {
  updateTargetTempFromDisplay();
}


/*Update target temperature to slider value */
void updateTargetTempFromDisplay() {
  uint32_t temp = minTemp - 1;
  sTargetTemp.getValue(&temp);
  setTargetTemp(temp, false);
}


// Settings callbacks

void numericSettingMinusCallback(void *ptr) {
  NumericSetting *numericSetting = (NumericSetting *) ptr;
  numericSetting->setVal(numericSetting->getVal() - 1);
  //numericSetting->nexNumber->setValue(numericSetting->getVal());
}

void numericSettingPlusCallback(void *ptr) {
  NumericSetting *numericSetting = (NumericSetting *) ptr;
  numericSetting->setVal(numericSetting->getVal() + 1);
  //numericSetting->nexNumber->setValue(numericSetting->getVal());
}


// Setters

/*Set target temperature and update display accordingly, only update slider if requierd to avoid display glitching*/
void setTargetTemp(int temp, bool updateSlider) {
  targetTemp = constrain(temp, minTemp, maxTemp);
  nTargetTemp.setValue(targetTemp);
  if (updateSlider) {
    sTargetTemp.setValue(targetTemp);
  }
}

/*Set maximum allowed target temperature and update slider boundaries*/
void setMaxTemp(int temp) {
  maxTemp = temp;
  sTargetTemp.setMaxval(temp);
}

/*Set minimum allowed target temperature and update slider boundaries*/
void setMinTemp(int temp) {
  minTemp = temp;
  sTargetTemp.setMinval(temp);
}


/*Set operation mode and update display accordingly*/
void setOperationMode(OperationMode mode) {
  if (mode == AUTO && sensorFault) return;
  switch (mode) {
    case OFF:
      state = NONE;
      break;

    case AUTO:
      if (operationMode != mode)
        state = STANDBY;
      break;

    case TIMER:
      state = TIMER_ON;
      break;
  }
  operationMode = mode;
  rOff.setValue(mode == OFF ? 1 : 0);
  rAuto.setValue(mode == AUTO ? 1 : 0);
  rTimer.setValue(mode == TIMER ? 1 : 0);
}

void setSolarEnabled(bool enabled) {
  solarEnabled = enabled;
  cSolar.setValue(enabled);
}

void setHeaterEnabled(bool enabled) {
  heaterEnabled = enabled;
  cHeater.setValue(enabled);
}


// Time Conversions

long secToMillis(int seconds) {
  return seconds * 1000UL;
}

long minToMillis(int minutes) {
  return minutes * 60 * 1000UL;
}

long hrToMillis(int hrs) {
  return hrs * 60 * 60 * 1000UL;
}

long minToSec(int minutes) {
  return minutes * 60UL;
}

long hrToSec(int hrs) {
  return hrs * 60 * 60UL;
}
