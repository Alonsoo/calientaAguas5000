#include <EEPROM.h>
#include "Nextion.h"

const int EEPROM_CHUNK_SIZE = 32;

enum OperationMode {
  OFF,
  AUTO,
  ON
};

enum State {
  NONE,
  DECIDE,
  STANDBY,
  PROBE,
  HEAT_AUTO,
  HEAT_FORCE
};

// Nextion pages
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
NexRadio rOn = NexRadio(0, 10, "rOn");
NexCheckbox cSolar = NexCheckbox(0, 17, "cSolar");
NexCheckbox cHeater = NexCheckbox(0, 18, "cCaldera");

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

NexNumber nTempTol = NexNumber(1, 4, "nTempTol");
NexNumber nSolarTol = NexNumber(1, 7, "nSolarTol");
NexNumber nSolarOffset = NexNumber(1, 11, "nSolarOffset");
NexNumber nProbeTime = NexNumber(1, 15, "nProbeTime");
NexNumber nStandbyTime = NexNumber(3, 4, "nStandbyTime");

// Info Page Components
NexNumber nInfoWaterTemp = NexNumber(4, 23, "nInfoWaterTemp");
NexNumber nInfoSolarTemp = NexNumber(4, 26, "nInfoSolarTemp");
NexText tInfoState = NexText(4, 19, "tInfoState");
NexText tInfoPumpOn = NexText(4, 20, "tInfoPumpOn");
NexText tInfoSolarOn = NexText(4, 21, "tInfoSolarOn");
NexText tInfoHeaterOn = NexText(4, 22, "tInfoHeaterOn");

NexWaveform wInfoTemp = NexWaveform(4, 2, "wInfoTemp");
NexText tInfoTimeLabels[] = {NexText(4, 7, "t1"),
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
  &rOn,
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
  NULL
};


typedef struct {
  int val;
  unsigned long takenAt;
} TimedReading;


// This struct makes it easier to connect variables to simple numeric controls on the settings page on the display
typedef struct {
  int value;
  NexNumber nexNumber;
} NumericSetting;

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


OperationMode operationMode;
State state = STANDBY;

//int tempTolerance = 2; // For hysteresis
//int solarTempTolerance = 2;// For hysteresis
//int solarTempOffset = 2; // Solar wont turn on unless solar temperature is solarTempOffset degrees above water temp

NumericSetting tempTolerance = {2, nTempTol}; // For hysteresis
NumericSetting solarTempTolerance = {2, nSolarTol}; // For hysteresis
NumericSetting solarTempOffset = {2, nSolarOffset}; // Solar wont turn on unless solar temperature is solarTempOffset degrees above water temp

int targetTemp;
int waterTemp;
int solarTemp;

int maxTemp;
int minTemp;

bool solarEnabled;
bool heaterEnabled;

bool solarOn = false;
bool pumpOn = false;
bool heaterOn = false;

long waterTempRecorededAt = -1; // initialize to -1 to ensure water temp readig is interpreted as stale on the first loop since there isnt a water temperature reading yet
long waterTempReadingLifetime = 8 * 1000UL; //10 * 60 * 1000UL; // 10 minutes

//long probeTime = 5 * 1000UL; //30 * 1000UL; //30 seconds
//long standbyTime = 10 * 1000UL; //15 * 60 * 1000UL; //15 minutes

NumericSetting probeTime = {5, nProbeTime}; // in seconds
NumericSetting standbyTime = {10, nStandbyTime};// in minutes

bool timerStarted = false;
long timerStartedAt;

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



void setup() {
  Serial.begin(9600);
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
  rOn.attachPush(rOnPushCallback);
  cSolar.attachPush(cSolarPushCallback);
  cHeater.attachPush(cCalderaPushCallback);

  bTempTolMinus.attachPush(numericSettingMinusCallback, &tempTolerance);
  bTempTolPlus.attachPush(numericSettingPlusCallback, &tempTolerance);
  bSolarTolMinus.attachPush(numericSettingMinusCallback, &solarTempTolerance);
  bSolarTolPlus.attachPush(numericSettingPlusCallback, &solarTempTolerance);
  bSolarOffsetMinus.attachPush(numericSettingMinusCallback, &solarTempOffset);
  bSolarOffsetPlus.attachPush(numericSettingPlusCallback, &solarTempOffset);
  bProbeTimeMinus.attachPush(numericSettingMinusCallback, &probeTime);
  bProbeTimePlus.attachPush(numericSettingPlusCallback, &probeTime);
  bStandbyTimeMinus.attachPush(numericSettingMinusCallback, &standbyTime);
  bStandbyTimePlus.attachPush(numericSettingPlusCallback, &standbyTime);

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
}


void loop() {
  nexLoop(nex_listen_list);

  updateSolarTemp();

  //  Serial.print("Solar: ");
  //  Serial.print(solarTemp);
  //  Serial.print(" Water: ");
  //  Serial.print(waterTemp);
  //
  //  String stateString;
  //  switch (state) {
  //    case DECIDE: stateString = "Decide"; break;
  //    case STANDBY: stateString = "Standby"; break;
  //    case PROBE: stateString = "Probe"; break;
  //    case HEAT_AUTO: stateString = "Heat A."; break;
  //    case HEAT_FORCE: stateString = "Heat F."; break;
  //    case NONE: stateString = "None"; break;
  //  }
  //
  //  Serial.print(" State: ");
  //  Serial.print(stateString);

  long timeEllapsed;

  switch (state) {
    case DECIDE:
      timerStarted = false;
      long waterTempReadingAge;

      waterTempReadingAge = millis() - waterTempRecorededAt;
      if (waterTempReadingAge > waterTempReadingLifetime || waterTempRecorededAt == -1) //Check weather current water temperature value is stale
        state = PROBE;
      else if (waterTemp < targetTemp - tempTolerance.value)
        state = HEAT_AUTO;
      else
        state = STANDBY;
      break;

    case STANDBY:
      //wait for a few minutes and probe again
      if (!timerStarted) {
        timerStarted = true;
        timerStartedAt = millis();
      }

      pumpOn = false;

      timeEllapsed = millis() - timerStartedAt;
      //TODO UNCOMMENT ON PRODUCTION
      //if (timeEllapsed > minToMillis(standbyTime.value)) {
      if (timeEllapsed > secToMillis(standbyTime.value)) {
        timerStarted = false;
        state = PROBE;
      }
      break;

    case PROBE:
      //run water for a few seconds and meassure temperature
      if (!timerStarted) {
        timerStarted = true;
        timerStartedAt = millis();
      }

      pumpOn = true;

      timeEllapsed = millis() - timerStartedAt;
      if (timeEllapsed > secToMillis(probeTime.value)) {
        timerStarted = false;
        updateWaterTemp();
        state = DECIDE;
      }
      break;

    case HEAT_AUTO:
      //run water and choose how to heat it until target temp. is reached
      updateWaterTemp();

      if (solarEnabled) {
        if (waterTemp < solarTemp - solarTempOffset.value - solarTempTolerance.value)
          solarOn = true;
        if (waterTemp > solarTemp - solarTempOffset.value)
          solarOn = false;
      }
      else {
        solarOn = false;
      }

      //Cosas de la caldera... idk

      pumpOn = solarOn || heaterOn;

      if (waterTemp > targetTemp + tempTolerance.value) {
        state = STANDBY;
      }
      break;

    case HEAT_FORCE:
      //run water and choose how to heat it
      updateWaterTemp();
      pumpOn = true;
      solarOn = solarEnabled;
      break;

    default:
    case NONE:
      //turn everything off
      solarOn = false;
      pumpOn = false;
      timerStarted = false;
      break;
  }


  //  Serial.print(" Solar On: ");
  //  Serial.print(solarOn);
  //  Serial.print(" Pump On: ");
  //  Serial.println(pumpOn);


  //Listen for changes to trigger display updates and update change listeners
  for (ChangeListener_base *cl : changeListenList) {
    if (cl->changed()) {
      cl->actionOnChange();
    }
    cl->updateValue();
  }

  delay(50);
}


// Temperature Sensor Updates

void updateWaterTemp() {
  int val = analogRead(1);
  waterTemp = map(val, 0, 1023, 0, 50);
  waterTempRecorededAt = millis();
  //  nWaterTemp.setValue(waterTemp);
  //  nInfoWaterTemp.setValue(waterTemp);
  //When we upgrade waterTemp to float remember to always truncate it to one decimal place so sensor noise wont trigger display updates every frame
}


long test_lastTempTakenAt = -1;
void updateSolarTemp() {
  int val = analogRead(0);
  solarTemp = map(val, 0, 1023, 0, 50);

  long ellapsedTime = millis() - test_lastTempTakenAt;
  if (ellapsedTime > 5000 || test_lastTempTakenAt == -1) {
    TimedReading reading = {solarTemp, millis()};
    registerWaterTemperature(reading);
    test_lastTempTakenAt = millis();
  }
  //updateTempWaveform();
  //  nInfoSolarTemp.setValue(solarTemp);
  //When we upgrade solarTemp to float remember to always truncate it to one decimal place so sensor noise wont trigger display updates every frame
}


// Display page updates

void updatePage0(void *ptr) {
  nWaterTemp.setValue(waterTemp);
  nTargetTemp.setValue(targetTemp);
  sTargetTemp.setValue(targetTemp);
  rOff.setValue(operationMode == OFF);
  rAuto.setValue(operationMode == AUTO);
  rOn.setValue(operationMode == ON);
  cSolar.setValue(solarEnabled);
  cHeater.setValue(heaterEnabled);
}

void updatePage1(void *ptr) {
  nTempTol.setValue(tempTolerance.value);
  nSolarTol.setValue(solarTempTolerance.value);
  nSolarOffset.setValue(solarTempOffset.value);
  nProbeTime.setValue(probeTime.value);
}

void updatePage3(void *ptr) {
  nStandbyTime.setValue(standbyTime.value);
}

void updatePage4(void *ptr) {
  nInfoWaterTemp.setValue(waterTemp);
  nInfoSolarTemp.setValue(solarTemp);

  displayUpdatePumpOn();
  displayUpdateSolarOn();
  displayUpdateHeaterOn();
  displayUpdateState();
  //waveform stuff
}


void displayUpdateWaterTemp() {
  nWaterTemp.setValue(waterTemp);
  nInfoWaterTemp.setValue(waterTemp);
}

void displayUpdateSolarTemp() {
  nInfoSolarTemp.setValue(solarTemp);
//  Serial.println("sup");
//  Serial.println(solarTemp);
}

void displayUpdateState() {
  String stateString;
  switch (state) {
    case DECIDE: stateString = "Decide"; break;
    case STANDBY: stateString = "Standby"; break;
    case PROBE: stateString = "Probe"; break;
    case HEAT_AUTO: stateString = "Heat A."; break;
    case HEAT_FORCE: stateString = "Heat F."; break;
    case NONE: stateString = "None"; break;
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


// Radio button callbacks

void rOffPushCallback(void *ptr) {
  setOperationMode(OFF);
}

void rAutoPushCallback(void *ptr) {
  setOperationMode(AUTO);
}

void rOnPushCallback(void *ptr) {
  setOperationMode(ON);
}



// Checkbox callbacks
void cSolarPushCallback(void *ptr) {
  uint32_t enabled;
  cSolar.getValue(&enabled);
  setSolarEnabled(enabled);
}

void cCalderaPushCallback(void *ptr) {
  uint32_t enabled;
  cHeater.getValue(&enabled);
  setHeaterEnabled(enabled);
}



// Temperature slider callbacks

/*When slider is pressed moved or released update target temperature to slider value*/
void sTempCallback(void *ptr) {
  updateTargetTempFromDisplay();
  //  if (operationMode == AUTO && state == STANDBY) {
  //    state = DECIDE;
  //  }
}


/*Update target temperature to slider value */
void updateTargetTempFromDisplay() {
  uint32_t temp = minTemp - 1;
  //  for (int i = 0; i <= 4; i++) { //Sometimes nextion library fails to fetch slider value. Try up to five times until success
  //    if (sTemp.getValue(&temp)) {
  //      setTargetTemp(temp, false);
  //      break;
  //    }
  //    delay(50);
  //  }
  sTargetTemp.getValue(&temp);
  setTargetTemp(temp, false);
}


// Settings callbacks

void numericSettingMinusCallback(void *ptr) {
  NumericSetting *numericSetting = (NumericSetting *) ptr;
  numericSetting->value--;
  numericSetting->nexNumber.setValue(numericSetting->value);
}

void numericSettingPlusCallback(void *ptr) {
  NumericSetting *numericSetting = (NumericSetting *) ptr;
  numericSetting->value++;
  numericSetting->nexNumber.setValue(numericSetting->value);
}


// Setters

/*Set target temperature and update display accordingly, only update slider if requierd to avoid display glitching*/
void setTargetTemp(int temp, bool updateSlider) {
  targetTemp = constrain(temp, minTemp, maxTemp);
  nTargetTemp.setValue(targetTemp);
  if (updateSlider) {
    sTargetTemp.setValue(targetTemp);
  }

  if (operationMode == AUTO && state == STANDBY) {
    state = DECIDE;
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
  switch (mode) {
    case OFF:
      state = NONE;
      break;

    case AUTO:
      if (operationMode != mode)
        state = DECIDE;
      break;

    case ON:
      state = HEAT_FORCE;
      break;
  }
  operationMode = mode;
  rOff.setValue(mode == OFF ? 1 : 0);
  rAuto.setValue(mode == AUTO ? 1 : 0);
  rOn.setValue(mode == ON ? 1 : 0);
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

float fmap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// Temperature Waveform stuff

//typedef struct timedReading {
//  int val;
//  long takenAt;
//} TimedReading;


// We will take water temperature readings at most every 15 minutes and want to store 12hrs of data so we will have at most 4 * 12 = 48 readings
const int maxReadings = 150;
int readingsTaken = 0;
TimedReading waterTempReadings[maxReadings];

void registerWaterTemperature(TimedReading reading) {
  if (readingsTaken < maxReadings) {
    //Store new reading on next available space on array
    waterTempReadings[readingsTaken] = reading;
    readingsTaken++;
  }
  else {
    //Shift array to the left and store new reading on the last spot
    for (int i = 0; i < (maxReadings - 1); i++) {
      waterTempReadings[i] = waterTempReadings[i + 1];
    }
    waterTempReadings[maxReadings - 1] = reading;
  }
  Serial.println(reading.val);
  displayWaterTempWaveform();
}

const unsigned long WAVEFORM_TIME_INTERVAL = 10 * 1000UL; // 10 seconds
const int WAVEFORM_COL_WIDTH = 48; //px
const int WAVEFORM_COL_NUM = 6;

const int WAVEFORM_HEIGHT = 136; //px
const int WAVEFORM_MIN_TEMP = 20;
const int WAVEFORM_MAX_TEMP = 40;


void displayWaterTempWaveform() {
  if (readingsTaken == 0) return;

  //int upperLimit = ((int)(waterTempReadings[maxReadings].takenAt / waveformTimeInterval))*waveformTimeInterval + waveformTimeInterval;

  unsigned long graphStartTime;
  unsigned long startTimeRoundedDown = floor(waterTempReadings[0].takenAt / (float)WAVEFORM_TIME_INTERVAL) * WAVEFORM_TIME_INTERVAL;

  if (waterTempReadings[readingsTaken - 1].takenAt - startTimeRoundedDown > WAVEFORM_TIME_INTERVAL * WAVEFORM_COL_NUM) {
    unsigned long graphEndTime = ceil(waterTempReadings[readingsTaken - 1].takenAt / (float)WAVEFORM_TIME_INTERVAL) * WAVEFORM_TIME_INTERVAL; //round up to the nearest graph time interval
    graphStartTime = graphEndTime - WAVEFORM_COL_NUM * WAVEFORM_TIME_INTERVAL;
  }
  else {
    graphStartTime = floor(waterTempReadings[0].takenAt / (float)WAVEFORM_TIME_INTERVAL) * WAVEFORM_TIME_INTERVAL; //round down to the nearest graph time interval
  }
  //  unsigned long graphStartTime = ceil(waterTempReadings[0].takenAt / (float)waveformTimeInterval) * waveformTimeInterval; //round up to the nearest graph time interval

  //  if (graphStartTime > waterTempReadings[readingsTaken - 1].takenAt) {
  //    return; //uuuhh this is an ugly solution, should fix it
  //  }

  //int numDataPoints = map(waterTempReadings[readingsTaken-1].takenAt - graphStartTime, 0, waveformTimeInterval * waveformColNum, 0, waveformColWidth * waveformColNum);
  long numDataPoints = ((waterTempReadings[readingsTaken - 1].takenAt - graphStartTime) / (float) WAVEFORM_TIME_INTERVAL) * WAVEFORM_COL_WIDTH;

  if (numDataPoints == 0) {
    Serial.println("No Data Points");
    return;
  }


  //  Serial.print("Readings taken: ");
  //  Serial.print(readingsTaken);
  //  Serial.print("  Num Data Points: ");
  //  Serial.println(numDataPoints);


  //Interpolate over current teperature readings to produce regular-intervaled data points
  int dataPoints[numDataPoints];
  for (int i = 0; i < numDataPoints; i++) {
    unsigned long pointTime = graphStartTime + ((i / (float)WAVEFORM_COL_WIDTH) * WAVEFORM_TIME_INTERVAL);

    //    Serial.print("graphStartTime: ");
    //    Serial.print(graphStartTime);
    //    Serial.print("  first r: ");
    //    Serial.println(waterTempReadings[readingsTaken - 1].takenAt);

    bool debug_found = false;
    if (waterTempReadings[0].takenAt >= pointTime) {
      //data point is from before any measurments were made, send 0
      dataPoints[i] = 1;
      debug_found = true;
    }
    else {
      //data points lies between measuements, serach for closest ones
      for (int j = 0; j < readingsTaken - 1; j++) {
        if (waterTempReadings[j + 1].takenAt >= pointTime) {
          //Interpolate dataPoint value at pointTime using closest temp readings
          float interp = (pointTime - waterTempReadings[j].takenAt) / (float) (waterTempReadings[j + 1].takenAt - waterTempReadings[j].takenAt);
          float val = ((waterTempReadings[j + 1].val - waterTempReadings[j].val) * interp) + waterTempReadings[j].val;

          //Scale and clip value to display in graph
          //val = min(max(val, WAVEFORM_MIN_TEMP), WAVEFORM_MAX_TEMP);
          val = fmap(val, WAVEFORM_MIN_TEMP, WAVEFORM_MAX_TEMP, 0, WAVEFORM_HEIGHT);
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

  //Serial.println("Khe");
  //dataPoints[0] = 32;
  //char hex[10];
  //sprintf(hex, "\\x%X", dataPoints[0]);
  //Serial.println(hex);

  //Send dataPoints to nextion display
//  Serial.print("Points: ");
  char dataString[2 * numDataPoints + 1];
  dataString[0] = 0;
  for (int i = 0; i < numDataPoints; i++) {
    //char hex[4];
    //sprintf(hex, "%X", dataPoints[i]);
    //strcat(dataString, hex);
    strcat(dataString, (char*) &dataPoints[i]);
    //Serial.print(dataPoints[i]);
  }
  //Serial.println("");

  int digits = numDataPoints >= 100 ? 3 : (numDataPoints >= 10 ? 2 : 1);
  char instructionString[21 + digits];
  sprintf(instructionString, "addt 2,0,%d\xFF\xFF\xFF", numDataPoints);

  //Serial.print(instructionString);
//  Serial.print("Data: ");
//  Serial.println(dataString);
  //  Serial.print("Mock: ");
  //  Serial.println("\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44");

  Serial2.write("cle 2,0\xFF\xFF\xFF");
  delay(50);
  Serial2.write(instructionString);
  delay(5);
  Serial2.write(dataString);

  //  Serial2.write("addt 2,0,48\xFF\xFF\xFF");
  //  delay(5);
  //  Serial2.write("\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44\x44");

}

//const long updateInterval = 10000 / 48; //one 48th of 2 seconds, (this is too fast to actually work but we'll see (its about 41ms and each loop takes at least 50ms))
//long lastWaveformUpdateAt = -1;
//
//void updateTempWaveform() {
//  long timeEllapsed = millis() - lastWaveformUpdateAt;
//  if (timeEllapsed > updateInterval || lastWaveformUpdateAt == -1) {
//    int data = max(min(solarTemp, 40), 20);
//    int height = map(data, 20, 40, 0, 136);
//    wInfoTemp.addValue(1, height);
//    //    Serial2.write("addt 2,0,48\xFF\xFF\xFF");
//    //    delay(5);
//    //    Serial2.write("\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50\x50");
//    lastWaveformUpdateAt = millis();
//  }
//}
