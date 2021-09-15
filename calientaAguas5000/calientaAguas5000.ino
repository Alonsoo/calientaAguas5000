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


// This struct makes it easier to connect variables to simple numeric controls on the settings page on the display and store them
struct NumericSetting{
  NexNumber *nexNumber;
  int eeAddress;

  NumericSetting(NexNumber* nexN, int addrIndex){
     nexNumber = nexN;
     eeAddress = addrIndex * EEPROM_CHUNK_SIZE;
  }

  void setVal(int newValue){
    //value = newValue;
    nexNumber->setValue(newValue);
    EEPROM.put(eeAddress, newValue);
  }

  int getVal(){
    //return value;
    int val;
    EEPROM.get(eeAddress, val);
    return(val);
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

typedef struct {
  int val;
  unsigned long takenAt;
} TimedReading;


struct TempGrapher {
  static const unsigned long WAVEFORM_TIME_INTERVAL = 10 * 1000UL; // 10 seconds
  static const int WAVEFORM_COL_WIDTH = 48; //px
  static const int WAVEFORM_COL_NUM = 6;

  static const int WAVEFORM_HEIGHT = 136; //px
  static const int WAVEFORM_MIN_TEMP = 20;
  static const int WAVEFORM_MAX_TEMP = 40;

  static const int MIN_READING_INTERVAL = 5; //minutes

  static long latestReadingTakenAt;

  // We will take water temperature readings at most every 5 minutes and want to store 12hrs of data so we will need at most (60 / 5) * 12 = 144 readings
  static const int MAX_READINGS = (60 / MIN_READING_INTERVAL) * 12;

  int channel;
  
  int readingsTaken = 0;
  TimedReading readings[MAX_READINGS];

  TempGrapher(int chnl){
    channel = chnl;
  }

  void registerTemperature(TimedReading reading) {
    long lastReadingTakenAt = readingsTaken > 0 ? readings[readingsTaken - 1].takenAt : 0;
    long ellapsedTime = millis() - lastReadingTakenAt;

    if (ellapsedTime > 5000 || readingsTaken == 0) {
    //if (ellapsedTime > minToMillis(MIN_READING_INTERVAL) || readingsTaken == 0) {
      TempGrapher::latestReadingTakenAt = millis();
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
      displayWaveform();
    }
  }

  void displayWaveform() {
    if (readingsTaken == 0) return;

    unsigned long graphStartTime;
    unsigned long startTimeRoundedDown = floor(readings[0].takenAt / (float)WAVEFORM_TIME_INTERVAL) * WAVEFORM_TIME_INTERVAL;

    if (latestReadingTakenAt - startTimeRoundedDown > WAVEFORM_TIME_INTERVAL * WAVEFORM_COL_NUM) {
      unsigned long graphEndTime = ceil(latestReadingTakenAt / (float)WAVEFORM_TIME_INTERVAL) * WAVEFORM_TIME_INTERVAL; //round up to the nearest graph time interval
      graphStartTime = graphEndTime - WAVEFORM_COL_NUM * WAVEFORM_TIME_INTERVAL;
    }
    else {
      graphStartTime = floor(readings[0].takenAt / (float)WAVEFORM_TIME_INTERVAL) * WAVEFORM_TIME_INTERVAL; //round down to the nearest graph time interval
    }

    long numDataPoints = ((readings[readingsTaken - 1].takenAt - graphStartTime) / (float) WAVEFORM_TIME_INTERVAL) * WAVEFORM_COL_WIDTH;

    if (numDataPoints == 0) {
      Serial.println("No Data Points");
      return;
    }

    //Interpolate over current teperature readings to produce regular-intervaled data points
    int dataPoints[numDataPoints];
    for (int i = 0; i < numDataPoints; i++) {
      unsigned long pointTime = graphStartTime + ((i / (float)WAVEFORM_COL_WIDTH) * WAVEFORM_TIME_INTERVAL);

      bool debug_found = false;
      if (readings[0].takenAt >= pointTime) {
        //data point is from before any measurments were made, send 1
        dataPoints[i] = 1;
        debug_found = true;
      }
      else {
        //data points lies between measuements, serach for closest ones and interpolate
        for (int j = 0; j < readingsTaken - 1; j++) {
          if (readings[j + 1].takenAt >= pointTime) {
            //Interpolate dataPoint value at pointTime using closest temp readings
            float interp = (pointTime - readings[j].takenAt) / (float) (readings[j + 1].takenAt - readings[j].takenAt);
            float val = ((readings[j + 1].val - readings[j].val) * interp) + readings[j].val;

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

    char dataString[2 * numDataPoints + 1];
    dataString[0] = 0;
    for (int i = 0; i < numDataPoints; i++) {
      strcat(dataString, (char*) &dataPoints[i]);
    }
    
    char clearString[20];
    sprintf(clearString, "cle 2,%d\xFF\xFF\xFF", channel);
    
    int digits = numDataPoints >= 100 ? 3 : (numDataPoints >= 10 ? 2 : 1);
    char instructionString[21 + digits];
    sprintf(instructionString, "addt 2,%d,%d\xFF\xFF\xFF", channel, numDataPoints);

    //Send dataPoints to nextion display
    Serial2.write(clearString);
    delay(50);
    Serial2.write(instructionString);
    delay(5);
    Serial2.write(dataString);

  }

};
long TempGrapher::latestReadingTakenAt = -1;


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


OperationMode operationMode;
State state = STANDBY;

//int tempTolerance = 2; // For hysteresis
//int solarTempTolerance = 2;// For hysteresis
//int solarTempOffset = 2; // Solar wont turn on unless solar temperature is solarTempOffset degrees above water temp

NumericSetting tempTolerance = NumericSetting(&nTempTol, 0); //{2, nTempTol, 0}; // For hysteresis
NumericSetting solarTempTolerance = NumericSetting(&nSolarTol, 1); //{2, nSolarTol, 1}; // For hysteresis
NumericSetting solarTempOffset = NumericSetting(&nSolarOffset, 2); //{2, nSolarOffset, 2}; // Solar wont turn on unless solar temperature is solarTempOffset degrees above water temp

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

NumericSetting probeTime = NumericSetting(&nProbeTime, 3); //{5, nProbeTime, 3}; // in seconds
NumericSetting standbyTime = NumericSetting(&nStandbyTime, 4); //{10, nStandbyTime, 4};// in minutes

bool timerStarted = false;
long timerStartedAt;

NumericSetting *settings[] = {
  &tempTolerance,
  &solarTempTolerance,
  &solarTempOffset,
  &probeTime,
  &standbyTime
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


void setup() {
  Serial.begin(9600);
  
//  fetchSettings();
  
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
      else if (waterTemp < targetTemp - tempTolerance.getVal())
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
      //if (timeEllapsed > minToMillis(standbyTime.getVal())) {
      if (timeEllapsed > secToMillis(standbyTime.getVal())) {
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
      if (timeEllapsed > secToMillis(probeTime.getVal())) {
        timerStarted = false;
        updateWaterTemp();
        state = DECIDE;
      }
      break;

    case HEAT_AUTO:
      //run water and choose how to heat it until target temp. is reached
      updateWaterTemp();

      if (solarEnabled) {
        if (waterTemp < solarTemp - solarTempOffset.getVal() - solarTempTolerance.getVal())
          solarOn = true;
        if (waterTemp > solarTemp - solarTempOffset.getVal())
          solarOn = false;
      }
      else {
        solarOn = false;
      }

      //Cosas de la caldera... idk

      pumpOn = solarOn || heaterOn;

      if (waterTemp > targetTemp + tempTolerance.getVal()) {
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
 
  TimedReading reading = {waterTemp, millis()};
  waterTempGrapher.registerTemperature(reading);
  //  nWaterTemp.setValue(waterTemp);
  //  nInfoWaterTemp.setValue(waterTemp);
  //When we upgrade waterTemp to float remember to always truncate it to one decimal place so sensor noise wont trigger display updates every frame
}


void updateSolarTemp() {
  int val = analogRead(0);
  solarTemp = map(val, 0, 1023, 0, 50);
  
  TimedReading reading = {solarTemp, millis()};
  solarTempGrapher.registerTemperature(reading);
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
  nTempTol.setValue(tempTolerance.getVal());
  nSolarTol.setValue(solarTempTolerance.getVal());
  nSolarOffset.setValue(solarTempOffset.getVal());
  nProbeTime.setValue(probeTime.getVal());
}

void updatePage3(void *ptr) {
  nStandbyTime.setValue(standbyTime.getVal());
}

void updatePage4(void *ptr) {
  nInfoWaterTemp.setValue(waterTemp);
  nInfoSolarTemp.setValue(solarTemp);

  displayUpdatePumpOn();
  displayUpdateSolarOn();
  displayUpdateHeaterOn();
  displayUpdateState();
  //waveform stuff
  waterTempGrapher.displayWaveform();
  solarTempGrapher.displayWaveform();
}


void displayUpdateWaterTemp() {
  nWaterTemp.setValue(waterTemp);
  nInfoWaterTemp.setValue(waterTemp);
}

void displayUpdateSolarTemp() {
  nInfoSolarTemp.setValue(solarTemp);
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
  numericSetting->setVal(numericSetting->getVal()-1);
  //numericSetting->nexNumber->setValue(numericSetting->getVal());
}

void numericSettingPlusCallback(void *ptr) {
  NumericSetting *numericSetting = (NumericSetting *) ptr;
  numericSetting->setVal(numericSetting->getVal()+1);
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


// EEPROM Storage

//void fetchSettings(){
//  for(NumericSetting *setting: settings){
//    //EEPROM.get(setting->eeAddress, setting->value);
//    setting->
//  }
//}
