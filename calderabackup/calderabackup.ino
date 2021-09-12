#include "Nextion.h"

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
  HEAT
};

NexSlider sTemp = NexSlider(0, 1, "sTargetTemp");
NexNumber nTargetTemp = NexNumber(0, 4, "nTargetTemp");
NexNumber nWaterTemp = NexNumber(0, 6, "nWaterTemp");
NexRadio rOff = NexRadio(0, 8, "rOff");
NexRadio rAuto = NexRadio(0, 9, "rAuto");
NexRadio rOn = NexRadio(0, 10, "rOn");
NexCheckbox cSolar = NexCheckbox(0, 17, "cSolar");
NexCheckbox cCaldera = NexCheckbox(0, 18, "cCaldera");

// Register objects to the touch event list.
NexTouch *nex_listen_list[] = {
  &sTemp,
  &rOff,
  &rAuto,
  &rOn,
  &cSolar,
  &cCaldera,
  NULL
};

OperationMode operationMode;
State state = STANDBY;

uint32_t tempTolerance = 2; // For hysteresis
uint32_t solarTempTolerance = 2;// For hysteresis
uint32_t solarTempOffset = 2; // Solar wont turn on unless solar temperature is solarTempOffset degrees above water temp

uint32_t targetTemp;
uint32_t waterTemp;
uint32_t solarTemp;

uint32_t maxTemp;
uint32_t minTemp;

bool solarEnabled;
bool calderaEnabled;

uint32_t waterTempRecorededAt = -1; // initialize to -1 to ensure water temp readig is interpreted as stale on the first loop since there isnt a water temperature reading yet
uint32_t waterTempReadingLifetime = 10 * 60 * 1000UL; // 10 minutes

uint32_t probeTime = 30 * 1000UL; //30 seconds
uint32_t standbyTime = 15 * 60 * 1000UL; //15 minutes. Should be longer than waterTempReadingLifetime

bool timerStarted = false;
uint32_t timerStartedAt;


void setup() {
  Serial.begin(9600);
  nexInit();

  //attach ui component press/release callback functions
  sTemp.attachPop(sTempPopCallback);
  sTemp.attachPush(sTempPushCallback);
  rOff.attachPush(rOffPushCallback);
  rAuto.attachPush(rAutoPushCallback);
  rOn.attachPush(rOnPushCallback);
  cSolar.attachPop(cSolarPushCallback);
  cCaldera.attachPop(cCalderaPushCallback);

  setMaxTemp(45);
  setMinTemp(25);
  setTargetTemp(30, true);
  setOperationMode(OFF);
  setSolarEnabled(true);
  setCalderaEnabled(false);
}


void loop() {
  nexLoop(nex_listen_list);

  //apagar bomba

  switch (state) {
    case DECIDE:
      uint32_t waterTempReadingAge;

      waterTempReadingAge = waterTempRecorededAt - millis();
      if (waterTempReadingAge > waterTempReadingLifetime || waterTempRecorededAt == -1) //Check weather current water temperature value is stale
        state = PROBE;
      else if (waterTemp < targetTemp - tempTolerance)
        state = HEAT;
      else
        state = STANDBY;
      break;

    case STANDBY:
      //wait for x minutes and probe again
      if (!timerStarted) {
        timerStarted = true;
        timerStartedAt = millis();
      }
      uint32_t timeEllapsed = millis() - timerStartedAt;
      if (timeEllapsed > standbyTime) {
        timerStarted = false;
        state = PROBE;
      }
      break;

    case PROBE:
      //run water for y seconds and meassure temperature
      if (!timerStarted) {
        timerStarted = true;
        timerStartedAt = millis();
      }

      //prender bomba

      uint32_t timeEllapsed = millis() - timerStartedAt;
      if (timeEllapsed > probeTime) {
        timerStarted = false;
        // Update water temp
        state = DECIDE;
      }
      break;

    case HEAT:
      //run water and choose how to heat it
      if (solarEnabled) {
        if (waterTemp < solarTemp - solarTempOffset - solarTempTolerance) {
          //prender solar
          //prender bomba
        }
        if (waterTemp > solarTemp - solarTempOffset) {
          //apagar solar
        }
      }
      else {
        //apagar solar
      }

      //Cosas de la caldera... idk

      if (waterTemp > targetTemp + tempTolerance) {
        state = STANDBY;
      }
      break;

    default:
    case NONE:
      //turn everything off
      break;
  }


  delay(20);
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
  cCaldera.getValue(&enabled);
  setCalderaEnabled(enabled);
}



// Temperature slider callbacks

/*When slider is moved or released update target temperature to slider value*/
void sTempPopCallback(void *ptr) {
  updateTargetTempFromDisplay();
}

/*When slider is pressed update target temperature to slider value*/
void sTempPushCallback(void *ptr) {
  updateTargetTempFromDisplay();
}

/*Update target temperature to slider value */
void updateTargetTempFromDisplay() {
  uint32_t temp = minTemp - 1;
  for (int i = 0; i <= 4; i++) { //Sometimes nextion library fails to fetch slider value. Try up to five times until success
    if (sTemp.getValue(&temp)) {
      setTargetTemp(temp, false);
      break;
    }
    delay(50);
  }
}



//setters

/*Set target temperature and update display accordingly, only update slider if requierd to avoid display glitching*/
void setTargetTemp(uint32_t temp, bool updateSlider) {
  targetTemp = constrain(temp, minTemp, maxTemp);
  nTargetTemp.setValue(targetTemp);
  if (updateSlider) {
    sTemp.setValue(targetTemp);
  }
}

/*Set maximum allowed target temperature and update slider boundaries*/
void setMaxTemp(uint32_t temp) {
  maxTemp = temp;
  sTemp.setMaxval(temp);
}

/*Set minimum allowed target temperature and update slider boundaries*/
void setMinTemp(uint32_t temp) {
  minTemp = temp;
  sTemp.setMinval(temp);
}


/*Set operation mode and update display accordingly*/
void setOperationMode(OperationMode mode) {
  switch (mode) {
    case OFF:
      state = NONE;
      break;

    case AUTO:
      if (operationMode != mode)
        state = PROBE;
      break;

    case ON:
      state = HEAT;
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

void setCalderaEnabled(bool enabled) {
  calderaEnabled = enabled;
  cCaldera.setValue(enabled);
}
