/***
 * Project: DryBox Control
 * File   : DryBoxControl.ino
 * Author : Werner Riemann 
 * Created: 18.10.2023
 * Board: Arduino Nano
 * 
 * Description: Controller for an active (heated) Filament DryBox.
 *              The Dryer contains a PTC Heater and a Fan for fresh dry air.
 *
 * Recommended drying temperatures:
 *    PLA, PLA+                : 40-45 degree
 *    ABS, ABS+,  PETG         : 45-50 degree
 *    PVA,PVB, ASA, TPU, PMMA  : 50-55 degree
 *
 *   Note: with one Heater, the box may barely reaches 55 degree. As max i reached 57 degree.
 *         I recommend not to go over 52-53 degree. If you need higher temperatures,
 *         install a second heater. The MosFet switch will manage this. But you
 *         also need a stronger power supply (12V, 8-10A).
 * 
 * Pins:
 * 
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 * 

 * D2     - not used
 * D1     - not used
 * D3     - not used
 * D4     - not used
 *
 * D5     - Rotary Encoder Switch
 * D6     - Rotary Encoder DT_pin                 Note: some encoder are different and you may have to change DT and CLK Pin.
 * D7     - Rotary Encoder CLK_pin                      Clockwise spin should increase all values
 * D8     - Digital in DHT11 sensor
 * D9     - Analog out PWM Fan Air exchange
 * D10    - Analog out PWM Fan Heating
 * D11    - Analog out PWM Heating
 * D12    - 
 * D13    - Digital out LEDPIN 13 (interne LED)
 * 
 *
 * --- Menu and functions ---
 * Main select: Temperature, Time, Start, Save, Version, Test functions (press Rotary to choose)
 * Temperature: change value by rotate, press to Rotary to exit
 * Time       : choose hour, minutes or Ret to exit. Press Rotary to enter value to change. Press Rotary again to exit value change.
 * Start      : press Rotary to start drying. Press Rotary during drying to pause or exit drying
 * Save       : press Rotary to save current setting for temperature and time to eeprom
 * Version    : shows the current version, press Rotary again to return to main
 * Test       : provides 3 functions for Heater, Heater Fan, Air Fan for Fresh air supply and exit Test.
 *              On exit Test menu, all values are set to zero and stops all modules.
 *
 *              This is useful to check all modules separately after assembling modules. Keep in mind to don't power
 *              up the heater without the fan for a longer time.
 */

#include <SPI.h>
#include <Wire.h>
#include <EEPROM.h>
#include <Arduino.h>
#include <DHT.h>
#include "DryBoxControl.h"
#include "WRKeyStateDef.h"
#include "DryBoxDisplay.h"

// defines for DHT11 Temp and Humitidy Sensor
#define DHTPIN 8        // Digital pin connected to the DHT sensor
#define DHTTYPE DHT11   // chose the DHT type you are using
//#define DHTTYPE DHT22 

volatile uint8_t B100HzToggle = 0;  // 100 Hertz Signal
uint8_t ui10MilliSekCount = 0;

int encoder_value = 500;
int last_encoder_value  = 500;
int EncSwitch = 5;
int DT_pin = 6;
int CLK_pin = 7;
int DT_pinstate;
int last_DT_pinstate;

int AirFanSpeed = 100;    // Airexchange Fan Speed in Percent
int HeatFanSpeed = 50;    // HeatFan Speed in Percent
int HeatingPower = 25;    // Heating Power in Percent

int testAirFanSpeed = 0;    // Airexchange Fan Speed in Percent
int testHeatFanSpeed =0;    // HeatFan Speed in Percent
int testHeatingPower =0;    // Heating Power in Percent

// default values at startup, if nothing is stored in the eeprom
int DryTemperature = 35;    // Destination Dry Temperature in degree celsius
int DryTime_Hours = 1;      
int DryTime_Minutes = 30;
int curDryTime_Hours = 1;
int curDryTime_Minutes = 30;

//-- State var for rotary encoder weitch  ----
uint8_t encoderBUTTON_State=0;

//-- Remember State on for Heater, Heater Fan and Ventilation
boolean StateHeaterOn = false;
boolean StateHeaterFanOn = false;
boolean StateVentilationOn = false;

DryBoxDisplay display;
DHT dht(DHTPIN, DHTTYPE);


/***
 * Saves the current settings for temperature, drying hours and minutes
 */
void SaveSettings() {
  EEPROM.write(0, (uint8_t) 17);
  EEPROM.write(1, (uint8_t) DryTemperature);
  EEPROM.write(2, (uint8_t) DryTime_Hours);
  EEPROM.write(3, (uint8_t) DryTime_Minutes);
}


/***
 * Reads the last saved settings for temperature, drying hours and minutes
 */
void ReadSettings() {
  uint8_t isInit = EEPROM.read(0);

  if(isInit == 17) {
    DryTemperature = EEPROM.read(1);
    DryTime_Hours = EEPROM.read(2);
    DryTime_Minutes = EEPROM.read(3);
  }
}


// Interrupt is called once a millisecond, 
SIGNAL(TIMER0_COMPA_vect) 
{
  unsigned long currentMillis = millis();
  ui10MilliSekCount ++;

  if(ui10MilliSekCount >= 10 ) {
    ui10MilliSekCount = 0;
    B100HzToggle ^= 1;
  }

  ReadEncoder();  
}

void setup() {
  
  // setup Rotary encoder
  pinMode (DT_pin, INPUT);
  pinMode (CLK_pin, INPUT);
  digitalWrite(DT_pin, HIGH);
  digitalWrite(CLK_pin, HIGH);

  // setup Heating and air 
  pinMode(FANAIR_PIN, OUTPUT);  
  analogWrite(FANAIR_PIN, 0);            // PWM Fan Airexchange off
  pinMode(FANHEATING_PIN, OUTPUT);  
  analogWrite(FANHEATING_PIN, 0);            // PWM Fan Heating off
  pinMode(HEATING_PIN, OUTPUT);
  analogWrite(HEATING_PIN, 0);            // PWM Heating off  

  ReadSettings();

  // Reads the initial state of DT
  last_DT_pinstate = digitalRead(DT_pin);

  display.Setup();
  display.SetVersion(APP_VERSION);
  display.ScreenOut(SCR_WELCOME);
  dht.begin();                      // start DHT

  // Timer setup --------------------------
  // Timer0 is already used for millis() - we'll just interrupt somewhere
  // in the middle and call the "Compare A" function below
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);    

}

void ReadEncoder() {
    DT_pinstate = digitalRead(DT_pin);
    if (DT_pinstate != last_DT_pinstate) { //did DT changed state?
      if (digitalRead(CLK_pin) == DT_pinstate) { // if DT changed state, check CLK
        encoder_value--; // rotation is counter-clockwise, decrement the value
      }else{
        encoder_value++; // rotation is clockwise, increment the value
      }
    last_DT_pinstate = DT_pinstate; //save the last state of DT
    }
}


/***
 * EncoderValueChange - increments or decrements the value of given var depending
 * from the direction the rotary encoder is moved. The Value of var is kept in range
 * of rangeMin and rangeMax.
 * 
 * param int * valToModify : pointer of var to modify
 * param int rangeMin: minimum value 
 * param int rangeMax: maximum value
*/
void EncoderValueChange(int * valToModify, int rangeMin, int rangeMax) {
    int aktValue = * valToModify;

    if(last_encoder_value != encoder_value)
    {

      if(encoder_value > last_encoder_value +1)
      {
        last_encoder_value = encoder_value;
        if(aktValue < rangeMax)
          aktValue ++;
      }
      
      if(encoder_value +1  < last_encoder_value )
      {
        last_encoder_value = encoder_value;
        if(aktValue > rangeMin)
        aktValue --;
      }
    }

    * valToModify = aktValue;
}


boolean IsPWMStateOn(int pwmValue) {
  if(pwmValue > 0)
    return true;
  else  
    return false;
}

void SetPWMRate(uint8_t pin, int ratePercent)
{
  int PWMVal = 255 * ratePercent / 100;
  analogWrite(pin,PWMVal);

  // check which Pin (heater, heater fan or ventilation) and store the state on or off
  switch (pin) {
  case FANAIR_PIN: StateVentilationOn = IsPWMStateOn(PWMVal);
  break;
  case FANHEATING_PIN: StateHeaterFanOn = IsPWMStateOn(PWMVal);
  break;
  case HEATING_PIN: StateHeaterOn = IsPWMStateOn(PWMVal);
  break;
  }
}


// Main loop -----------------------------------------------------------
void loop() {
  static uint8_t AppState=0;
  static uint8_t StateTrigger = 0;
  uint8_t aktStateTrigger;
  static int aktModeNo = 1;
  static int aktTestModeNo = 1;
  static int aktTimeEdMode = 1;  // 1= edit hour, 2= minutes, 3= exit edit
  static int aktBreakModNo = 1;

  static uint8_t ui100HzSecCounter=0;   // counter for a second
  static uint8_t ui100HzSensorTimer=200;  // read DHT11 sensor every 2 seconds
  static int runMinuteTimer = 6000;
  static int airExChgEndCounter = 2000;
  static float humidity = 0.0;
  static float temperature = 0.0;
  
  int oldHeatFanSpeed=0;
  int oldAirFanSpeed=0;
  int oldHeatingPower=0;
  int oldModeNo=0;
  int oldTestModeNo=0;
  int oldTimeEdMode=0;  
  int oldDryTemperature=0;
  int oldTimeVal=0;
  int oldBreakModNo = 1;
  
  char szBuf[8];

  aktStateTrigger = B100HzToggle;

  // Processing State-Machine. ----------------------------------------------------------
  // The state machine is triggert on every Change of flank of the timer toogle.
  // This happens 100 times in a second.

  if(aktStateTrigger != StateTrigger) {
    StateTrigger = aktStateTrigger;

    CheckKeyState(&encoderBUTTON_State, EncSwitch);

    // The DHT11 is a little bit slow, but cheap and fine for this project. So we shouldn't
    // receive values in to fast intervals
    if(ui100HzSensorTimer > 0) {
      ui100HzSensorTimer--;
    } else {
      ui100HzSensorTimer = 200;
      humidity = dht.readHumidity();
      temperature = dht.readTemperature();
      if(AppState == AST_MODE_SELECT || AppState == AST_RUNDRYING || AppState == AST_TESTMODE) {
        display.PrintTHValue(temperature, humidity);
      }
    }

    // App-State-Machine processing and step through
    switch(AppState) {

      case AST_IDLE:
          
        if(encoderBUTTON_State == 1 )
        {
        AppState = AST_PREPARE_SELECT;
        }
        break;

      case AST_PREPARE_SELECT:
        display.ScreenOut(SCR_MENUBASE);
        display.updateModSelect(aktModeNo);
        if(aktModeNo == 1)
          display.PrintDestTemp(DryTemperature, 6);
        if(aktModeNo == 2)
          display.PrintDestTime(DryTime_Hours, DryTime_Minutes, 6);
        AppState = AST_MODE_SELECT;      
        break;

      case AST_MODE_SELECT:
        oldModeNo = aktModeNo;    
        EncoderValueChange(&aktModeNo, 1, 6);     
        if(oldModeNo != aktModeNo)  // there is a change
        {
          display.updateModSelect(aktModeNo);
          if(aktModeNo == 1)
            display.PrintDestTemp(DryTemperature, 6);
          if(aktModeNo == 2)
            display.PrintDestTime(DryTime_Hours, DryTime_Minutes, 6);
        }         

        if(encoderBUTTON_State == 1 && aktModeNo == SELMOD_DRYTEMP)  // encoder switch pressed, set temperature
        {
          display.ScreenOut(SCR_SETTEMP);
          display.PrintDestTemp(DryTemperature, 0);
          display.CursorPos(1, 1);
          display.CursorOn();
          AppState = AST_SET_DRYTEMP;
        }

        if(encoderBUTTON_State == 1 && aktModeNo == SELMOD_DRYTIME)  // encoder switch pressed, set time
        {
          display.ScreenOut(SCR_SETTIME);
          display.PrintDestTime(DryTime_Hours, DryTime_Minutes, 0);
          display.CursorPos(0, 1);
          display.BlinkOn();          
          aktTimeEdMode = 1;
          AppState = AST_SET_DRYTIME;
        }

        if(encoderBUTTON_State == 1 && aktModeNo == SELMOD_DRYSTART)  // encoder switch pressed, start drying
        {
          display.ScreenOut(SCR_RUNNING);
          display.PrintHFVState(StateHeaterOn, StateHeaterFanOn, StateVentilationOn);
          display.PrintDestTemp(DryTemperature, 0);
          display.PrintDestTime(DryTime_Hours, DryTime_Minutes, 5);
          curDryTime_Hours = DryTime_Hours;
          curDryTime_Minutes = DryTime_Minutes;
          runMinuteTimer = 6000;
          dryController(DST_STARTUP, temperature);
          AppState = AST_RUNDRYING;
        }

        // save actual settings
        if(encoderBUTTON_State == 1 && aktModeNo == SELMOD_SAVE)
        {
          display.ScreenOut(SCR_SAVED);
          SaveSettings();
          AppState = AST_IDLE;
        }

        if(encoderBUTTON_State == 1 && aktModeNo == SELMOD_VERSION)
        {
          display.ScreenOut(SCR_WELCOME);
          AppState = AST_IDLE;
        }

        if(encoderBUTTON_State == 1 && aktModeNo == SELMOD_TESTING)  // encoder switch pressed, start testing mode
        {
          display.ScreenOut(SCR_TESTING);
          aktTestModeNo = 1;
          display.updateTestModSelect(aktTestModeNo);
          testAirFanSpeed = 0;    
          testHeatFanSpeed =0;    
          testHeatingPower =0;  
          AppState = AST_TESTMODE;
        }
        break;

      case AST_SET_DRYTEMP:
        oldDryTemperature = DryTemperature;
        // with one Heater, 57° is maximum. For more temperature, a second Heater 
        // and a bigger power supply is necessary.
        EncoderValueChange(&DryTemperature, 1, 55);   
        if(oldDryTemperature != DryTemperature)
        {
          display.PrintDestTemp(DryTemperature, 0);    
          display.CursorPos(1, 1);      
        }

        if(encoderBUTTON_State == 1)
        {
          display.CursorOff();
          AppState = AST_PREPARE_SELECT;
        }
        break;

      case AST_SET_DRYTIME: // select hour, minutes or Return
        oldTimeEdMode = aktTimeEdMode;
        EncoderValueChange(&aktTimeEdMode, 1, 3);
        if(oldTimeEdMode != aktTimeEdMode)
        {
          display.SetEdTimeCursorPos(aktTimeEdMode);
          display.BlinkOn();        
        }

        if(encoderBUTTON_State == 1 && aktTimeEdMode == 1) // change hours
        {
          display.BlinkOff();
          display.CursorOn();
          AppState = AST_ED_DRYHOUR;
        } 

        if(encoderBUTTON_State == 1 && aktTimeEdMode == 2) // change minutes
        {
          display.BlinkOff();
          display.CursorOn();
          AppState = AST_ED_DRYMINUTE;
        }                       

        if(encoderBUTTON_State == 1 && aktTimeEdMode == 3)
        {
          display.BlinkOff();
          AppState = AST_PREPARE_SELECT;
        }
        break;

      case AST_ED_DRYHOUR:
        oldTimeVal = DryTime_Hours;
        EncoderValueChange(&DryTime_Hours, 0, 9);
        if(oldTimeVal != DryTime_Hours)
        {
          display.PrintDestTime(DryTime_Hours, DryTime_Minutes, 0);
          display.SetEdTimeCursorPos(aktTimeEdMode);         
        }

        if(encoderBUTTON_State == 1) // return to Edit select
        {
          display.SetEdTimeCursorPos(aktTimeEdMode);
          display.CursorOff();
          display.BlinkOn();
          AppState = AST_SET_DRYTIME;
        }
        break;

      case AST_ED_DRYMINUTE:
        oldTimeVal = DryTime_Minutes;
        EncoderValueChange(&DryTime_Minutes, 0, 59);
        if(oldTimeVal != DryTime_Minutes)
        {
          display.PrintDestTime(DryTime_Hours, DryTime_Minutes, 0);
          display.SetEdTimeCursorPos(aktTimeEdMode);         
        }

        if(encoderBUTTON_State == 1) // return to Edit select
        {
          display.SetEdTimeCursorPos(aktTimeEdMode);
          display.CursorOff();
          display.BlinkOn();
          AppState = AST_SET_DRYTIME;
        }      
        break;        

      case AST_RUNDRYING:
        // Timer check and display -------------------------------
        // The active state is called 100 times per second. So 6000 calls are one minute
        if(runMinuteTimer > 0) {
          runMinuteTimer--;
        }
        else {
          runMinuteTimer = 6000;
          if(curDryTime_Minutes > 0) {
            curDryTime_Minutes--;
            display.PrintHFVState(StateHeaterOn, StateHeaterFanOn, StateVentilationOn);
            display.PrintDestTime(curDryTime_Hours, curDryTime_Minutes, 5);
               
          } else {
            if(curDryTime_Hours > 0) {
              curDryTime_Hours--;
              curDryTime_Minutes = 59;
              display.PrintDestTime(curDryTime_Hours, curDryTime_Minutes, 5);
            } else {
              // drying ready
              dryController(DST_TEARDOWN, temperature);
              display.PrintHFVState(StateHeaterOn, StateHeaterFanOn, StateVentilationOn);
              AppState = AST_ENDVENTILATION;
              //add a las fresh air ventilation after drying
              airExChgEndCounter=2000;  // fresh air for 20 seconds
              SetPWMRate(FANAIR_PIN, 80);              
              display.PrintHFVState(StateHeaterOn, StateHeaterFanOn, StateVentilationOn);
            }
          }
        }

        // Heating control ------------------------------------
        if(AppState == AST_RUNDRYING) {// only while AST_RUNDRYING is active, call dryController
          dryController(0, temperature);
          display.PrintHFVState(StateHeaterOn, StateHeaterFanOn, StateVentilationOn);
        }

        if(encoderBUTTON_State == 1)
        {
          AppState = AST_RUNPAUSE;
          dryController(DST_BREAK, temperature);
          display.ScreenOut(SCR_RUNBREAK);
          display.CursorPos(0, 1);          
          display.BlinkOn();
          aktBreakModNo=1;
        }
        break;        

      case AST_ENDVENTILATION:
        if(airExChgEndCounter > 0) {
          airExChgEndCounter--;
        } 
        else {
          SetPWMRate(FANAIR_PIN, 0);
          AppState = AST_PREPARE_SELECT;
        }
        break;

      case AST_RUNPAUSE:
        oldBreakModNo = aktBreakModNo;
        EncoderValueChange(&aktBreakModNo, 1, 2);
        if(oldBreakModNo != aktBreakModNo) {
          display.SetBreakCursorPos(aktBreakModNo);
        }

        if(encoderBUTTON_State == 1 && aktBreakModNo == 1) // continue
        {
          display.BlinkOff();
          dryController(DST_CONTINUE, temperature);
          dryController(0, temperature);
          AppState = AST_RUNDRYING;
          display.ScreenOut(SCR_RUNNING);
          display.PrintHFVState(StateHeaterOn, StateHeaterFanOn, StateVentilationOn);
          display.PrintDestTemp(DryTemperature, 0);
          display.PrintDestTime(curDryTime_Hours, curDryTime_Minutes, 5);
        }    

        if(encoderBUTTON_State == 1 && aktBreakModNo == 2) // stop
        {
          display.BlinkOff();
          dryController(DST_TEARDOWN, temperature);
          AppState = AST_PREPARE_SELECT;           
        }      
        break;
      
      case AST_TESTMODE:
        oldTestModeNo = aktTestModeNo;
        EncoderValueChange(&aktTestModeNo, 1, 4);
        if(oldTestModeNo != aktTestModeNo) {
          display.updateTestModSelect(aktTestModeNo);
        if(aktTestModeNo == 2)
          display.PrintPercentValue(testHeatFanSpeed);
        if(aktTestModeNo == 3)
          display.PrintPercentValue(testHeatingPower);
        if(aktTestModeNo == 4)
          display.PrintPercentValue(testAirFanSpeed);
        }    

        if(encoderBUTTON_State == 1 && aktTestModeNo == 1) { // exit testing
          SetPWMRate(HEATING_PIN, 0);
          SetPWMRate(FANHEATING_PIN, 0);
          SetPWMRate(FANAIR_PIN, 0);

          AppState = AST_PREPARE_SELECT;
        }

        if(encoderBUTTON_State == 1 && aktTestModeNo == 2) { // set test Fan speed
          display.PrintPercentValue(testHeatFanSpeed);
          display.CursorPos(6, 1);
          display.CursorOn();          
          AppState = AST_FAN_CONTROL;
        }

        if(encoderBUTTON_State == 1 && aktTestModeNo == 3) { // set test Heat power
          display.PrintPercentValue(testHeatingPower);
          display.CursorPos(6, 1);
          display.CursorOn();          
          AppState = AST_HEAT_CONTROL;
        }

        if(encoderBUTTON_State == 1 && aktTestModeNo == 4) { // set test Air Fan speed
          display.PrintPercentValue(testAirFanSpeed);
          display.CursorPos(6, 1);
          display.CursorOn();          
          AppState = AST_AIR_CONTROL;
        }        
        break;
      
      // Fan Speed, on exit switch off Fan
      case AST_FAN_CONTROL:
        oldHeatFanSpeed = testHeatFanSpeed;
        EncoderValueChange(&testHeatFanSpeed, 0, 99);
        if(oldHeatFanSpeed != testHeatFanSpeed) {
          display.PrintPercentValue(testHeatFanSpeed);
          SetPWMRate(FANHEATING_PIN, testHeatFanSpeed);
        }

        if(encoderBUTTON_State == 1 )
        { 
        display.CursorOff();
        display.updateTestModSelect(aktTestModeNo);
        display.PrintPercentValue(testHeatFanSpeed);
        AppState = AST_TESTMODE;
        }
        break;

      case AST_AIR_CONTROL:
        oldAirFanSpeed = testAirFanSpeed;
        EncoderValueChange(&testAirFanSpeed, 0, 99);
        if(oldAirFanSpeed != testAirFanSpeed) {
          display.PrintPercentValue(testAirFanSpeed);
          SetPWMRate(FANAIR_PIN, testAirFanSpeed);
        }

        if(encoderBUTTON_State == 1 )
        {
        display.CursorOff();
        display.updateTestModSelect(aktTestModeNo);
        display.PrintPercentValue(testAirFanSpeed);
        AppState = AST_TESTMODE;
        }
        break;

      case AST_HEAT_CONTROL:
        oldHeatingPower = testHeatingPower;
        EncoderValueChange(&testHeatingPower, 0, 99);
        if(oldHeatingPower != testHeatingPower) {
          display.PrintPercentValue(testHeatingPower);
          SetPWMRate(HEATING_PIN, testHeatingPower);
        }

        if(encoderBUTTON_State == 1 )
        {
        display.CursorOff();
        display.updateTestModSelect(aktTestModeNo);
        display.PrintPercentValue(testHeatingPower);
        AppState = AST_TESTMODE;
        }
        break;        
    }

  }

}

// --- functions for temperature control ------------------

/***
* Setup the rampup values. All power values are percentage values.
* We need different power values depending from the choosed temperature. Currently there are
* 6 ranges defined. Most importent are the ranges between 40 and 55 degrees.
*
* param uint8_t rampHeatValues[], uint8_t rampHeatFanValues[]  - adress of an array containing 4 values
* param int dryDestTemp  - destination drying temperature
* param float *compareOffset - offset comparing destination temperature
* param uint8_t *nearDestHeaterValue - heater power value near before destination temperature
* param uint8_t *ventilationHeaterValue - heater power value during ventilation
*/
void setHeatupRamp(uint8_t rampHeatValues[], uint8_t rampHeatFanValues[], int dryDestTemp,
                   float *compareOffset, uint8_t *nearDestHeaterValue, uint8_t *ventilationHeaterValue) {

  // we are using the same variables for each temperature range. But depending from the range, they are filled with matching values
  if(dryDestTemp <= 32) {
    rampHeatValues[0]=26; rampHeatFanValues[0]=32;    // rampHeatValues[0] - the lowest heater fan value is also used, when destinatiion temp is reached
    rampHeatValues[1]=40; rampHeatFanValues[1]=44;
    rampHeatValues[2]=50; rampHeatFanValues[2]=56;
    rampHeatValues[3]=72; rampHeatFanValues[3]=72;    // rampHeatValues[3] - the last entry is the final value to power the heater and heater fan
    *compareOffset = 0.5;                             // compareOffset - value below destination temperature, when heater is switched to lower power
    *nearDestHeaterValue = 44;                        // nearDestHeaterValue - power value for heater near before destination temperature
                                                      // this two values above should prevent over shoot the destination temp to much
    *ventilationHeaterValue = 40;                     // ventilationHeaterValue - power value for heater during ventilation
  }

  if(dryDestTemp > 32 && dryDestTemp <= 35) {
    rampHeatValues[0]=28; rampHeatFanValues[0]=34;
    rampHeatValues[1]=52; rampHeatFanValues[1]=52;
    rampHeatValues[2]=64; rampHeatFanValues[2]=64;
    rampHeatValues[3]=80; rampHeatFanValues[3]=76;
    *compareOffset = 0.5;
    *nearDestHeaterValue = 58;  
    *ventilationHeaterValue = 50; 
  }

  if(dryDestTemp > 35 && dryDestTemp <= 40) {
    rampHeatValues[0]=30; rampHeatFanValues[0]=34;
    rampHeatValues[1]=54; rampHeatFanValues[1]=56;
    rampHeatValues[2]=78; rampHeatFanValues[2]=68;
    rampHeatValues[3]=88; rampHeatFanValues[3]=78;
    *compareOffset = 0.2;
    *nearDestHeaterValue = 62; // 62
    *ventilationHeaterValue = 54;  // 54 
  }

  if(dryDestTemp > 40 && dryDestTemp <= 45) {   
    rampHeatValues[0]=35; rampHeatFanValues[0]=40;
    rampHeatValues[1]=56; rampHeatFanValues[1]=64;
    rampHeatValues[2]=68; rampHeatFanValues[2]=72;
    rampHeatValues[3]=88; rampHeatFanValues[3]=82;
    *compareOffset = 0.2;
    *nearDestHeaterValue = 66; // may be 68
    *ventilationHeaterValue = 56;    
  }

  if(dryDestTemp > 45 && dryDestTemp <= 50) {   // set is fine for 45-50 degrees
    rampHeatValues[0]=35; rampHeatFanValues[0]=40;
    rampHeatValues[1]=58; rampHeatFanValues[1]=64;
    rampHeatValues[2]=82; rampHeatFanValues[2]=72;
    rampHeatValues[3]=96; rampHeatFanValues[3]=84;
    *compareOffset = 0.2;
    *nearDestHeaterValue = 70;
    *ventilationHeaterValue = 60;    
  }

  if(dryDestTemp > 50 ) {   
    rampHeatValues[0]=45; rampHeatFanValues[0]=50;
    rampHeatValues[1]=65; rampHeatFanValues[1]=64;
    rampHeatValues[2]=86; rampHeatFanValues[2]=72;
    rampHeatValues[3]=98; rampHeatFanValues[3]=88;
    *compareOffset = 0.2;
    *nearDestHeaterValue = 75;
    *ventilationHeaterValue = 65;    
  }
}



/***
 * dryController(uint8_t doState, float aktTemperature)
 *
 * Controlling the drying process with heating and fresh air.
 * 
 * param uint8_t doState: force action from external caller
 * param float aktTemperature: actual temperature given from extern
 */
 void dryController(uint8_t doState, float aktTemperature)
 {
   static uint8_t DryState=0;
   static uint8_t oneSecondCounter=100; // The Controller is called 100 times every second
   static uint8_t rampSecCounter=0;
   static int     airExChgOneMinuteCounter=6000; // Controller is called 100 times every second, 6000 counts equals 1 minute
   static uint8_t airExChgMinutesCounter=0;

   // for different temperature ranges we need different power for heating and fan
   // don't change the values here. This has no effect. Change the values in function setHeatupRamp for adjusting
   static uint8_t rampUpHeatPWM[4] = {25, 40,60, 80};
   static uint8_t rampUpFanPWM[4] = {34, 40,60, 74};
   static float DestCompareOffset = 0.5;      // offset for comparing destination temp
   static uint8_t NearDestHeaterPWM = 30;     // Heater PWM value near before reaching dest temp
   static uint8_t VentilationHeaterPWM = 40;  // Heater PWM value during ventilation to avoid to much temp drop

   if(doState > 0) {
     DryState = doState;
   } else {
     if(DryState != DST_AIR_EXCHANGE) {
        if(airExChgOneMinuteCounter > 0 ) {
          airExChgOneMinuteCounter--;
        } else {
          airExChgOneMinuteCounter=6000;
          airExChgMinutesCounter++;
          
        }
     }
   }

   switch(DryState) {
     case DST_STARTUP:
        oneSecondCounter=100;
        rampSecCounter=0;
        airExChgOneMinuteCounter=6000;
        airExChgMinutesCounter=0;
        setHeatupRamp(rampUpHeatPWM, rampUpFanPWM, DryTemperature, &DestCompareOffset, &NearDestHeaterPWM, &VentilationHeaterPWM);
        DryState = DST_RAMPUP_HEATER;
        break;

     case DST_TEARDOWN:
        SetPWMRate(FANHEATING_PIN, 0);
        SetPWMRate(HEATING_PIN, 0);
        SetPWMRate(FANAIR_PIN, 0);
        break;

     case DST_RAMPUP_HEATER:
        if(oneSecondCounter > 0) {
          oneSecondCounter--;
        } else {
          oneSecondCounter=100;
          if(rampSecCounter < 3) {
            rampSecCounter++;
            SetPWMRate(FANHEATING_PIN, rampUpFanPWM[rampSecCounter]);
            SetPWMRate(HEATING_PIN, rampUpHeatPWM[rampSecCounter]);  
          } else {
            oneSecondCounter=250; // wait in next step for 2,5 seconds
            DryState = DST_WAIT_DEST_TEMP;
          }
        }
        
        break;

     case DST_WAIT_DEST_TEMP:
        if(oneSecondCounter > 0) {
          oneSecondCounter--;
        } else { 
           if(aktTemperature + DestCompareOffset >= DryTemperature) {
            SetPWMRate(HEATING_PIN, NearDestHeaterPWM);              
          } 
        }

        if(aktTemperature >= DryTemperature) {
          SetPWMRate(FANHEATING_PIN, rampUpFanPWM[0]); // heating fan continue with lower speed. Lowest defined speed for the range
          SetPWMRate(HEATING_PIN, 0);                  // heater off
          DryState = DST_TEMP_REACHED;         
        }        
        break;        

     case DST_TEMP_REACHED:
        if(aktTemperature < DryTemperature) {
          oneSecondCounter=100;
          rampSecCounter=0;
          DryState = DST_RAMPUP_HEATER;               // go back to ramp up the heater
        }

        if(airExChgMinutesCounter >= AIR_EXCHANGE_MINUTES_INTERVAL) {
          airExChgOneMinuteCounter=2000;  // fresh air for 20 seconds
          SetPWMRate(FANAIR_PIN, 80);     // speed for ventilation fan. Adjust value here, if you need more or less
          SetPWMRate(HEATING_PIN, VentilationHeaterPWM);  // additional power up the heater a bit for lower temperature drop
          DryState = DST_AIR_EXCHANGE;
        }
        break;

      case DST_AIR_EXCHANGE:
        if(airExChgOneMinuteCounter > 0) {
          airExChgOneMinuteCounter--;
        } else {
          airExChgOneMinuteCounter = 6000; //set for next count
          airExChgMinutesCounter = 0;
          SetPWMRate(HEATING_PIN, 0); // heater has to switch off. Otherwise the temp will continue to raise up 
          SetPWMRate(FANAIR_PIN, 0);
          DryState = DST_TEMP_REACHED;
        }
        break;

      case DST_BREAK:
        SetPWMRate(FANHEATING_PIN, 0);
        SetPWMRate(HEATING_PIN, 0);      
        break;

      case DST_CONTINUE:
        oneSecondCounter=100;
        rampSecCounter=0;      
        DryState = DST_RAMPUP_HEATER;
        break;        
   }

 }

