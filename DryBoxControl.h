/***
 * Project: DryBoxControl
 * File   : DryBoxControl.h
 * Author : Werner Riemann 
 * Created: 18.10.2023
 * Board: Arduino Nano
 * 
 * Description: Controlling Filament DryBox
 * 
 * Pins:
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 * 
 * 
 */

// ensure this library description is only included once
#ifndef DryBoxControl_h
#define DryBoxControl_h

#include <Arduino.h>

#define APP_VERSION "0.49"

// Defines Pins
#define FANAIR_PIN 9
#define FANHEATING_PIN 10
#define HEATING_PIN 11

// Defines Mode selection -------------------------------------------
#define SELMOD_DRYTEMP      1
#define SELMOD_DRYTIME      2
#define SELMOD_DRYSTART     3
#define SELMOD_SAVE         4
#define SELMOD_VERSION      5
#define SELMOD_TESTING      6

// Defines Statemachine ----------------------------------------------
#define AST_IDLE            0

#define AST_PREPARE_SELECT  10
#define AST_MODE_SELECT     11

#define AST_SET_DRYTEMP     20

#define AST_SET_DRYTIME     30
#define AST_ED_DRYHOUR      31
#define AST_ED_DRYMINUTE    32
#define AST_ED_DRYRET       33

#define AST_RUNDRYING       40
#define AST_RUNPAUSE        41
#define AST_ENDVENTILATION  42

#define AST_TESTMODE        80
#define AST_FAN_CONTROL     81
#define AST_HEAT_CONTROL    82
#define AST_AIR_CONTROL     83

// States for DryController -----
#define DST_STARTUP          1
#define DST_TEARDOWN         2
#define DST_RAMPUP_HEATER    10
#define DST_WAIT_DEST_TEMP   20
#define DST_TEMP_REACHED     30
#define DST_AIR_EXCHANGE     31
#define DST_BREAK            40
#define DST_CONTINUE         41


// Prototypen in CameraSlider.ino ------------------------------------

void SaveSettings();
void ReadSettings();
void ReadEncoder();
void EncoderValueChange(int * valToModify, int rangeMin, int rangeMax);
boolean IsPWMStateOn(int pwmValue);
void SetPWMRate(uint8_t pin, int ratePercent);
void setHeatupRamp(uint8_t rampHeatValues[], uint8_t rampHeatFanValues[], int dryDestTemp,
                   float *compareOffset, uint8_t *nearDestHeaterValue, uint8_t *ventilationHeaterValue);
void dryController(uint8_t doState, float aktTemperature);

#endif
