#include <stdint.h>
/***
 * Project: DryBox Control
 * File   : DryBoxDisplayDisplay.h
 * Author : Werner Riemann 
 * Created: 18.10.2023
 * Board: Arduino Nano
 * 
 * Description: Modul for Display outputs
 * 
 * Pins:
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 * 
 * 
 */

#include <Arduino.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

// Display constants for ScreenOut
#define SCR_CLEAR       0       // Display clear
#define SCR_WELCOME     1       // Welcome Message
#define SCR_SAVED       2       // Message settings saved
#define SCR_MENUBASE    10      // Basenumber Menu
#define SCR_SETTEMP     20      // Set temperature
#define SCR_SETTIME     30      // Set dry time
#define SCR_ERROR       90

#define SCR_RUNNING     40      // Show data for temperature and humidity
#define SCR_RUNBREAK    41      // Run Drying paused or select stop 
#define SCR_TESTING     80      // Show data for temperature and humidity

class DryBoxDisplay
{
private:
    /* data */


public:
    DryBoxDisplay(/* args */);
    void Setup();
    void SetVersion(const char  *strVersion);
    void SetBacklight(uint8_t mode);
    void BlinkOn();
    void BlinkOff();
    void CursorOn();
    void CursorOff();    
    void CursorPos(int x, int y);
    void SetEdTimeCursorPos(int edTimeModeNo);
    void SetBreakCursorPos(int selectModeNo);
    void updateModSelect(uint8_t modeNo);
    void updateTestModSelect(uint8_t testModeNo);
    void PrintActiveMode(uint8_t modeNo);
    void PrintTHValue(float temp, float humidity);
    void PrintPercentValue(int pValue);
    void PrintDestTemp(int dvalue, uint8_t startPos);
    void PrintDestTime(int hour, int minute, uint8_t startPos);
    void PrintHFVState(boolean heater, boolean heaterfan, boolean ventilation, boolean turbomode);
    void ScreenOut(uint8_t uiScreenID);
    void PrintError(const char* errorMsg);

};
