#include "Arduino.h"
/***
 * Project: DryBox Control
 * File   : DryBoxDisplay.cpp
 * Author : Werner Riemann 
 * Created: 18.10.2023
 * Board: Arduino Nano
 * 
 * Description: Modul for Display outputs
 *
 * NOTE: this modul uses an old version of I2C LiquidCrytal lib called "NewliquidCrystal".
 *       If you want to use a newer one, you have to adjust the parameter calling the constructor.
 * 
 * Pins:
 * A4,A5  - I2C Display control (A4 - SDA, A5 - SCL)
 * 
 */

#include "DryBoxDisplay.h"

LiquidCrystal_I2C lcd(0x27,2,1,0,4,5,6,7,3,POSITIVE);

char szVersion[17] = "";

char szModNames[6][12] = {
  "Temp: 40C",
  "Time: 0:00",
  "-> Start",
  "-> Save",
  "-> Version",
  "-> Test"
};

char szTestNames[4][12] = {
  "Exit Test ",
  "HFan  0%",
  "Heat  0%",
  "AFan  0%"
};

DryBoxDisplay::DryBoxDisplay(/* args */)
{
}

void DryBoxDisplay::Setup()
{
  lcd.begin(16, 2);
  lcd.clear();  
}

void DryBoxDisplay::SetVersion(const char  *strVersion)
{
  strcpy(szVersion, "[    V");
  strcat(szVersion, strVersion);
  strcat(szVersion, "     ]");
}

void DryBoxDisplay::SetBacklight(uint8_t mode)
{
    lcd.setBacklight(mode);
}

void DryBoxDisplay::BlinkOn()
{
  lcd.blink();
}

void DryBoxDisplay::BlinkOff()
{
  lcd.noBlink();
}

void DryBoxDisplay::CursorOn()
{
  lcd.cursor();
}

void DryBoxDisplay::CursorOff()
{
  lcd.noCursor();
}

void DryBoxDisplay::CursorPos(int x, int y)
{
  lcd.setCursor(x,y);
}

void DryBoxDisplay::SetEdTimeCursorPos(int edTimeModeNo)
{
  uint8_t xpos[3] = {0,3,13};
  lcd.setCursor(xpos[edTimeModeNo-1],1);
}

void DryBoxDisplay::SetBreakCursorPos(int selectModeNo)
{
  uint8_t xpos[2] = {0,5};
  lcd.setCursor(xpos[selectModeNo-1],1);
}

/***
 * Prints the selected Modus. 
 * uint8_t modeNo: number of Modus, starting at 1
*/
void DryBoxDisplay::updateModSelect(uint8_t modeNo)
{
  
lcd.setCursor(0,1); 
lcd.print("          ");
lcd.setCursor(0,1);
lcd.print(szModNames[modeNo -1]);
 
}

void DryBoxDisplay::updateTestModSelect(uint8_t testModeNo)
{
  
lcd.setCursor(0,1); 
lcd.print("          ");
lcd.setCursor(0,1);
lcd.print(szTestNames[testModeNo -1]);
 
}

void DryBoxDisplay::PrintActiveMode(uint8_t modeNo)
{
lcd.setCursor(0,1); 
lcd.print("          ");
lcd.setCursor(0,0); 
lcd.print("-        -");
lcd.setCursor(1,0);
lcd.print(szModNames[modeNo -1]);
 
}

void DryBoxDisplay::PrintTHValue(float temp, float humidity)
{
  char szBuf[8];
  dtostrf(temp, 3, 1, szBuf);
  lcd.setCursor(11,0);
  lcd.print(szBuf);
  dtostrf(humidity, 2, 0, szBuf);
  lcd.setCursor(13,1);
  lcd.print(szBuf);
}

void DryBoxDisplay::PrintPercentValue(int pValue)
{
  char buf[6]="";
  char valBuf[6]="";
  int i,l;

  strcpy(valBuf,"  ");
  itoa(pValue, buf,10);
  l = strlen(buf);
  
  strcpy(&valBuf[2-l], buf);
  strcat(valBuf, "%");
  lcd.setCursor(5,1);
  lcd.print(valBuf);
  lcd.setCursor(6,1);
}


void DryBoxDisplay::PrintDestTemp(int dValue, uint8_t startPos)
{
  char buf[4]="";
  char valBuf[6]="";
  int i,l;

  strcpy(valBuf,"  ");
  itoa(dValue, buf,10);
  l = strlen(buf);
  
  strcpy(&valBuf[2-l], buf);
  
  lcd.setCursor(0 + startPos,1);
  lcd.print(valBuf);
}

void DryBoxDisplay::PrintDestTime(int hour, int minute, uint8_t startPos)
{
  char buf[4]="";
  char valBuf[6]="";
  int i,l;

  // convert hour
  strcpy(valBuf,"0:00");
  itoa(hour, buf,10);
  valBuf[0] = buf[0];    // copy a single digit into the outbuf

  // convert minutes
  itoa(minute, buf,10);
  l = strlen(buf);
  
  strcpy(&valBuf[2+2-l], buf);
  
  lcd.setCursor(0 + startPos,1);
  lcd.print(valBuf);
}

//lcd.print("-RUN- HFV|     C");
void DryBoxDisplay::PrintHFVState(boolean heater, boolean heaterfan, boolean ventilation, boolean turbomode)
{
  char szBuf[2] = "";
  char szOneChar[2] = " ";

  lcd.setCursor(5 , 0);
  strcpy(szBuf, szOneChar);
  if(turbomode == true) strcpy(szBuf, "T");
  lcd.print(szBuf);

  lcd.setCursor(6 , 0);
  strcpy(szBuf, szOneChar);
  if(heater == true) strcpy(szBuf, "H");
  lcd.print(szBuf);

  lcd.setCursor(7 , 0);
  strcpy(szBuf, szOneChar);
  if(heaterfan == true) strcpy(szBuf, "F");
  lcd.print(szBuf);  

  lcd.setCursor(8 , 0);
  strcpy(szBuf, szOneChar);
  if(ventilation == true) strcpy(szBuf, "V");
  lcd.print(szBuf);    

}

void DryBoxDisplay::PrintError(const char* errorMsg) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR:");
    lcd.setCursor(0, 1);
    lcd.print(errorMsg);
}

void DryBoxDisplay::ScreenOut(uint8_t uiScreenID)
{
lcd.clear();
lcd.setCursor(0,0); // start ist immer oben links

switch(uiScreenID) 
  {
  case SCR_CLEAR:
    lcd.clear();
    break;

  case SCR_WELCOME:
    lcd.print("-DryBox Control-");
    lcd.setCursor(0,1);
    lcd.print(szVersion);
    lcd.setCursor(4,1);
    break;

  case SCR_SAVED:
    lcd.print("[  Settings    ]");
    lcd.setCursor(0,1);
    lcd.print("|  saved       |");
    break;

  case SCR_MENUBASE:
    //lcd.setCursor(0,0);
    lcd.print("[ Select ]     C");
    lcd.setCursor(0,1);
    lcd.print("Temp: 40C  H   %");
    break;

  case SCR_SETTEMP:
    //lcd.setCursor(0,0);
    lcd.print("[ Set Dry Temp ]");
    lcd.setCursor(0,1);
    lcd.print("40 DegreeC | RET");
    break;

  case SCR_SETTIME:
    //lcd.setCursor(0,0);
    lcd.print("[ Set Dry Time ]");
    lcd.setCursor(0,1);
    lcd.print("1:30  h:mm | RET");  
    break;

  case SCR_RUNNING:
    //lcd.setCursor(0,0);
    //TODO: adjust screen for showing the state of Heater, Heaterfan and Ventilation
    //lcd.print("-Running-|     C");
      lcd.print("-RUN- HFV|     C");
    lcd.setCursor(0,1);
    lcd.print("35C h5:30| H   %");    
    break;

  case SCR_RUNBREAK:
    lcd.print("Run Break|");
    lcd.setCursor(0,1);
    lcd.print("cont stop|");      
    break;
  
  case SCR_TESTING:
    //lcd.setCursor(0,0);
    lcd.print("-Testing-|     C");
    lcd.setCursor(0,1);
    lcd.print("HFan  0% | H   %");    
    break;

  case SCR_ERROR:
    lcd.setCursor(0, 0);
    lcd.print("Error Screen");
    break;
  }
}
