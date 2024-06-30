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

char szModNames[8][12] = {
  "Temp: 40C",
  "Time: 0:00",
  "tRPM: 300",
  "-> Start",
  "-> Save",
  "-> Version",
  "-> Test",
  "RPM"
};

char szTestNames[4][12] = {
  "Exit Test ",
  "HFan  0%",
  "Heat  0%",
  "AFan  0%"
};

int textLength; 
int lcdWidth = 16; // Adjust based on your LCD's character width
int scrollDelay = 250; // Adjust scroll speed (milliseconds)
unsigned long previousScrollMillis = 0;
int scrollPosition = 0;

void scrollText(const char* longText) 
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousScrollMillis >= scrollDelay) 
  {
    previousScrollMillis = currentMillis;

    int printedChars = 0;
    lcd.setCursor(0, 1); 

    for (int i = scrollPosition; i < scrollPosition + lcdWidth; i++) 
    {
      int index = i % textLength;
      if (longText[index] != '\0') 
      {
        lcd.print(longText[index]);
        printedChars++;
      }
      if (printedChars >= lcdWidth) 
        break;      
    }
    
    scrollPosition++;
    if (scrollPosition >= textLength) 
      scrollPosition = 0;     
  }
}

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
  int l;
  strcpy(valBuf,"  ");
  itoa(pValue, buf,10);
  l = strlen(buf);  
  strcpy(&valBuf[2-l], buf);
  strcat(valBuf, "%");
  lcd.setCursor(5,1);
  lcd.print(valBuf);
  lcd.setCursor(6,1);
}

void DryBoxDisplay::FanRPM(int rpm)
{  
  lcd.setCursor(4, 1);
  lcd.print("     ");
  lcd.setCursor(4, 1);
  lcd.print(rpm);
}

void DryBoxDisplay::PrintDestTemp(int dValue, uint8_t startPos)
{
  char buf[5] = ""; 
  lcd.setCursor(startPos, 1);
  lcd.print("     "); 
  lcd.noCursor();
  itoa(dValue, buf, 10);
  strcat(buf, "\xDF");
  strcat(buf, "C");
  lcd.setCursor(startPos, 1);
  lcd.print(buf);
}

void DryBoxDisplay::PrintDestRPM(int dValue, uint8_t startPos) 
{  
  char buf[5] = ""; 
  lcd.setCursor(startPos, 1);
  lcd.print("    "); 
  lcd.noCursor();
  if (dValue >= 0) 
  {
    itoa(dValue, buf, 10);
    lcd.setCursor(startPos, 1);
    lcd.print(buf);
  } 
  else 
  {
    lcd.setCursor(startPos, 1);
    lcd.print("OFF");
  }
}

void DryBoxDisplay::PrintDestTime(int hour, int minute, uint8_t startPos)
{
    char buf[4] = "";
    String valStr = "";
    int l;

    lcd.setCursor(startPos, 1);
    lcd.print("     "); 

    // Convert hour
    itoa(hour, buf, 10);
    valStr += buf; // Append hour

    // Add colon
    valStr += ":";

    // Convert minutes
    itoa(minute, buf, 10);
    if (minute < 10) {
        valStr += "0"; 
    }
    valStr += buf; 

    int actualStartPos = startPos;
    if (hour < 10) {
        actualStartPos -= 1;
    }

    lcd.setCursor(actualStartPos, 1);
    lcd.print(valStr);
}
void DryBoxDisplay::PrintHFVState(int temp, int humid) {
    lcd.setCursor(0, 0);
    String tempStr = String(temp);
    String humidStr = String(humid);    
    String message = "RUN " + tempStr + "\xDF" + "C, H" + humidStr;
    int remainingSpace = 16 - message.length() - 1; 
    String spaces = "";
    while (remainingSpace-- > 0) {
        spaces += " ";
    }
    message = "RUN " + spaces + tempStr + "\xDF" + "C, H" + humidStr + "%";
    lcd.print(message);
}

void DryBoxDisplay::PrintError(const char* errorMsg) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ERROR:");
    lcd.setCursor(0, 1);
    lcd.print(errorMsg);
}

void DryBoxDisplay::DisScrollText(const char* scrollMsg) {
    textLength = strlen(scrollMsg);
    scrollText(scrollMsg);
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
    lcd.print("[ Select ]    ");
    lcd.write(byte(223));
    lcd.print("C");
    lcd.setCursor(0, 1);
    lcd.print("Temp: 40C   H  %");
    break;

  case SCR_SETTEMP:						 
    lcd.print("[ Set Dry Temp ]");
    lcd.setCursor(0,1);
    lcd.print("40C        | RET");
    break;

  case SCR_SETTIME:						 
    lcd.print("[ Set Dry Time ]");
    lcd.setCursor(0,1);
    lcd.print("1:30  h:mm | RET");  
    break;

  case SCR_SETRPM:
    lcd.print("Set threshld RPM");
    lcd.setCursor(0,1);
    lcd.print("300    RPM | RET");  
    break;

  case SCR_RUNNING:	
    lcd.print("-RUN-          C");
    lcd.setCursor(0,1);
    lcd.print("35C h5:30| H   %");    
    break;

  case SCR_RUNBREAK:
    lcd.print("Run Break|");
    lcd.setCursor(0,1);
    lcd.print("cont stop|");      
    break;
  
  case SCR_TESTING:						 
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
