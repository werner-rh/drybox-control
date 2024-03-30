/***
 * Project: Toolfunctions for Key input State
 * File   : WRKeyState.cpp
 * Author : Werner Riemann 
 * Created: 25.07.2022
 * Board: Arduino Nano
 * 
 * Description: Software debounce of input buttons
 * 
 * 
 * 
 */

#include "WRKeyStateDef.h"

uint8_t CheckKeyState(uint8_t * KeyState, uint8_t KeyPin)
{
 uint8_t RetState = 0;
 uint8_t curKeyVal = digitalRead(KeyPin);   // 0,LOW = Key down ; 1,HIGH = Key up

 if(* KeyState == 0 && curKeyVal == LOW) //Taster wird gedrueckt (steigende Flanke)   
   {   
   * KeyState = 1;   
   RetState = 1;   
   }   
 else if (* KeyState == 1 && curKeyVal == LOW) //Taster wird gehalten   
   {   
   * KeyState = 2;   
   RetState = 0;   
   }
 else if (* KeyState == 2 && curKeyVal == HIGH) //Taster wird losgelassen (fallende Flanke)
   {
   * KeyState = 3;
   RetState = 0;
   }
   else if (* KeyState == 3 && curKeyVal == HIGH)  //Taster losgelassen
   {
   * KeyState = 0;
   RetState = 0;
   }
   else if (* KeyState == 1 && curKeyVal == HIGH)  //Taster losgelassen
   {
   * KeyState = 0;
   RetState = 0;
   }

 return RetState;
}

