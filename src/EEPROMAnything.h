#ifndef EEPROMAnything_h
#define EEPROMAnything_h

#include "EEPROM.h"
#if ARDUINO < 100
#include <WProgram.h>
#else
#include <Arduino.h>
#endif

template <class T> int EEPROM_writeAnything(int ee, const T& value)
{
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        EEPROM.update(ee++, *p++);
    return i;
}

template <class T> int EEPROM_readAnything(int ee, T& value)
{
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
        *p++ = EEPROM.read(ee++);
    return i;
}

#endif

/*
#include <EEPROM.h>
#include "EEPROMAnything.h"

char string1; //where EEPROM_readAnything will save data to

void setup()
{
  char* fox = "The quick brown fox"; //SEND TO EEPROM
  byte len = (byte) strlen (fox);
  EEPROM_writeAnything (0, len);
  EEPROM_writeAnything (1, fox);
}
void loop()
{
   if(EEPROM_readAnything(0, string1))
    Serial.println("marker"); //SEE if anything happens on SM
    Serial.println(string1);
}
*/