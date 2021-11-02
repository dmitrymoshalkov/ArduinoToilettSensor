/* ToDo

1. Добавить настройку расстояния для срабатывания присутствия через MQTT 
2. Добавить регулировку частоты опроса расстояния

*/


#include <Arduino.h>

#include <Ethernet.h>
#include <Wire.h>
#include <avr/wdt.h>
#include "EEPROMAnything.h"
#include <PubSubClient.h> // https://github.com/knolleary/pubsubclient


#define ALARM_PIN 3
#define ETHERNET_RESET_PIN  4
#define ARDUINO_RESET_PIN  A0
#define MQTTUPDATEINTERVAL 300
#define OCCUPIERLED_PIN A1
#define DOORLED_PIN A2
#define WATERCLOSE_PIN 8
#define PRESENCE_RANGE 880 //750
#define RANGECHECKINTERVAL 3000
#define MQTTCHECKINTERVAL 300000
#define MQTTINTERVAL 290000 




bool bOccupied = false;
bool bOccupiedLed = false;
bool bDoorstate = false;
bool bValvestate = false; //false - closed
uint8_t uinoWaterLeak = 1;
unsigned short uiRange = 0;
bool bUpdateMQTT = false;

unsigned short lenth_val = 0;
unsigned char i2c_rx_buf[16];
unsigned char dirsend_flag=0;

uint8_t mqttUnaviable = 0;

unsigned long lastRangeUpdateMillis = 0;
unsigned long currentRangeUpdateMillis = 0;
unsigned long lastMQTTUpdateMillis = 0;
unsigned long currentMQTTUpdateMillis = 0;

unsigned long currentCheckMQTTMillis = 0;
unsigned long lastMQTTCheckUpdate = 0;

byte mac[] = {
  0x00, 0x12, 0x1B, 0xBC, 0x12, 0x07
};


const char MQTTserver[]  = {"192.168.100.110"};
const char MQTTuser[]  = {"openhabian"};
const char MQTTpwd[]  = {"damp19ax"};
const char MQTTClientID[]  = {"toiletpres"};
const char MQTTtopic[]  = {"home/toilett/presence/state"};
const char MQTTtopicSet[]  = {"home/toilett/presence/set"};
const char willTopic[] = {"home/toilett/presence/LWT"};


#include "services.h"



void pop ()
{
  
  uinoWaterLeak = digitalRead(ALARM_PIN);

  if ( uinoWaterLeak == LOW && bUpdateMQTT == false)
  {
    digitalWrite(WATERCLOSE_PIN, LOW);
    bValvestate = false;
    bUpdateMQTT = true;
    //write eeprom relay state
    EEPROM_writeAnything (0, bValvestate);
  }
  else if (uinoWaterLeak == HIGH && bUpdateMQTT == false)
  {
    bUpdateMQTT = true;
  }


}


void setup() {

digitalWrite(ARDUINO_RESET_PIN,HIGH);
delay(200);   
pinMode(ARDUINO_RESET_PIN, OUTPUT);     

pinMode(WATERCLOSE_PIN, OUTPUT);
pinMode(OCCUPIERLED_PIN, OUTPUT);
pinMode(DOORLED_PIN, OUTPUT);

pinMode(ALARM_PIN, INPUT_PULLUP);

if(EEPROM_readAnything(0, bValvestate))
{
  digitalWrite(WATERCLOSE_PIN,bValvestate?HIGH:LOW);
}

Wire.begin();



   //     Serial.begin(115200);



// Reset the W5500 module
  pinMode(ETHERNET_RESET_PIN, OUTPUT);
  digitalWrite(ETHERNET_RESET_PIN, LOW);
  delay(800); //100
  digitalWrite(ETHERNET_RESET_PIN, HIGH);
  delay(800); //100



if ( Ethernet.begin(mac) == 0)
{

  //Enable watchdog timer
  // wdt_enable(WDTO_8S);



  resetBoard();


}




  pinMode(ALARM_PIN, INPUT_PULLUP);


  attachInterrupt(digitalPinToInterrupt(ALARM_PIN),pop,CHANGE);





setupMQTT();  

bUpdateMQTT = true;

}



void SensorRead(unsigned char addr,unsigned char* datbuf,unsigned char cnt) 
{
  unsigned short result=0;
  // step 1: instruct sensor to read echoes
  Wire.beginTransmission(82); // transmit to device #82 (0x52), you can also find this address using the i2c_scanner code, which is available on electroniclinic.com
  // the address specified in the datasheet is 164 (0xa4)
  // but i2c adressing uses the high 7 bits so it's 82
  Wire.write(byte(addr));      // sets distance data address (addr)
  Wire.endTransmission();      // stop transmitting
  // step 2: wait for readings to happen
  delay(1);                   // datasheet suggests at least 30uS
  // step 3: request reading from sensor
  Wire.requestFrom(82, cnt);    // request cnt bytes from slave device #82 (0x52)
  // step 5: receive reading from sensor
  if (cnt <= Wire.available()) { // if two bytes were received
    *datbuf++ = Wire.read();  // receive high byte (overwrites previous reading)
    *datbuf++ = Wire.read(); // receive low byte as lower 8 bits
  }
}
 
void ReadDistance(){

    currentRangeUpdateMillis = millis();

    if( (currentRangeUpdateMillis - lastRangeUpdateMillis ) > RANGECHECKINTERVAL )
    {

        SensorRead(0x00,i2c_rx_buf,2);
        lenth_val=i2c_rx_buf[0];
        lenth_val=lenth_val<<8;
        lenth_val|=i2c_rx_buf[1];
        uiRange = lenth_val;
        if ( lenth_val < PRESENCE_RANGE && bOccupied == false)
        {
          bOccupied = true;
          bUpdateMQTT = true;
          digitalWrite(OCCUPIERLED_PIN, HIGH);
          bOccupiedLed = true;
        }
        else if ( lenth_val >= PRESENCE_RANGE && bOccupied == true)
        {
          bOccupied = false;
          bUpdateMQTT = true;
          digitalWrite(OCCUPIERLED_PIN, LOW);
          bOccupiedLed = false;
        }
    
      lastRangeUpdateMillis = currentRangeUpdateMillis;
    
    }
    
}


void reportMQTT()
{

    currentMQTTUpdateMillis = millis();

    if( (currentMQTTUpdateMillis - lastMQTTUpdateMillis ) > MQTTCHECKINTERVAL || bUpdateMQTT )
    {
      sendDataToMQTT();
      bUpdateMQTT = false;

      lastMQTTUpdateMillis = currentMQTTUpdateMillis;
    }

}

void loop() {


    mqttAPI.loop();

  switch (Ethernet.maintain()) {
      case 1:
        //renewed fail

             resetBoard();
        break;

      case 2:
        //renewed success

        break;

      case 3:
        //rebind fail

             resetBoard();
        break;

      case 4:
        //rebind success

        break;

      default:
        //nothing happened
        break;
    }

    ReadDistance();





    currentCheckMQTTMillis = millis();

    if( ((currentCheckMQTTMillis - lastMQTTCheckUpdate ) > MQTTINTERVAL) || bUpdateMQTT )
    {
       checkMQTT();

       if (mqttAPI.connected())
       {
         
         mqttUnaviable =0;

       } else
       {
           
           mqttUnaviable++;

           if (mqttUnaviable > 21)
           {
              resetBoard();
           }
       }

         lastMQTTCheckUpdate = currentCheckMQTTMillis;
    }



    reportMQTT();

}



