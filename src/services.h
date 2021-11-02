#ifndef SERVICES_H
#define SERVICES_H


#include <PubSubClient.h>

#include <ArduinoJson.h>

 StaticJsonDocument<35> jsonBuffer;

EthernetClient ethClient;
PubSubClient mqttAPI(ethClient);




void checkMQTT();
void callback(char* topic, byte* payload, unsigned int length);




void sendDataToMQTT();


void resetBoard()
{


  digitalWrite(ETHERNET_RESET_PIN, LOW);
  delay(800); //100
  digitalWrite(ETHERNET_RESET_PIN, HIGH);
  
//asm volatile("jmp 0x00");
  digitalWrite(ARDUINO_RESET_PIN,LOW);

}




bool mqttPublish(String topic, char* data) {  
yield();

   Serial.println( data);

   return mqttAPI.publish(MQTTtopic, data, true);

}



void sendDataToMQTT() {
 

    if (mqttAPI.connected()) {



      char statusLine[110];

      sprintf(statusLine, "{\"waterleak\":%s,\"occupied\":%s,\"range\":%d,\"oculed\":\"%s\",\"valve\":\"%s\",\"doorled\":\"%s\"}",uinoWaterLeak?"false":"true",bOccupied?"true":"false", uiRange, bOccupiedLed?"ON":"OFF", bValvestate?"ON":"OFF",bDoorstate?"ON":"OFF"  );
      

       mqttPublish("", statusLine );
   
   
  }

}


uint8_t convertState(char *str)
{



  if (str[1] == 'N') return 1;

  return 0;
}


void callback(char* topic, byte* payload, unsigned int length) {



    auto error = deserializeJson(jsonBuffer, payload);
     if (!error) 
     {

       if (jsonBuffer.containsKey("valve"))
       {
        const char* vsensor = jsonBuffer["valve"];

        switch ( (int) convertState(vsensor) )
        {
          case 1:
            
            bValvestate = true;
            digitalWrite(WATERCLOSE_PIN,HIGH);
            break;
          case 0:
            bValvestate = false;    
            digitalWrite(WATERCLOSE_PIN,LOW);     
            break;             
        }

        EEPROM_writeAnything (0, bValvestate);


       }

       if (jsonBuffer.containsKey("doorled"))
       {
        const char* vdoorled = jsonBuffer["doorled"];

        switch (convertState(vdoorled))
        {
          case 1:
            bDoorstate = true;
            digitalWrite(DOORLED_PIN,HIGH);
            break;
          case 0:
            bDoorstate = false;    
            digitalWrite(DOORLED_PIN,LOW);  
            break;                
        }


       }

  
     }

//  }

sendDataToMQTT();


}



void setupMQTT() {
  if (Ethernet.localIP() ) {

    mqttAPI.setServer(MQTTserver, 1883);

    /* Устанавливаем обработчик */
   mqttAPI.setCallback(callback);
   


  }

}

void checkMQTT()
{
  yield();
  if (!mqttAPI.connected())
  {

      if( mqttAPI.connect(MQTTClientID, MQTTuser, MQTTpwd, willTopic, 1, true, "offline") )          
      {


           mqttAPI.subscribe(MQTTtopicSet);

           mqttAPI.publish(willTopic, "online", true);

      }

  }



}


#endif
