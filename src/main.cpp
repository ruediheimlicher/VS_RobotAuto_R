/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp-now-one-to-many-esp32-esp8266/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>

#include <Ticker.h> 
#include "elapsedMillis.h"
#include "expo.h"

// von ESP32 ROBOTAUTO
#ifndef LED_BUILTIN
#define LED_BUILTIN 16
#endif

#define NUM_SERVOS 4

#define MAX_TICKS 3400
#define MIN_TICKS 1700
uint16_t   servomittearray[NUM_SERVOS] = {}; // Werte fuer Mitte

uint16_t maxwinkel = 180;

uint8_t buttonstatus = 0;
uint8_t tonindex = 0;
void playTon(int ton);
#define START_TON 1
#define LICHT_ON 2

#define TON_PIN 4
elapsedMillis tonposition;
uint8_t expolevel = 3;

uint16_t ubatt = 0;



int ledintervall = 1000;
Ticker timer;
elapsedMillis ledmillis;
elapsedMillis tonmillis;
int tonfolge[3] = {554, 329, 440};

struct ServoPins
{
  Servo servo;
  int servoPin;
  String servoName;
  int initialPosition;  
};
std::vector<ServoPins> servoPins = 
{
  { Servo(), 12 , "Dir", 90}, // Richtung
  { Servo(), 14 , "Pitch", 90}, // Gas
  { Servo(), 13 , "Elbow", 90},
  { Servo(), 15 , "Gripper", 90},
};

struct RecordedStep
{
  int servoIndex;
  int value;
  int delayInStep;
};
std::vector<RecordedStep> recordedSteps;

bool recordSteps = false;
bool playRecordedSteps = false;

unsigned long previousTimeInMilli = millis();

void playTon(int ton)
{
  // Cis: 554
  // e: 330
  // a: 440
  tone(TON_PIN,tonfolge[ton],800);
}


void writeServoValues(int servoIndex, int value)
{
  if (recordSteps)
  {
    RecordedStep recordedStep;       
    if (recordedSteps.size() == 0) // We will first record initial position of all servos. 
    {
      for (uint8_t i = 0; i < servoPins.size(); i++)
      {
        recordedStep.servoIndex = i; 
        recordedStep.value = servoPins[i].servo.read(); 
        recordedStep.delayInStep = 0;
        recordedSteps.push_back(recordedStep);         
      }      
    }
    unsigned long currentTime = millis();
    recordedStep.servoIndex = servoIndex; 
    recordedStep.value = value; 
    recordedStep.delayInStep = currentTime - previousTimeInMilli;
    recordedSteps.push_back(recordedStep);  
    previousTimeInMilli = currentTime;         
  }
  //Serial.printf("pin: [%d] servoindex: [%d] value: [%d]  \n",servoPins[servoIndex].servoPin , servoIndex, value);
  servoPins[servoIndex].servo.write(value);   
}



//Structure example to receive data
//Must match the sender structure
typedef struct canal_struct 
{
  uint8_t lx;
  uint8_t ly;
  uint8_t rx;
  uint8_t ry;

 uint8_t digi;

  uint8_t x;
  uint8_t y;
} canal_struct;

//Create a struct_message called canaldata
canal_struct canaldata;
//structindexarray[] = {"lx","ly","rx","ry"};

uint16_t tickslimited(uint16_t inticks)
{
  if(inticks > MAX_TICKS)
  {
    return  MAX_TICKS;
  }
  else if (inticks < MIN_TICKS)
  {
    return MIN_TICKS;
  }
 return inticks;
}

uint16_t servoticks(uint16_t inticks)
{
  uint16_t expovalue = 0;
  inticks = map(inticks,MIN_TICKS,MAX_TICKS, 0,maxwinkel);
  if (inticks > maxwinkel/2)
  {
     expovalue = maxwinkel/2 +  expoarray[expolevel][inticks - maxwinkel/2];
  }
  else
  {
     expovalue = maxwinkel/2 -  expoarray[expolevel][maxwinkel/2 - inticks ];
  }
 return expovalue;
}

//callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac, uint8_t *incomingData, uint8_t len) 
{
  // blink
  memcpy(&canaldata, incomingData, sizeof(canaldata));
 
  Serial.print("Bytes received: ");
  //Serial.println(mac);

/*
  Serial.print("lx: ");
  Serial.print(canaldata.lx);
  Serial.print(" ");
  Serial.print("ly: ");
  Serial.print(canaldata.ly);
  Serial.print(" ");
  Serial.print("digi: ");
  Serial.println(canaldata.digi);
*/
 // lx
 
  // 
  uint8_t lx = canaldata.lx;
  writeServoValues(0,  lx);
  uint8_t ly = canaldata.ly;
  writeServoValues(1,  ly);
//Serial.printf("lx: %d ly: %d \n",lx,ly);
  /*
  uint16_t outvalue_lx = servoticks(lx);
  Serial.print(" outvalue_lx: ");
  Serial.println(outvalue_lx);
 
 //writeServoValues(1, outvalue_lx); // red Ausschlaege
 */
  
 // ly
 // Tickbereich einhalten
 /*
  uint16_t ly = tickslimited(canaldata.ly);
  
  uint16_t outvalue_ly = servoticks(ly);

  Serial.print(" outvalue_ly: ");
  Serial.println(outvalue_ly);
 
 //writeServoValues(0, outvalue_ly); // red Ausschlaege
*/

if (canaldata.digi & (1<<START_TON))
{
  if (!(buttonstatus & (1<<START_TON)))
  {
    //Serial.println("digi start");
    //Serial.println(canaldata.digi);
    buttonstatus |= (1<<START_TON);
    tonindex = 0;

  }
  
}
}

void setUpPinModes()
{

  for (uint8_t i = 0; i < servoPins.size(); i++)
  {
    servoPins[i].servo.attach(servoPins[i].servoPin);
    servoPins[i].servo.write(servoPins[i].initialPosition);    
  }
 
}


 
void setup() {
  //Initialize Serial Monitor
  Serial.begin(115200);
  pinMode(TON_PIN,OUTPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  // https://community.platformio.org/t/esp8266-gibberish-serial-monitor-output/30027
  //Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  //Init ESP-NOW
  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  setUpPinModes();
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

}
 
void loop() 
{
  
  if (ledmillis >= ledintervall)
  {
    ledmillis = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

    

    //Serial.println("led");
  }

  if (buttonstatus & (1<<START_TON))
  {
    
    if (tonindex == 0)
    {
      tonmillis = 0;
      //Serial.print("************ TON start index: ");
      //Serial.println(tonindex);
      playTon(tonindex);
      tonindex++;
    }
    
    
    if (tonindex < 3)
    {
        if (tonmillis > 850)
        {
          //Serial.print("************ TON next index: ");
          //Serial.println(tonindex);
          playTon(tonindex);
          tonindex++;
          
          tonmillis = 0;
        }
        
    }
    else
    {
      Serial.print("************ TON END");
      buttonstatus &= ~(1<<START_TON);
      tonindex = 0;
    }
    
  }

}
