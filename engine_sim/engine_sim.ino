/**
 * LM4F120 - timer based blink
 */
 
#include "timer_blink.h"
#include "Engine.h"

#define TOOTH_OFFSET 2400
#define THROTTLE_PIN A0
#define TOOTH_PIN PD_0
#define IGN_PIN PD_1
#define FUEL_PIN PD_2

#define LAMBDA_PIN PB_4

unsigned crank_angle;
int RPM;
long crankTimer;
long fuelTimer;
int fuelDuration;
int fuelDelay;
boolean toothFlag = false;
boolean fired = false;

boolean analogThrottle = true;

long ignTimer;
int ignDelay;
String inString;
int lambdaOut = 128;
 
Engine engine;
 
void setup()
{ 
  
  pinMode(TOOTH_PIN,OUTPUT);
  pinMode(RED_LED,OUTPUT);
  pinMode(BLUE_LED,OUTPUT);
  pinMode(PUSH2,INPUT);
  pinMode(PUSH1,INPUT);
  pinMode(PE_0,INPUT);
  
  pinMode(FUEL_PIN,INPUT);
  pinMode(IGN_PIN,INPUT);
  
  pinMode(LAMBDA_PIN,OUTPUT);
  
  RPM = 100;
  crank_angle = 0;
  initTimer((int)RPM);
  inString = "";
  fuelTimer = 0;
  fuelDelay = 0;
  
  
  Serial.begin(115200);
  
  Serial.println("Init");
  Serial.println(RPM);
  
  //attachInterrupt(PUSH1, button1Push, CHANGE); // Button 1 is broken...
  //attachInterrupt(PUSH2, button2Push, CHANGE);
  
  attachInterrupt(FUEL_PIN, fuelChange, CHANGE);
  attachInterrupt(IGN_PIN, ignChange, FALLING);
  
  engine.throttle = 1;
  setTimer(2896);
}
 
void loop()
{
    digitalWrite(RED_LED,LOW);
   delay(50);
   engine.simulate(0.05);
   
   lambdaOut = 128*(engine.lambda);
   
   if(lambdaOut > 255){
      lambdaOut = 255; 
   }
   if(lambdaOut < 0){
      lambdaOut = 0; 
   }
   
   // Output the AFR on an analog pin to be read by ECU
   analogWrite(LAMBDA_PIN,lambdaOut);
   
   if(true == analogThrottle){
      engine.throttle = analogRead(THROTTLE_PIN)/40.95;
   }
   
   
   Serial.print(engine.throttle);   Serial.print("\t");
   Serial.print(engine.s);   Serial.print("\t");
   Serial.print(fuelDelay);   Serial.print("\t");
   Serial.print(engine.F);   Serial.print("\t");
   Serial.print(engine.AFR);   Serial.print("\t");
   Serial.print(lambdaOut);   Serial.print("\t");
   Serial.println(engine.ignition/10.0);
   
   
   
   setTimer((int)engine.s*30);
}

void serialEvent(){
  char temp = Serial.read();
  switch(temp){
    case 't':
      Serial.println("Set Throttle");
      if(Serial.peek() != 'a'){
        engine.throttle = Serial.parseInt();
        analogThrottle = false;
        Serial.println("Throttle set by Serial");
      }else{
        analogThrottle = true;
        Serial.println("Throttle set by analog input");
      }
    break;
    case 'l':
      engine.TL = Serial.parseInt();
    break;
    case 'e':
      analogWrite(LAMBDA_PIN,Serial.parseInt());
    break;
  }
  // Clear out anything left in the serial buffer (ignore it)
  while(Serial.available()){
   Serial.read(); 
  }
}

void fuelChange(){
  //setTimer((int)engine.s);
  if(digitalRead(FUEL_PIN) == HIGH){
     fuelDelay = micros() - fuelTimer;
     fuelTimer = micros();
     digitalWrite(BLUE_LED,HIGH);
  }else{
     fuelDuration = micros() - fuelTimer;
     fuelTimer = micros();
     //engine.F = fuelDuration;
     engine.F = (float)0.2* engine.F + (float)0.8* fuelDuration;
     digitalWrite(BLUE_LED,LOW);
     //outputMarker(BLUE_LED);
  }
}

void ignChange(){
  engine.ignition = TOOTH_OFFSET - (int)crank_angle;
  fired = true;
}

void button1Push(){
  Serial.println("Button 1 Pushed!");
  engine.throttle -= 5;
}

void button2Push(){
  Serial.println("Button 2 Pushed!");
  // If the button is now low, then record the time
  if(digitalRead(PUSH2) == HIGH){
   engine.throttle += 5;
  }
}

// Do a short pulse on the given pin
// Can be used to show events as a pin output
void outputMarker(unsigned pin){
   digitalWrite(pin,LOW);
   delayMicroseconds(10);
   digitalWrite(pin,HIGH);
   delayMicroseconds(100);
   digitalWrite(pin,LOW); 
   delayMicroseconds(10);
}

extern "C" {
  void scott(){
    if(crank_angle < 3450 && crank_angle%30 == 0){
      //digitalWrite(RED_LED,0);
      if(toothFlag){
       toothFlag = false;
       digitalWrite(TOOTH_PIN,0);
      }else{
        toothFlag = true;
       digitalWrite(TOOTH_PIN,1);  
      }
    }else if(crank_angle%30 == 0){
      digitalWrite(TOOTH_PIN,0);
     // digitalWrite(RED_LED,1);
    }
    crank_angle += 1;
    if(crank_angle >= 3600){
      //Serial.println(toothFlag);
      toothFlag = 0;
      crank_angle = 0;
      if(fired == false){
        Serial.println("Missfire! ");
      };
      outputMarker(BLUE_LED);
      fired = false;
      fuelTimer = micros();
    }
    
  }
}
