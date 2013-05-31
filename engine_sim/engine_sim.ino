/**
 * LM4F120 - timer based blink
 */
 
#include "timer_blink.h"
#include "Engine.h"

#define TOOTH_OFFSET 240
#define THROTTLE_PIN A0
#define TOOTH_PIN PD_0
#define IGN_PIN PD_1
#define FUEL_PIN PD_2

#define LAMBDA_PIN PB_4

int crank_angle;
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
   
   if(analogThrottle == true){
      engine.throttle = analogRead(THROTTLE_PIN)/40.95;
   }
   
   
   Serial.print(engine.throttle);
   Serial.print("\t");
   Serial.print(engine.s);
   Serial.print("\t");
   Serial.print(engine.AFR);
   Serial.print("\t");
   Serial.print(engine.ignition);
   Serial.print("\t");
   Serial.println(lambdaOut);
   
   
   setTimer((int)engine.s);
}

void serialEvent(){
  char temp = Serial.read();
  switch(temp){
    case 't':
      if(Serial.peek() != 'a'){
        engine.throttle = Serial.parseInt();
        analogThrottle = false;
      }else{
        analogThrottle = true;
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
  }else{
     fuelDuration = micros() - fuelTimer;
     fuelTimer = micros();
     engine.F = fuelDuration;
  }
}

void ignChange(){
  engine.ignition = TOOTH_OFFSET - crank_angle;
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

extern "C" {
  void scott(){
    
    if(crank_angle < 345){
      digitalWrite(RED_LED,0);
      if(toothFlag){
       toothFlag = false;
       digitalWrite(TOOTH_PIN,0);
      }else{
        toothFlag = true;
       digitalWrite(TOOTH_PIN,1);  
      }
    }else{
      digitalWrite(TOOTH_PIN,0);
      digitalWrite(RED_LED,1);
    }
    crank_angle += 3;
    if(crank_angle == 360){
      crank_angle = 0;
      if(fired == false){
        Serial.println("Missfire!");
      };
      fired = false;
    }
    
  }
}
