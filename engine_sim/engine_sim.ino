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

#define CDM PA_3
#define TRIGGER PA_2
#define PRESSURE PA_4

unsigned crank_angle;
int RPM;
long crankTimer;
long fuelTimer;
int fuelDuration;
int fuelDelay;
boolean toothFlag = false;
boolean fired = false;
boolean stroke = 0;

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
  
  pinMode(CDM,OUTPUT);
  pinMode(TRIGGER,OUTPUT);
  pinMode(PRESSURE,OUTPUT);
  
  RPM = 100;
  crank_angle = 0;
  initTimer((int)RPM);
  inString = "";
  fuelTimer = 0;
  fuelDelay = 0;
  
  
  Serial.begin(115200);
  
  Serial.println("Init");
  Serial.println(RPM);
  
  attachInterrupt(PUSH1, button1Push, CHANGE); // Button 1 is broken...
  attachInterrupt(PUSH2, button2Push, CHANGE);
  
  attachInterrupt(FUEL_PIN, fuelChange, CHANGE);
  attachInterrupt(IGN_PIN, ignChange, FALLING);
  
  engine.throttle = 100;
  setTimer(210*30);
}
 
void loop()
{
   digitalWrite(RED_LED,LOW);
   delay(50);
   
   // The value passed to simulate is the simulation timestep size in seconds
   // For real time accurate simulation it should be the same length of real time
   // between engine simulations calls, hence it is the same as the loop delay() value
   engine.simulate(0.05);
   
   lambdaOut = 51*(engine.lambda);
   
   //lambdaOut = map(engine.lambda, 0,5,0,255);
   
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
   
   if(millis() % 100 < 5){
     Serial.print("TPS");   Serial.print("\t");
     Serial.print("SPD");   Serial.print("\t");
     Serial.print("FDelay");   Serial.print("\t");
     Serial.print("Fuel");   Serial.print("\t");
     Serial.print("AFR");   Serial.print("\t");
     Serial.print("Lambda");   Serial.print("\t");
     Serial.print("LambdaOut");   Serial.print("\t");
     Serial.println("Ignition"); 
     
   }
   Serial.print(engine.throttle);   Serial.print("\t");
   Serial.print(engine.s);   Serial.print("\t");
   Serial.print(fuelDelay);   Serial.print("\t");
   Serial.print(engine.F);   Serial.print("\t");
   Serial.print(engine.AFR);   Serial.print("\t");
   Serial.print(engine.lambda);   Serial.print("\t");
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
    case 's':
      //Give the engine some initial speed similar to cranking it over
      engine.crank(Serial.parseInt());
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
  //Serial.println(engine.ignition/10.0);
  if(fired == true){
    Serial.println("multi spark!"); 
  }
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
    // Crank degree marker
    if(crank_angle%5 == 0){
      digitalWrite(CDM,!digitalRead(CDM)); 
    }
    
    // Output pressure wave
    if(stroke == 1 && crank_angle > 1000 && crank_angle < 2600){
      digitalWrite(PRESSURE,HIGH); 
    }else{
      digitalWrite(PRESSURE,LOW); 
    }
    
    // Missing tooth wheel (60/2) normal tooth
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
    
    // Missing tooth and TRIGGER marker
    if(crank_angle >= 3600){
      digitalWrite(TRIGGER,HIGH);
      //Serial.println(toothFlag);
      toothFlag = 0;
      crank_angle = 0;
      if(fired == false){
        //Serial.println("Missfire! ");
      };
      
      fired = false;
      fuelTimer = micros();
      digitalWrite(TRIGGER,LOW);
      
      stroke = !stroke;    // Swap 4 cycle stroke
    }
    
  }
}
