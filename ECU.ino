#include <PID_v1.h>
#include "includes.h"

// Missing tooth
unsigned long lastTooth = 0;          // Point in time that the last tooth occured
unsigned toothTime = 32000;          // The time between teeth
unsigned crank_angle;            // The current crank angle
unsigned RPM = 0;                // The current engine RPM
long testTimer = 0;

// Lambda
double lambda = 512;              // The value read from the lambda analog input pin 512 should be lambda = 1
unsigned lambdaDeadband = 20;        // The deadback for lambda feedback, don't adjust the output within this region from lambda = 512
double lambdaSetpoint = 512;
String fuelControl = "disabled";  // Whether to use lambda feedback or the engine map

// Fuel
unsigned fuelTime = calcTime(1000,64);// The fuel pulse timing delay, default value at this timer's prescaler
double fuelDuration = calcTime(4000,64); // The fuel pulse duration, default value at this timer's prescaler

// Ignition  
float ignAngle = 33; // The ignition delay time @TODO modify to be crank angle based
unsigned ignDuration = calcTime(1500,64); // The time that the ignition coil charges for
String ignControl = "TRIM";

// For PID
float kP = 0.1;
float kI = 5.0;
float kD = 0;

PID lambdaPID(&lambda,&fuelDuration,&lambdaSetpoint,kP,kI,kD,DIRECT); // Settings for map with lambda
void setup() {
  
  // initialize the crank sensor as a input:
  pinMode(TOOTH_PIN, INPUT);
  // initialize the fuel as an output:
  pinMode(FUEL_PIN, OUTPUT);
  
  // Turn the fuel off!
  digitalWrite(FUEL_PIN,LOW);
  
  analogReference(EXTERNAL);
  //pinMode(THROTTLE_PIN, INPUT);
  
  // initialize serial communication:
  Serial.begin(115200);
  
  attachInterrupt(0, missingToothISR, CHANGE);
  
  // PID
  lambdaPID.SetMode(AUTOMATIC);
  lambdaPID.SetOutputLimits(1, 65535);
  
   // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;               // Control register for waveform generation
  TCCR1B = 0;               // Turn off noise cancelling, turn off edge select, waveform gen mode 0, no clock source
  TCCR1B |= (1 << ICNC1);   // Turn on noise cancelling (samples 4 times before considered a change)
  TCNT1  = 0;               // Reset timer counter to 1
  
  OCR1A = fuelTime; // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // Load prescaler
  TCCR1B |= (1 << CS10);    // Load prescaler
  
  // initialise timer2
  TCCR3A = 0;               // Control register for waveform generation (off
  TCCR3B = 0;               // Turn off noise cancelling, turn off edge select, waveform gen mode 0, no clock source
  TCNT3  = 0;               // Reset timer counter to 1
  
  TCCR3B |= (1 << WGM32);   // CTC mode
  TCCR3B |= (1 << CS31);    // Load prescaler
  TCCR3B |= (1 << CS30);    // Load prescaler
  //TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt
  
  interrupts();             // enable all interrupts
}

void loop() {
  //delay(50);
  
  //lambda = (float)0.9*lambda + (float)analogRead(lambdaPin)*0.1; // low pass filter
  lambda = analogRead(LAMBDA_PIN);  // no filter
  //lambda = map(analogRead(lambdaPin),0,865,0,1023);    // Remap for max vals
  if(fuelControl == "ONOFF" && millis()%50 >45){
    if(lambda > 512 + lambdaDeadband && fuelDuration < 65534){
       fuelDuration += 1; 
    }else if(lambda < 512 - lambdaDeadband && fuelDuration > 1){
       fuelDuration -= 1; 
    }
  }else if(fuelControl == "PID"){ // Use the map if lambda is turned off
      // PID Library control
      lambdaPID.Compute();
  }else if(fuelControl == "TRIM"){
    
    // Use the analog inputs to set the fuelling
    fuelDuration = analogRead(FUEL_TRIM)*4;
    
  }else if(fuelControl == "MAP"){
    
    // Read the fuel value out of the map
    //fuelDuration = calcTime(fuelMap[0][round((float)analogRead(THROTTLE_PIN)/102.3)][round((float)RPM/500)],64);
    unsigned time = calcTime(fuelMap[0][round((float)analogRead(THROTTLE_PIN)/102.3)][round((float)RPM/500)],64);
    //Serial.print(time-lambdaDeadband);Serial.print("\t");Serial.println(time+lambdaDeadband);
    lambdaPID.SetOutputLimits(time-lambdaDeadband, time+lambdaDeadband);
    lambdaPID.Compute();
    
  }else if(fuelControl == "MAP_LAMBDA"){
    unsigned minVal,maxVal,tps,spd;
    // Use the fuel map to set the limits for the lambda control
    tps = round((float)analogRead(THROTTLE_PIN)/102.3);
    if(tps >=0 ){
      tps--;
    }
    
    spd = round((float)RPM/500);
    if(spd >=0){
      spd--;
    }
    
    minVal = calcTime(fuelMap[0][tps][spd],64);
    tps = round((float)analogRead(THROTTLE_PIN)/102.3);
    if(tps < MAP_TPS -1){
      tps++;
    }
    
    spd = round((float)RPM/500);
    if(spd < MAP_SPD -1){
      spd++;
    }
    
    maxVal = calcTime(fuelMap[0][tps][spd],64);
    //Serial.print(tps);Serial.print("\t");Serial.print(spd);Serial.print("\t");Serial.print(fuelMap[0][round((float)analogRead(THROTTLE_PIN)/102.3)-1][round((float)RPM/500)-1]);Serial.print("\t");Serial.println(fuelMap[0][tps][spd]);
    lambdaPID.SetOutputLimits(minVal, maxVal);
    lambdaPID.Compute();
  }
  
  if(ignControl == "TRIM"){
    ignAngle = 13 + (float)analogRead(IGN_TRIM)/20;
  }else if(ignControl == "MAP"){
    ignAngle = fuelMap[1][round((float)analogRead(THROTTLE_PIN)/102.3)][round((float)RPM/500)];
  }
  
  if(millis()%500 >495){
    RPM = 500000/toothTime;
    Serial.print(RPM);
    Serial.print("\t");
    Serial.print(fuelDuration*4);
    Serial.print("\t\t");
    Serial.print(fuelTime*4);
    Serial.print("\t");
    Serial.print(ignDuration*4);
    Serial.print("\t\t");
    Serial.print(ignAngle);
    Serial.print("\t");
    Serial.print(lambda);
    Serial.print("\t"); 
    Serial.print(lambdaSetpoint);
    Serial.print("\t");
    Serial.print(analogRead(FUEL_TRIM));
    Serial.print("\t");
    Serial.print(analogRead(IGN_TRIM));
    Serial.print("\t");
    Serial.println(analogRead(THROTTLE_PIN)/10.23);
    if(millis()%5000 >4995){
      Serial.println(fuelControl);
      Serial.print("RPM");
      Serial.print("\t");
      Serial.print("fuelDuration");
      Serial.print("\t");
      Serial.print("fuelTime");
      Serial.print(" ");
      Serial.print("ignDuration");
      Serial.print("\t");
      Serial.print("ignTime");
      Serial.print("\t");
      Serial.print("lambda");
      Serial.print("\t");
      Serial.print("lambdaSetpoint");
      Serial.print("\t");
      Serial.println("throttle");
    }
  }
  
}

void serialEvent() {
  char temp = Serial.read();
  //char next;
  switch(temp){
    case 'f':
      switch(Serial.read()){
          case 'x':
            Serial.println("Fuel Disabled");
            fuelControl = "disabled";
          break;
          case 'w':
            Serial.print("Set Fuel");
            fuelDuration = calcTime(Serial.parseInt(),64);
            Serial.println("Fuel feedback set to manual control");
            // Turn lambda feedback off, and use the map instead
            fuelControl = "manual"; 
          break;
          case 'd':
            fuelTime = calcTime(Serial.parseInt(),64);
            Serial.print("Fuel delay now set to: ");
            Serial.println(fuelTime);
          break;
          case 't':
          Serial.println("Fuel set to use trim pots");
            fuelControl = "TRIM";
          break;
      }
    case 'i':
      Serial.println("i is for Ignition");
      switch(Serial.read()){
        case 'x':
          Serial.println("Ignition Disabled");
          ignControl = "disabled";
        break;
        case 'a':
          ignAngle = Serial.parseInt();
          Serial.print("Ignition angle is now: ");Serial.println(ignAngle);
          ignControl = "manual";
        break;
        case 'd':
          ignDuration = Serial.parseInt();
          Serial.print("Ignition duration is now: ");Serial.println(ignDuration);
        break;
        case 't':
          Serial.println("ignition timing now from trim pots");
          ignControl = "TRIM";
        break;
        case 'm':
          Serial.println("m is for Map");
          switch(Serial.read()){
            case 's':
              Serial.println("Stored current values in map");
              Serial.println(round((float)analogRead(THROTTLE_PIN)/102.3));
              Serial.println(round((float)RPM/500));
              Serial.println(ignAngle);
              fuelMap[1][round((float)analogRead(THROTTLE_PIN)/102.3)][round((float)RPM/500)] = ignAngle;
            break;
            case 'e':
              ignControl = "MAP";
            break;
            case 'd':
              mapDisplay(1);
            break;
            case 'w':
              Serial.println("Write map to EEPROM");
              mapOutput(1);
            break;
            case 'r':
              Serial.println("Reset the map");
              mapReset(1);
            break;
            case 'h':
            default:
              Serial.println("s is for Store");
              Serial.println("w is for Write (output in a saveable format)");
              Serial.println("d is for Display");
              Serial.println("r is for Reset");
            break;
          }
        break;
        case 'h':
        default:
          Serial.println("a is for Angle of ignition advance");
          Serial.println("d is for Duration of coil charge time (in ms)");
          Serial.println("t is for Trim potentiometer for setting advance");
          Serial.println("m is for Map");
        break;
      }
    break;
    case 'l':
      Serial.println("L is for Lambda");
      switch(Serial.read()){
        case 'd':
          Serial.print("Fuel set to use ON OFF feedback new deadband:");
          lambdaDeadband = Serial.parseInt();
          Serial.println(lambdaDeadband);
          fuelControl = "ONOFF";
        break;
        case 's':
          Serial.println("Fuel set to use PID feedback");
          fuelControl = "PID";
          lambdaSetpoint = Serial.parseInt();
          lambdaPID.SetOutputLimits(1, 65535);
        break;
        case 'm':
          Serial.println("Fuel set to use map");
          Serial.print("new deadband:");
          lambdaDeadband = Serial.parseInt();
          Serial.println(lambdaDeadband);
          fuelControl = "MAP";
        break;
        case 'l':
          Serial.println("Fuel set to use map with lambda");
          fuelControl = "MAP_LAMBDA";
        break;
        case 'h':
        default:
          Serial.println("d is for Deadband and ONOFF control");
          Serial.println("s is for Setpoint and PID control");
          Serial.println("t is for Trimpot adjustment");
          Serial.println("m is for Map");
          Serial.println("l is for map with Lambda");
        break;
      }
    break;
    case 'm':
      Serial.println("m is for Map");
      switch(Serial.read()){
        case 's':
          Serial.println("Stored current values in map");
          Serial.println(round((float)analogRead(THROTTLE_PIN)/102.3));
          Serial.println(round((float)RPM/500));
          Serial.println(fuelDuration*4);
          fuelMap[0][round((float)analogRead(THROTTLE_PIN)/102.3)][round((float)RPM/500)] = fuelDuration*4;
    
        break;
        case 'd':
          mapDisplay(0);
        break;
        case 'w':
          Serial.println("Write map to EEPROM");
          mapOutput(0);
        break;
        case 'r':
          Serial.println("Reset the map");
          mapReset(0);
        break;
        case 'h':
        default:
          Serial.println("s is for Store");
          Serial.println("w is for Write (output in a saveable format)");
          Serial.println("d is for Display");
          Serial.println("r is for Reset");
        break;
      }
    break;
    case 'p':
      Serial.println("\np is for PID.\n");
      // 0.01 0.1 0
      switch(Serial.read()){
       case 'p':
         kP = Serial.parseFloat();
         Serial.print("Proportional is now: ");Serial.println(kP);
       break;
       case 'i':
         kI = Serial.parseFloat();
         Serial.print("Integral is now: ");Serial.println(kI);
       break;
       case 'd':
         kD = Serial.parseFloat();
         Serial.print("Derivative is now: ");Serial.println(kD);
       break;
       case 's':
         lambdaPID.SetSampleTime(Serial.parseInt());
       break;
       case 'h':
       default:
       
         Serial.print("p is for Proportional: ");Serial.println(kP);
         Serial.print("i is for Integral: ");Serial.println(kI);
         Serial.print("d is for Derivative: ");Serial.println(kD);
         Serial.println("s is for Sample time");
       break;
      }
      lambdaPID.SetTunings(kP,kI,kD);
      
    break;
    case 'h':
    default:
      Serial.println("\nh is for help.\n");
      Serial.println("m is for Map");
      Serial.println("l is for Lambda");
      Serial.println("i is for ignition");
      Serial.println("f is for fuel duration");
      Serial.println("d is for fuel timing\n");
      Serial.println("p is for PID");
    break;
  }  
  // Clear out anything left in the serial buffer (ignore it)
  while(Serial.available()){
   Serial.read(); 
  }

}

void missingToothISR(){
  long time = micros();
  long temp = time - lastTooth;
  

  if(temp > (5*toothTime) || crank_angle > 345){
    // Missing tooth detected
    //outputMarker(IGN_PIN);
    
    // Sort out fuel timer
    if(fuelControl != "disabled" && RPM > MIN_SPEED){
      TIFR1 |= 1 << OCF1A;        // Write a 1 to the interrupt flag to clear it
      TCNT1 = 0;                  // Reset the timer count to 0
      TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
      if(fuelTime > 0){
        OCR1A = fuelTime;           // Load the compare match register
      }else{
        digitalWrite(FUEL_PIN,HIGH);
        OCR1A = fuelDuration; 
      }
    }
    
    if(ignControl != "disabled" && RPM > MIN_SPEED ){ 
      // Start the ignition delay timer
      TIFR3 |= 1 << OCF3A;        // Write a 1 to the interrupt flag to clear it
      TCNT3 = 0;                  // Reset the timer count to 0
      TIMSK3 |= (1 << OCIE3A);    // enable timer compare interrupt
      
      //OCR3A = ignTime;            // Load the compare match register
      
      // Work out the timings for the fuel and ignition events based on the length of time the last tooth took
      // 1 tooth is 3 degrees, so toothTime = 3 degrees worth of microseconds
      // If the ignition pulse should finish 18 degrees before TDC then it has to start 18 degrees + duration of charge before TDC
      // Convert 18 degrees into microseconds: (18/3)*toothTime
      // Plus the time for charging: (18/3)*toothTime + ignDuration  = A
      // Time after this missing tooth is TOOTH_OFFSET in microseconds minus A
      // (TOOTH_OFFSET/3) * toothTime - ((18/3)*toothTime +ignDuration)
      // (TOOTH_OFFSET/3) * toothTime -(18/3)*toothTime -ignDuration
      // (TOOTH_OFFSET-18)*toothTime/3 - ignDuration
      // This method requires careful prescaler choice since the timer is fully utilised at the RPM limits 64 seems to work ok
      // The calculations also need to be cast to floats to avoid strange results
      OCR3A = calcTime((float)(TOOTH_OFFSET - ignAngle)*toothTime/3.0 - (ignDuration << 2) ,64);            // Load the compare match register
      //OCR3A = 1000;            // Load the compare match register
    }
    //Serial.print(time);Serial.print("\t");
//    Serial.print(lastTooth);Serial.print("\t");
//    Serial.print(temp);Serial.println("\t");
//    Serial.println(crank_angle);

    if(crank_angle <= 345){
      temp = temp/6.0;              // This is how long a tooth would have taken and allows for correct calculation of engine RPM
    }
    
    crank_angle = 0;            // Reset the crank angle
    

  }else{    
    crank_angle+=3;
    /*
    if(crank_angle >= (TOOTH_OFFSET - ignAngle) && crank_angle < (TOOTH_OFFSET - ignAngle + 3)){
      digitalWrite(IGN_PIN,HIGH);
      TIFR3 |= 1 << OCF3A;        // Write a 1 to the interrupt flag to clear it
      TCNT3 = 0;                  // Reset the timer count to 0
      TIMSK3 |= (1 << OCIE3A);    // enable timer compare interrupt
      OCR3A = (int)ignDuration;
    }*/
    
  }
  lastTooth = time;
   toothTime = (int)((float)0.9*toothTime + (float)0.1*temp);  
}

// Deal with turning the fuel on and off
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
   if(digitalRead(FUEL_PIN) == LOW){
     // If fuel is off, then turn it on and set timer to turn it off
     digitalWrite(FUEL_PIN,HIGH);
     TIFR1 |= 1 << OCF1A;        // Write a 1 to the interrupt flag to clear it
     TCNT1 = 0;                  // Reset the timer count to 0
     OCR1A = (int)fuelDuration;        // Load the compare match register with the coil charge time
     
   }else{
     // If fuel was on, then turn it off
     digitalWrite(FUEL_PIN,LOW);
     TIMSK1 &= ~(1 << OCIE1A);  // disable timer compare interrupt
   }
}

// Deal with turning the spark on and off
ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
   
   if(digitalRead(IGN_PIN) == LOW){
     // If ignition off, then turn it on and set timer to turn it off again 
     digitalWrite(IGN_PIN,HIGH);
     TIFR3 |= 1 << OCF3A;        // Write a 1 to the interrupt flag to clear it
     TCNT3 = 0;                  // Reset the timer count to 0
     //Serial.println(ignDuration);
     OCR3A = (int)ignDuration;        // Load the compare match register with the coil charge time
     testTimer = micros();
   }else{
     //Serial.println(micros()- testTimer);
     
     // If ignition on, turn it off and disable timer until next missing tooth
     digitalWrite(IGN_PIN,LOW);
     TIMSK3 &= ~(1 << OCIE3A);  // disable timer compare interrupt 
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

//  Calculate the number of timer counts required for the given time with the given prescaler
//  Time is in nanoseconds.
//  Returns the integer value to load into the timer counter register
unsigned calcTime(unsigned long time, unsigned prescaler){
  return (int)(time/(prescaler >> 4));
}
