#include <PID_v1.h>
#include "includes.h"

// Missing tooth
unsigned long lastTooth = 0;          // Point in time that the last tooth occured
float toothTime = 32000;          // The time between teeth
unsigned crank_angle;            // The current crank angle
unsigned RPM = 0;                // The current engine RPM
long testTimer = 0;

// Lambda
double lambda = 1;              // The value read from the lambda analog input pin 512 should be lambda = 1
unsigned lambdaDeadband = 20;        // The deadback for lambda feedback, don't adjust the output within this region from lambda = 512
double lambdaSetpoint = 1;

// Fuel
unsigned fuelTime = calcTime(1000,64);// The fuel pulse timing delay, default value at this timer's prescaler
double fuelDuration = calcTime(4000,64); // The fuel pulse duration, default value at this timer's prescaler
String fuelControl = "disabled";  // Whether to use lambda feedback or the engine map

// Ignition  
float ignAngle = TOOTH_OFFSET - 22; // The ignition delay angle before TDC
unsigned ignDuration = calcTime(8000,64); // The time that the ignition coil charges for
String ignControl = "TRIM";
unsigned ignTimeVal = 0;

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
  // Don't enable just yet though
  
  interrupts();             // enable all interrupts
}

void loop() {
  //delay(50);
  
  // Calculate the ignition timer value
  ignTimeVal = ignAngle *toothTime/12 - (ignDuration - (900 - toothTime));

  
  //Serial.print(toothTime);  Serial.print("\t");Serial.println(ignTimeVal);
  /* **********************************************************************
  *************************** FUEL CONTROL *********************************
  **************************************************************************/
  //lambda = (float)0.9*lambda + (float)analogRead(lambdaPin)*0.1; // low pass filter
  lambda = analogRead(LAMBDA_PIN)/204.6;  // no filter
  //lambda = map(analogRead(LAMBDA_PIN),0,1023.0,0,5.0);    // Remap for max vals
  if(fuelControl == "ONOFF" && millis()%50 >45){
    if(lambda > lambdaSetpoint + lambdaDeadband && fuelDuration < 65534){
       fuelDuration += 1; 
    }else if(lambda < lambdaSetpoint - lambdaDeadband && fuelDuration > 1){
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
  
  /*************************************************************************
  *************************** IGNITION CONTROL *****************************
  **************************************************************************/
  
  if(ignControl == "TRIM"){
    ignAngle = TOOTH_OFFSET - (13 + (float)analogRead(IGN_TRIM)/20);
  }else if(ignControl == "MAP"){
    ignAngle = TOOTH_OFFSET - fuelMap[1][round((float)analogRead(THROTTLE_PIN)/102.3)][round((float)RPM/500)];
  }
  
  /*************************************************************************
  *************************** SERIAL OUTPUT ********************************
  **************************************************************************/
  RPM = 500000/toothTime;
  
  if(millis()%500 >495){
    Serial.print(RPM);Serial.print("\t");
    Serial.print(fuelDuration*4); Serial.print("\t\t");
    Serial.print(fuelTime*4); Serial.print("\t\t");
    Serial.print(ignDuration*4); Serial.print("\t\t");
    Serial.print(TOOTH_OFFSET - ignAngle); Serial.print("\t\t");
    Serial.print(lambda); Serial.print("\t"); 
    //Serial.print(OCR3A); Serial.print("\t"); 
    Serial.print(lambdaSetpoint); Serial.print("\t\t");
    Serial.println(analogRead(THROTTLE_PIN)/10.23);
    if(millis()%5000 >4995){
      Serial.print("Fuel control:"); Serial.println(fuelControl);
      Serial.print("RPM");  Serial.print("\t");
      Serial.print("fuelDuration");  Serial.print("\t");
      Serial.print("fuelTime");  Serial.print("\t");
      Serial.print("ignDuration");  Serial.print("\t");
      Serial.print("ignAngle");  Serial.print("\t");
      Serial.print("lambda");  Serial.print("\t");
      Serial.print("lambdaSetpoint");  Serial.print("\t");
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
          case 'f':
            Serial.println("F is for feedback");
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
          case 'h':
          default:
            Serial.println("x is to disable");
            Serial.println("w is for Width (fuel pulse width)");
            Serial.println("d is for Delay of fuel pulse");
            Serial.println("t is for Trim pots");
            Serial.println("f is for Feedback");
            Serial.println("m is for Map");
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
          ignAngle = TOOTH_OFFSET - Serial.parseInt();
          Serial.print("Ignition angle is now: ");Serial.println(ignAngle);
          ignControl = "manual";
        break;
        case 'd':
          ignDuration = calcTime(Serial.parseInt(),64);
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
              fuelMap[1][round((float)analogRead(THROTTLE_PIN)/102.3)][round((float)RPM/500)] = TOOTH_OFFSET + ignAngle;
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
          Serial.println("x is to disable igniton");
          Serial.println("a is for Angle of ignition advance");
          Serial.println("d is for Duration of coil charge time (in ms)");
          Serial.println("t is for Trim potentiometer for setting advance");
          Serial.println("m is for Map");
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
    case 's':
      fuelControl = "disabled";
      ignControl = "disabled";
    break;
    case 'h':
    default:
      Serial.println("\nh is for help.\n");
      Serial.println("f is for Fuel");
      Serial.println("i is for Ignition");
      Serial.println("p is for PID");
      Serial.println("s is for Stop the engine");
    break;
  }  
  // Clear out anything left in the serial buffer (ignore it)
  while(Serial.available()){
   Serial.read(); 
  }

}

void missingToothISR(){
  unsigned long time = micros();
  unsigned temp = time - lastTooth;
  

  if(crank_angle >= 345 && temp > ((unsigned)toothTime<<2) ){
    // Missing tooth detected

    // Only enable the timer interrupts if it's safe and engine is ready
    // e.g. there's no point in sparking if the engine is stopped!
    if(ignControl != "disabled" && RPM > MIN_SPEED){
      // Start the ignition delay timer
      TIFR3 |= 1 << OCF3A;        // Write a 1 to the interrupt flag to clear it
      TCNT3 = 0;                  // Reset the timer count to 0
      TIMSK3 |= (1 << OCIE3A);    // enable timer compare interrupt
    }
    
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
    
    

    if(crank_angle <= 345){
      temp = temp/5.0;              // This is how long a tooth would have taken and allows for correct calculation of engine RPM
    }
    
    crank_angle = 0;            // Reset the crank angle
    

  }else{
    // Load up the timer if the ignition's not already on, and if it's
    // Not already close to the timer elapsing
    if(bitRead(PORTB,7) == LOW && ((OCR3A - TCNT3) << 2) > toothTime  ){
      OCR3A = ignTimeVal;            // Load the compare match register
    }
    
    crank_angle+=3;
  }
  
  lastTooth = time;
  
  // Under heavy acceleration, the tooth width will change drastically
  // To avoid trying to average this out (which we don't want)
  // We ignore large changes, and simply reset the average value.
  // This also helps in the initial sitution where toothTime is very large.
  // The size of this factor depends on how much the engine can accerlate over 1 tooth
  if(abs(temp-toothTime) < MAX_TOOTH_CHANGE){
    toothTime = (float)0.9*toothTime + (float)0.1*temp; 
  }else{
    toothTime = temp; 
  }
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
   
   if(digitalRead(IGN_PIN) == HIGH){
   //if(bitRead(PORTB,7) == HIGH){
     //Serial.println(micros()- testTimer);
     
     // If ignition on, turn it off and disable timer until next missing tooth
     digitalWrite(IGN_PIN,LOW);
//     bitWrite(PORTB,7,0);
     TIMSK3 &= ~(1 << OCIE3A);  // disable timer compare interrupt 
     
   }else{
     // If ignition off, then turn it on and set timer to turn it off again 
     digitalWrite(IGN_PIN,HIGH);
     //bitWrite(PORTB,7,1);
     TIFR3 |= 1 << OCF3A;        // Write a 1 to the interrupt flag to clear it
     TCNT3 = 0;                  // Reset the timer count to 0
     //Serial.println(ignDuration);
     OCR3A = (int)ignDuration- (900 - toothTime);        // Load the compare match register with the coil charge time
     testTimer = micros();
   }
}


