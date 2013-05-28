const int missingTooth = 2;      // the pin that the pushbutton is attached to
unsigned lastTooth = 0;          // Point in time that the last tooth occured
unsigned toothTime = 0;          // The time between teeth
unsigned crank_angle;            // The current crank angle
unsigned RPM = 0;                // The current engine RPM

// Throttle
const int throttlePin = 0;       // the pin that the throttle position sensor is on

// Lambda
const int lambdaPin = 1;         // the pin that the lambda sensor reading is on
float lambda = 512;              // The value read from the lambda analog input pin 512 should be lambda = 1
char lambdaDeadband = 20;        // The deadback for lambda feedback, don't adjust the output within this region from lambda = 512
boolean lambdaFeedback = false;  // Whether to use lambda feedback or the engine map

// Fuel
const int fuelPin = 12;          // the pin that the LED is attached to
unsigned fuelTime = calcTime(1000,64);// The fuel pulse timing delay, default value at this timer's prescaler
unsigned fuelDuration = calcTime(1000,64); // The fuel pulse duration, default value at this timer's prescaler

// Ignition
const int ignPin = 13;           // the pin that the ignition is attached to
unsigned ignTime = calcTime(1000,8); // The ignition delay time @TODO modify to be crank angle based
unsigned ignDuration = calcTime(1500,8); // The time that the ignition coil charges for

void setup() {
  // initialize the crank sensor as a input:
  pinMode(missingTooth, INPUT);
  // initialize the fuel as an output:
  pinMode(fuelPin, OUTPUT);
  
  // Turn the fuel off!
  digitalWrite(fuelPin,LOW);
  
  analogReference(EXTERNAL);
  //pinMode(throttlePin, INPUT);
  
  // initialize serial communication:
  Serial.begin(115200);
  
  attachInterrupt(0, missingToothISR, RISING);
  
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
  
  OCR3A = ignTime;  // Load the compare match register
  TCCR3B |= (1 << WGM32);   // CTC mode
  TCCR3B |= (1 << CS31);    // Load prescaler
  //TIMSK3 |= (1 << OCIE3A);  // enable timer compare interrupt
  
  interrupts();             // enable all interrupts
}

void loop() {
  //delay(5);
  
  lambda = (float)0.9999*lambda + (float)analogRead(lambdaPin)*0.0001;
  if(millis()%50 >45){
    if(lambda > 512 + lambdaDeadband && fuelDuration < 65534){
       fuelDuration += 1; 
    }else if(lambda < 512 - lambdaDeadband && fuelDuration > 1){
       fuelDuration -= 1; 
    }
  }
  if(millis()%500 >495){
    RPM = 1000000/toothTime;
    Serial.print(RPM);
    Serial.print("\t");
    Serial.print(fuelDuration);
    Serial.print("\t\t");
    Serial.print(fuelTime);
    Serial.print("\t\t");
    Serial.print(ignDuration);
    Serial.print("\t\t");
    Serial.print(ignTime);
    Serial.print("\t");
    Serial.print(lambda);
    Serial.print("\t");
    Serial.println(analogRead(throttlePin)/10.23);
    if(millis()%5000 >4995){
      Serial.print("RPM");
      Serial.print("\t");
      Serial.print("fuelDuration");
      Serial.print("\t");
      Serial.print("fuelTime");
      Serial.print("\t");
      Serial.print("ignDuration");
      Serial.print("\t");
      Serial.print("ignTime");
      Serial.print("\t");
      Serial.print("lambda");
      Serial.print("\t");
      Serial.println("throttle");
    }
  }
}

void serialEvent() {
  char temp = Serial.read();
  switch(temp){
    case 'f':
      fuelDuration = calcTime(Serial.parseInt(),64);
    break;
    case 'd':
      fuelTime = calcTime(Serial.parseInt(),64);
    break;
    case 'i':
      ignDuration = calcTime(Serial.parseInt(),8);
    break;
    case 't':
      ignTime = calcTime(Serial.parseInt(),8);
    break;
    case 'l':
      if(Serial.peek() == 'd'){
        lambdaDeadband = Serial.parseInt();
        lambdaFeedback = true;
      }else{
        // Turn lambda feedback off, and use the map instead
        lambdaFeedback = false; 
      }
    break;
  }
}

void missingToothISR(){
  unsigned temp = micros() - lastTooth;
  lastTooth = micros();

  if(temp > (2*toothTime) || crank_angle > 360){
    // Missing tooth detected
    
    // Sort out fuel timer
    TIFR1 |= 1 << OCF1A;        // Write a 1 to the interrupt flag to clear it
    TCNT1 = 0;                  // Reset the timer count to 0
    TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
    OCR1A = fuelTime;            // Load the compare match register
    
    // Start the ignition delay timer
    TIFR3 |= 1 << OCF3A;        // Write a 1 to the interrupt flag to clear it
    TCNT3 = 0;                  // Reset the timer count to 0
    TIMSK3 |= (1 << OCIE3A);    // enable timer compare interrupt
    OCR3A = ignTime;            // Load the compare match register
    
    crank_angle = 0;            // Reset the crank angle
    temp = temp/3;              // This is how long a tooth would have taken and allows for correct calculation of engine RPM
  }else{
    crank_angle+=3;    
        //Serial.println(temp);
  }

   toothTime = temp; 
}

// Deal with turning the fuel on and off
ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
   if(digitalRead(fuelPin) == LOW){
     // If fuel is off, then turn it on and set timer to turn it off
     digitalWrite(fuelPin,HIGH);
     TIFR1 |= 1 << OCF1A;        // Write a 1 to the interrupt flag to clear it
     TCNT1 = 0;                  // Reset the timer count to 0
     OCR1A = fuelDuration;        // Load the compare match register with the coil charge time
     
   }else{
     // If fuel was on, then turn it off
     digitalWrite(fuelPin,LOW);
     TIMSK1 &= ~(1 << OCIE1A);  // disable timer compare interrupt
   }
}

// Deal with turning the spark on and off
ISR(TIMER3_COMPA_vect)          // timer compare interrupt service routine
{
   
   if(digitalRead(ignPin) == LOW){
     // If ignition off, then turn it on and set timer to turn it off again 
     digitalWrite(ignPin,HIGH);
     TIFR3 |= 1 << OCF3A;        // Write a 1 to the interrupt flag to clear it
     TCNT3 = 0;                  // Reset the timer count to 0
     OCR3A = ignDuration;        // Load the compare match register with the coil charge time
     
   }else{
     // If ignition on, turn it off and disable timer until next missing tooth
     digitalWrite(ignPin,LOW);
     TIMSK3 &= ~(1 << OCIE3A);  // disable timer compare interrupt 
   }
}

// Do a short pulse on the given pin
// Can be used to show events as a pin output
void outputMarker(unsigned pin){
   digitalWrite(pin,LOW);
   delayMicroseconds(10);
   digitalWrite(pin,HIGH);
   delayMicroseconds(500);
   digitalWrite(pin,LOW); 
   delayMicroseconds(10);
}

//  Calculate the number of timer counts required for the given time with the given prescaler
//  Time is in nanoseconds.
//  Returns the integer value to load into the timer counter register
unsigned calcTime(unsigned long time, unsigned prescaler){
  return (int)(time/((float)prescaler/16));
}
