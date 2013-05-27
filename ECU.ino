const int missingTooth = 2;     // the pin that the pushbutton is attached to
const int fuelPin = 12;       // the pin that the LED is attached to
const int ignPin = 13;       // the pin that the LED is attached to
unsigned lastTooth = 0;
unsigned toothTime = 0;
unsigned crank_angle;
unsigned revTimer;
unsigned RPM = 0;

void setup() {
  // initialize the button pin as a input:
  pinMode(missingTooth, INPUT);
  // initialize the LED as an output:
  pinMode(fuelPin, OUTPUT);
  
  // Turn the fuel off!
  digitalWrite(fuelPin,LOW);
  
  // initialize serial communication:
  Serial.begin(115200);
  
  attachInterrupt(0, missingToothISR, RISING);
  
   // initialize timer1 
  noInterrupts();           // disable all interrupts
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;
  
  OCR1A = 1100;             // compare match register 16MHz/256/2Hz
  TCCR1B |= (1 << WGM12);   // CTC mode
  TCCR1B |= (1 << CS11);    // 256 prescaler
  
  interrupts();             // enable all interrupts
}

void loop() {
  delay(500);
  RPM = 1000000/toothTime;
  Serial.println(RPM);
  Serial.println(OCR1A);
}

void serialEvent() {
  char temp = Serial.read();
  if(temp == 'f'){
    OCR1A = Serial.parseInt();
  }else if(temp == 'f'){
    TIMSK1 &= ~(1 << OCIE1A);  // disable timer compare interrupt  
  }else{
    
  }
  
}

void missingToothISR(){
  unsigned temp = micros() - lastTooth;
  lastTooth = micros();

  if(temp > (2*toothTime) || crank_angle > 360){
    // Missing tooth detected
    digitalWrite(fuelPin,HIGH); 
    
    // Sort out timer
    TIFR1 |= 1 << OCF1A;        // Write a 1 to the interrupt flag to clear it
    TCNT1 = 0;                  // Reset the timer count to 0
    TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
    
    crank_angle = 0;            // Reset the crank angle
  }else{
    crank_angle+=3;    
  }

   toothTime = temp; 
}

ISR(TIMER1_COMPA_vect)          // timer compare interrupt service routine
{
   digitalWrite(fuelPin,LOW);
  TIMSK1 &= ~(1 << OCIE1A);  // disable timer compare interrupt
}

void testISR(){
  if(digitalRead(fuelPin)){
   digitalWrite(fuelPin,LOW);  
  }else{
   digitalWrite(fuelPin,HIGH);  
  }
}
