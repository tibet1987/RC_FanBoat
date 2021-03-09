
// variables for reverseDirection PPM signal read-in
const byte reverseDirection_interruptPin = 3;
int revDirLastPosEdge_micros = 0;
int revDirPulseWidth_micros= 0;
int revDirThreshold = 1200;

// variables for motor throttle PPM signal read-in
const byte motorThrottle_interruptPin = 2;
int motorThrottleLastPosEdge_micros = 0;
int motorThrottlePulseWidth_micros= 0;
int ppmSig_lowVal = 1100;
int ppmSig_upMinLowVal = 1984;
long ppmSig_min1000 = 0;
long ppmSig_max2000 = 0;
boolean forwardDirectionState = false; // false: such that positive edge is provoked and switches to forward mode initially
boolean directionForwardRequest = true;
uint8_t testState = 0;

// signal for PWM generation
int sensorValue = 0; // test-signal from poti
int fanPWMSig_fromPPMSig = 0;

#define Lpositive_ON TCCR1A |= (1 << COM1B1)
#define Lpositive_OFF TCCR1A &= ~(1 << COM1B1)

#define Lnegative_ON TCCR1A |= (1 << COM1A1)
#define Lnegative_OFF TCCR1A &= ~(1 << COM1A1)

#define Hpositive_ON PORTB |= (1 << PORTB0)
#define Hpositive_OFF PORTB &= ~(1 << PORTB0)

#define Hnegative_ON PORTD |= (1 << PORTD7)
#define Hnegative_OFF PORTD &= ~(1 << PORTD7)

void setup() {
  Serial.begin(9600);
  pinMode(motorThrottle_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(motorThrottle_interruptPin),motorThrottle_pinChangeDetected,CHANGE);

  pinMode(reverseDirection_interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(reverseDirection_interruptPin),reverseDirection_pinChangeDetected,CHANGE);
  
  // setup PWM 
  TCCR1A = 0; // reset all registers
  DDRB = (1 << DDB2);     // PB2 is now an output for OC1B -> L+  (D10)
  DDRB |= (1 << DDB1);    // PB1 is now an output for OC1A -> L-  (D9)
  DDRB |= (1 << DDB0);    // PB0 is now an output -> H+  (D8)
  DDRD = (1 << DDD7);     // PD7 is now an output -> H-  (D7)

  // set up PWM outputs
  TCCR1A |= (1 << WGM10);  // set phase correct Mode (takes double the time for a PWM cycle)
  TCCR1B = (1 << CS10);    // set prescaler to 0 (32.25kHz) and start PWM
  OCR1A = 0x00;            // set PWM for 50% duty cycle
  OCR1B = 0x00;            // set PB2 PWM for 50% duty cycle

  // turn off powerstage completely in the beginning
  Hpositive_OFF;
  Hnegative_OFF;
  Lpositive_OFF;
  Lnegative_OFF;
}

void loop() {  
  // condition to change 'directionForwardRequest'
  if(revDirPulseWidth_micros < revDirThreshold){
    directionForwardRequest=false;
  }
  else{
    directionForwardRequest=true;
  }
  
  // switch to change motor throttle direction
  if( (forwardDirectionState==false) && (directionForwardRequest==true) ){
    // switching to forward mode
    Hnegative_OFF;
    Lpositive_OFF;
    delay(1); // safety waiting time not to cause half-bridge short-circuits
    Hpositive_ON;
    Lnegative_ON;
    forwardDirectionState = directionForwardRequest; // update state
    testState = 1;
  }
  else if( (forwardDirectionState==true) && (directionForwardRequest==false) ){
    // switching to backward mode
    Hpositive_OFF;
    Lnegative_OFF;
    delay(1); // safety waiting time not to cause half-bridge short-circuits
    Hnegative_ON;
    Lpositive_ON;
    forwardDirectionState = directionForwardRequest; // update state
    testState = 2;
  }
  
  // make sure the PPM pulse duration has its lowest value at "nm" and has a maximum value add of 1000µs
  ppmSig_min1000 = (long)min(max(motorThrottlePulseWidth_micros-ppmSig_lowVal,0),1000);
  // now make sure that the PPM pulse is not bigger than "ppmSig_upMinLowVal"
  ppmSig_max2000 = min(ppmSig_min1000+ppmSig_lowVal-1000+ppmSig_upMinLowVal,2000);

  // scale ppmSig_min1000 range of 0..1000 down to 0..255
  //ppmSig_max2000 = (ppmSig_min1000 *(long)255) / (long)1000;
  OCR1A = min( (uint8_t)( (ppmSig_min1000 *(long)255) / (long)1000), 255);
  OCR1B = OCR1A; // forward and backward have the same duty cycle (distinguished by forward-backward-bit)
  Serial.print("Fwd/Bwd: ");
  Serial.print(forwardDirectionState,DEC);
  Serial.print(",  motorThrottle: ");
  Serial.print(motorThrottlePulseWidth_micros,DEC);  
  Serial.print(",  OCR1A/B: ");
  Serial.print(OCR1A,DEC);
  Serial.print(",  ppmSig_min1000: ");
  Serial.print(ppmSig_min1000,DEC);
  Serial.print(",  ppmSig_max2000: ");
  Serial.println(ppmSig_max2000,DEC);
}

void motorThrottle_pinChangeDetected(){
  if(digitalRead(motorThrottle_interruptPin)==LOW){
    motorThrottlePulseWidth_micros = micros() - motorThrottleLastPosEdge_micros;
  }
  else{
    motorThrottleLastPosEdge_micros = micros();
  }
}

void reverseDirection_pinChangeDetected(){
  if(digitalRead(reverseDirection_interruptPin)==LOW){
    revDirPulseWidth_micros = micros() - revDirLastPosEdge_micros;
  }
  else{
    revDirLastPosEdge_micros = micros();
  }
}
