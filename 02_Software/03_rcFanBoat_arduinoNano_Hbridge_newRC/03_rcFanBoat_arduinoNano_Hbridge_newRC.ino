
// variables for reverseDirection PPM signal read-in
const byte reverseDirection_interruptPin = 3;
int revDirLastPosEdge_micros = 0;
int revDirPulseWidth_micros= 0;
int revDirThreshold = 1500;

// variables for motor throttle PPM signal read-in
const byte motorThrottle_interruptPin = 2;
int motorThrottleLastPosEdge_micros = 0;
int motorThrottlePulseWidth_micros= 0;
int ppmSig_bottomVal = 1000;
int ppmSig_topVal = 2000;
int ppmSig_min = 0;
int ppmSig_min_max = 0;
int directionState = 0; // 0=undefined, 1=fwd, 2=bwd
boolean directionBackwardRequest = false;

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
  // condition to change 'directionBackwardRequest'
  if(revDirPulseWidth_micros < revDirThreshold){
    directionBackwardRequest=false;
  }
  else{
    directionBackwardRequest=true;
  }
  
  // switch to change motor throttle direction
  if( ((directionState==2) && (directionBackwardRequest==false)) || ((directionState==0)&&(directionBackwardRequest==false)) ){
    // switching to forward mode when switch goes from bwd to fwd
    // or during startup, when the state has not been set yet and the switch is set to fwd
    Hnegative_OFF;
    Lpositive_OFF;
    delay(1); // safety waiting time not to cause half-bridge short-circuits
    Hpositive_ON;
    Lnegative_ON;
    directionState = 1; // update state
  }
  else if( (directionState==1) && (directionBackwardRequest==true) || ((directionState==0)&&(directionBackwardRequest==true)) ){
    // switching to backward mode when switch goes from fwd to bwd
    // or during startup, when the state has not been set yet and the switch is set to bwd
    Hpositive_OFF;
    Lnegative_OFF;
    delay(1); // safety waiting time not to cause half-bridge short-circuits
    Hnegative_ON;
    Lpositive_ON;
    directionState = 2; // update state
  }
  
  // make sure the motorThrottlePulseSignal is clipped beyond  [ppmSig_bottomVal, ppmSig_topVal]
  ppmSig_min = max(motorThrottlePulseWidth_micros,ppmSig_bottomVal);
  ppmSig_min_max = min(ppmSig_min,ppmSig_topVal);
  // scale motorThrottlePulseSignal down to 0..255
  OCR1A = (int)((float)(ppmSig_min_max-ppmSig_bottomVal)*1.02f/4.0f); // divide by 2 such that [1000..1510] is mapped onto [0..255]
  OCR1B = OCR1A; // forward and backward have the same duty cycle (distinguished by forward-backward-bit)
  
  Serial.print("Fwd/Bwd-Switch: ");
  Serial.print(directionBackwardRequest,DEC);
  Serial.print(", Fwd/Bwd-state: ");
  Serial.print(directionState,DEC);
  Serial.print(",  motorThrottle: ");
  Serial.print(motorThrottlePulseWidth_micros,DEC);  
  Serial.print(",  revDirPulse: ");
  Serial.print(revDirPulseWidth_micros,DEC);
  Serial.print(",  OCR1A: ");
  Serial.println(OCR1A,DEC);
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
