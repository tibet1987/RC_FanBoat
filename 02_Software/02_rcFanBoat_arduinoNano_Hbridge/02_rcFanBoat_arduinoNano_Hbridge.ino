
// variables for PPM signal read-in
int ppmSigLastPosEdge_micros = 0;
int ppmSignalPulseWidth_micros= 0;
const byte interruptPin = 2;
int ppmSig_lowVal = 996;
int ppmSig_upMinLowVal = 1996;
long ppmSig_min1000 = 0;
long ppmSig_max2000 = 0;
boolean forwardDirectionState = true;
boolean directionForwardRequest = true;

// signal for PWM generation
int sensorValue = 0; // test-signal from poti
int fanPWMSig_fromPPMSig = 0;
long cnt = 0;

#define Hpositive_ON TCCR1A |= (1 << COM1A1)
#define Hnegative_ON TCCR1A |= (1 << COM1B1)
#define Hpositive_OFF TCCR1A &= ~(1 << COM1A1)
#define Hnegative_OFF TCCR1A &= ~(1 << COM1B1)
#define Lpositive_ON PORTB |= (1 << PORTB0)
#define Lpositive_OFF PORTB &= ~(1 << PORTB0)
#define Lnegative_ON PORTD |= (1 << PORTD7)
#define Lnegative_OFF PORTD &= ~(1 << PORTD7)

void setup() {
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin),PPMchangeDetected,CHANGE);

  // setup PWM 
  TCCR1A = 0; // reset all registers
  DDRB = (1 << DDB1);      // PB1 is now an output for OC1A -> H+  (D9)
  DDRB |= (1 << DDB2);      // PB2 is now an output for OC1B -> H-  (D10)
  DDRB |= (1 << DDB0);      // PB0 is now an output -> L-  (D8)
  DDRD = (1 << DDD);      // PD7 is now an output -> L+  (D7)

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
  // A = positive
  // B = negative

  if( (forwardDirectionState==true) && (directionForwardRequest==false) ){
    // the following sequence has to stay the same!
    Hpositive_OFF;
    delay(1); // safety waiting time
    Lnegative_OFF;
    Lpositive_ON;
    Hnegative_ON;
    forwardDirectionState = directionForwardRequest;
  }
  elseif( (forwardDirectionState==false) && (directionForwardRequest==true) ){
    Hpositive_OFF;
    Hnegative_OFF;
    Lpositive_OFF;
    Lnegative_OFF;
    forwardDirectionState = directionForwardRequest;
  }
  
  // make sure the PPM pulse duration has its lowest value at "ppmSig_lowVal" and has a maximum value add of 1000µs
  ppmSig_min1000 = (long)min(max(ppmSignalPulseWidth_micros-ppmSig_lowVal,0),1000);
  // now make sure that the PPM pulse is not bigger than "ppmSig_upMinLowVal"
  ppmSig_max2000 = min(ppmSig_min1000+(ppmSig_upMinLowVal-ppmSig_lowVal),(long)2000);

  // scale ppmSig_min1000 range of 0..1000 down to 0..255
  //ppmSig_max2000 = (ppmSig_min1000 *(long)255) / (long)1000;
  OCR1A = min( (uint8_t)( (ppmSig_min1000 *(long)255) / (long)1000), 255);
  Serial.print(ppmSignalPulseWidth_micros,DEC);
  Serial.print(", ");
  Serial.print(ppmSig_min1000,DEC);
  Serial.print(", ");
  Serial.println(test,DEC);
}

void PPMchangeDetected(){
  if(digitalRead(2)==LOW){
    ppmSignalPulseWidth_micros = micros() - ppmSigLastPosEdge_micros;
  }
  else{
    ppmSigLastPosEdge_micros = micros();
  }
}

