
// variables for PPM signal read-in
int ppmSigLastPosEdge_micros = 0;
int ppmSignalPulseWidth_micros= 0;
const byte interruptPin = 2;
int ppmSig_lowVal = 1005;
int ppmSig_upMinLowVal = 990;
long ppmSig_min1000 = 0;

// signal for PWM generation
int sensorValue = 0; // test-signal from poti
int fanPWMSig_fromPPMSig = 0;
long cnt = 0;

#define Hpositive_ON TCCR1A |= (1 << COM1A1)
#define Hnegative_ON TCCR1A |= (1 << COM1B1)
#define Hpositive_OFF TCCR1A &= ~(1 << COM1A1)
#define Hnegative_OFF TCCR1A &= ~(1 << COM1B1)

void setup() {
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin),PPMchangeDetected,CHANGE);

  // setup PWM 
  TCCR1A = 0; // reset all registers
  DDRB = (1 << DDB1);      // PB1 is now an output for OC1A
  DDRB |= (1 << DDB2);      // PB2 is now an output for OC1B
  OCR1A = 0x7F;            // set PWM for 50% duty cycle
  OCR1B = 0x7F;            // set PB2 PWM for 50% duty cycle
  TCCR1A |= (1 << COM1A1) | (1 << COM1B1); // Clear OC1A and OC1B on Compare Match when up-counting. 
                                           //  Set  OC1A and OC1B on Compare Match when down-counting.
  TCCR1A |= (1 << WGM10);  // set phase correct Mode (takes double the time for a PWM cycle)
  TCCR1B = (1 << CS10);    // set prescaler to 0 (32.25kHz) and start PWM
}

void loop() {
  // A = positive
  // B = negative
  
  sensorValue = analogRead(0);
  Serial.print(sensorValue,DEC);
  Serial.print(", ");
  if(sensorValue<512){
    Hpositive_ON;
    Hnegative_OFF;
    OCR1A = (uint8_t)(sensorValue>>1);
    OCR1B = 0x00;
    Serial.print(" 0 ");
    Serial.print(", ");
    Serial.print((uint8_t)OCR1A,DEC);
    Serial.print(", ");
    Serial.println((uint8_t)OCR1B,DEC);
  }
  else{ //(sensorValue>=512)
    Hpositive_OFF;
    Hnegative_ON;
    OCR1A = 0x00;
    OCR1B = (uint8_t)(sensorValue>>1) + 256;
    Serial.print(" 1 ");
    Serial.print(", ");
    Serial.print((uint8_t)OCR1A,DEC);
    Serial.print(", ");
    Serial.println((uint8_t)OCR1B,DEC);
  }

  /*
  //Serial.print(ppmSignalPulseWidth_micros,DEC);
  //Serial.print(", ");
  //Serial.println(min( (int)(((long)min(max(ppmSignalPulseWidth_micros-ppmSig_lowVal,0),1000)*(long)255)/(long)ppmSig_upMinLowVal), 255),DEC);

  //sensorValue = analogRead(0);
  //OCR1A = sensorValue>>2;
  
  // make sure the PPM pulse duration has its lowest value at "ppmSig_lowVal" and has a maximum value add of 1000µs
  ppmSig_min1000 = (long)min(max(ppmSignalPulseWidth_micros-ppmSig_lowVal,0),1000);
  // no7w make sure that the PPM pulse is not bigger than "ppmSig_upMinLowVal"
  OCR1A = min( (int)( (ppmSig_min1000 *(long)255) / (long)ppmSig_upMinLowVal), 255);
  */
}

void PPMchangeDetected(){
  if(digitalRead(2)==LOW){
    ppmSignalPulseWidth_micros = micros() - ppmSigLastPosEdge_micros;
  }
  else{
    ppmSigLastPosEdge_micros = micros();
  }
}

