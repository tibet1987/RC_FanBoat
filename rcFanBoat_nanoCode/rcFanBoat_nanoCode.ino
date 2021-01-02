
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


void setup() {
  Serial.begin(9600);
  pinMode(interruptPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(interruptPin),PPMchangeDetected,CHANGE);

  // setup PWM 
  TCCR1A = 0; // reset all registers
  DDRB = (1 << DDB1);      // PB1 is now an output for OC1A
  OCR1A = 0x7F;            // set PWM for 50% duty cycle
  TCCR1A |= (1 << COM1A1); // Clear OC0A on Compare Match when up-counting. Set OC0A on Compare Match when down-counting.
  TCCR1A |= (1 << WGM10);  // set phase correct Mode (takes double the time for a PWM cycle)
  TCCR1B = (1 << CS10);    // set prescaler to 0 (32.25kHz) and start PWM
}

void loop() {
  Serial.print(ppmSignalPulseWidth_micros,DEC);
  Serial.print(", ");
  Serial.println(min( (int)(((long)min(max(ppmSignalPulseWidth_micros-ppmSig_lowVal,0),1000)*(long)255)/(long)ppmSig_upMinLowVal), 255),DEC);

  //sensorValue = analogRead(0);
  //OCR1A = sensorValue>>2;
  ppmSig_min1000 = (long)min(max(ppmSignalPulseWidth_micros-ppmSig_lowVal,0),1000);
  OCR1A = min( (int)( (ppmSig_min1000 *(long)255) / (long)ppmSig_upMinLowVal), 255);
}

void PPMchangeDetected(){
  if(digitalRead(2)==LOW){
    ppmSignalPulseWidth_micros = micros() - ppmSigLastPosEdge_micros;
  }
  else{
    ppmSigLastPosEdge_micros = micros();
  }
}

