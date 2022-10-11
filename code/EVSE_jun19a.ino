#include "arduino_secrets.h"
#include "thingProperties.h"
#include <Timer5.h>
#include <LiquidCrystal.h>
LiquidCrystal lcd(5, 4, 3, 2, 1, 0);

volatile int inputSignal; // from oscilloscope into ADC
float samplingRate = 10000.0;
volatile boolean Int_t5_flag = false;
float currentY = 0.0;
float previousY = 0.0;
float alpha = 0.03045902796; // for cutoff of 50Hz

// Frequency calculation declaration
float zeroCrossRef = 187.65; // Setting zero crossing reference at half of 90% of the max y which is 0.9*418.5/2
float numZeroCross = 150; // 3 sec / 200 ms = 150; 200 ms is 50 Hz
float zeroCross = 0.0;
int count;
float freq = 0.0;
// Number of interrupts in 1 sec = 10917
// Time period of interrupt = 1/10917
float interruptTimePeriod = 9.151642719e-05;    // Used to calculate the frequency of input signal

// RMS votlage calculation declaration
float vSquare = 0.0;
float vRms = 0.0;
float rmsCounter = 0.0;
float nSamples = 0; // number of samples for RMS value calculation

// LED pin declarations to indicate changing frequency
const int redLedPin = A4;         // Indicates freq > 50.1 Hz
const int greenLedPin = A5;       // Indicates normal reserve where 49.9 Hz < freq < 50.1 Hz
const int yellowLedPin = A6;        // Indicates freq < 49.9 Hz

// Droop control to calculate the current from the grid frequency
float kDroop = 0.0027;
float f0 = 49.88;

// PWM duty cycle calculation and charging current
float i0_1 = -0.1305;         // i0 for slope 1 from 0 - 52 A
float i0_2 = -160;            // i0 for slope 2 from 52.5 - 80 A
float kDroop1 = 0.6133;       // Droop constant for slope 1
float kDroop2 = 2.5;          // Droop constant for slope 2


// Boosts the ADC sampling rate
void AdcBooster()
{
  ADC->CTRLA.bit.ENABLE = 0;                     // Disable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );        // Wait for synchronization
  ADC->CTRLB.reg = ADC_CTRLB_PRESCALER_DIV16 |   // Divide Clock by 64.
                   ADC_CTRLB_RESSEL_10BIT;       // Result on 12 bits
  ADC->AVGCTRL.reg = ADC_AVGCTRL_SAMPLENUM_1 |   // 1 sample
                     ADC_AVGCTRL_ADJRES(0x00ul); // Adjusting result by 0
  ADC->SAMPCTRL.reg = 0x00;                      // Sampling Time Length = 0
  ADC->CTRLA.bit.ENABLE = 1;                     // Enable ADC
  while( ADC->STATUS.bit.SYNCBUSY == 1 );        // Wait for synchronization
}

void initializePWM() {
  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(8) |          // Divide the 8MHz clock source by divisor 1: 8MHz/8 = 1MHz
                    GCLK_GENDIV_ID(4);            // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
                     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_OSC8M |   // Set the 8MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Enable the port multiplexer for the digital pin D7 
  PORT->Group[g_APinDescription[7].ulPort].PINCFG[g_APinDescription[7].ulPin].bit.PMUXEN = 1;
  
  // Connect the TCC0 timer to digital output D7 - port pins are paired odd PMUO and even PMUXE
  // F & E specify the timers: TCC0, TCC1 and TCC2
  PORT->Group[g_APinDescription[6].ulPort].PMUX[g_APinDescription[6].ulPin >> 1].reg = PORT_PMUX_PMUXO_F;

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1;   // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Dual slope PWM operation: timers countinuously count up to PER register value then down 0
  REG_TCC0_WAVE |= TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
                    TCC_WAVE_WAVEGEN_DSBOTH;    // Setup dual slope PWM on TCC0
  while (TCC0->SYNCBUSY.bit.WAVE);               // Wait for synchronization

  // Each timer counts up to a maximum or TOP value set by the PER register,
  // this determines the frequency of the PWM operation: 
  REG_TCC0_PER = 500;         // Set the frequency of the PWM on TCC0 to 1kHz -- 1 MHz / (2*1 kHz*Prescalar) = 500
  while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
  
  // Set the PWM signal to output 50% duty cycle
  REG_TCC0_CC3 = 0.5*500;         // TCC0 CC3 - on D7 -- half of 500 = 250
  while (TCC0->SYNCBUSY.bit.CC3);                // Wait for synchronization
  
  // Divide the 1 MHz signal by 1 giving 1 MHz (1 us) TCC0 timer tick and enable the outputs
  REG_TCC0_CTRLA |= TCC_CTRLA_PRESCALER_DIV1 |    // Divide GCLK4 by 1
                    TCC_CTRLA_ENABLE;             // Enable the TCC0 output
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
}

void setup() {
  // Initialize serial and wait for port to open:
  Serial.begin(9600);
  // This delay gives the chance to wait for a Serial Monitor without blocking if none is found
//  delay(1500); 
  
  // Defined in thingProperties.h
  initProperties();

  // Connect to Arduino IoT Cloud
  ArduinoCloud.begin(ArduinoIoTPreferredConnection);
  
  setDebugMessageLevel(2);
  ArduinoCloud.printDebugInfo();

  AdcBooster();
  initializePWM();
 
  pinMode(A1, INPUT);
  pinMode(A0, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  pinMode(greenLedPin, OUTPUT);
  pinMode(yellowLedPin, OUTPUT);

  lcd.begin(16, 2);
  lcd.clear();      // Clear the lcd so that lines from previously written code dont stick arround if the new string is shorter. 
  lcd.setCursor(0, 0);
  lcd.print("EVSE Project");
  lcd.setCursor(0, 1);
  lcd.print("Response Signal");
  
  analogWriteResolution(10);    // DAC resolution
  MyTimer5.begin(samplingRate);  //How often interrupt occurs
  MyTimer5.attachInterrupt(timer5_IRQ);
  ArduinoCloud.update();
}

void loop() {
  detectZeroCross();

  // Glowing led for a given frequency range
  if (freq < 49.8) {
    digitalWrite(yellowLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
    digitalWrite(greenLedPin, LOW);
  }
  else if (freq > 50.1) {
    digitalWrite(redLedPin, HIGH);
    digitalWrite(greenLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
  }
  else if (freq >= 49.8 && freq <= 50.1) {
    digitalWrite(greenLedPin, HIGH);
    digitalWrite(redLedPin, LOW);
    digitalWrite(yellowLedPin, LOW);
  }

  // RMS voltage
  if (rmsCounter < samplingRate) {
    vSquare = vSquare + sq((inputSignal*3.3/1023.0)-0.8);        // Division by 1024 to scale properly, multiply by 3.3 V and subtract the offset
    rmsCounter++;
  }
  else if (rmsCounter >= samplingRate) {
    vRms = sqrt(vSquare/rmsCounter);
    if (remoteControl == false) {
      selectChargingCurrent = calculateCurrent();
      setPWMDutyCycle(selectChargingCurrent);
    }
    ArduinoCloud.update();
    displayOutputs();
    rmsCounter = 0.0;
    vSquare = 0.0;
  }
  
}

// Interrupt 
void timer5_IRQ(){
  inputSignal = analogRead(A1);
  currentY = filterSignal(inputSignal);

  // Checks whether the current y is above and the previous y
  // is below the defined zeroCrossRef
  if(currentY > zeroCrossRef && previousY < zeroCrossRef){
    zeroCross++;
  }
  
  previousY = currentY;
  count++;
  Int_t5_flag = true;
}

// Low pass filter
float filterSignal(int inputSignal) {
  if (Int_t5_flag == true) {
    float y = alpha*inputSignal+((1-alpha)*previousY);
    Int_t5_flag = false;
    return y;
  }
}

// Zero cross detection
void detectZeroCross(){
    if (zeroCross >= numZeroCross) {
      freq = numZeroCross/(count * interruptTimePeriod);
      count = 0;
      zeroCross = 0;
    }
}

// Calculate current
float calculateCurrent() {
  float i = 0.0;
  if (freq > 49.88 && freq < 50.15){
    i = (freq - f0)/kDroop;
  }
  else if (freq <= 49.88) {
    i = 0.1;                      // Setting a current value less than 6 A (not according to standard)
  }
  else if (freq >= 50.15) {
    i = 80;                       // Max current (according to standard)
  }
  return i;
//  Serial.print("Current: ");
//  Serial.println(i);
}

// Set PWM duty cycle
void setPWMDutyCycle(float i) {
  float dutyCycleMultiplier = 0.0; 
  if (i >= 6 && i < 52) {
    dutyCycleMultiplier = (i-i0_1)/(100*kDroop1);  // Dividing by 100 to convert to percentage
  }
  else if (i > 52.5) {
    dutyCycleMultiplier = (i-i0_2)/(100*kDroop2); // Dividing by 100 to convert to percentage
  }
  else if (i >= 52 && i <= 52.5) {
    dutyCycleMultiplier = 0.85;
  }
  else if (i < 6) {
    dutyCycleMultiplier = 0.0;
  }

//  Serial.print("Duty cycle: ");
//  Serial.println(dutyCycleMultiplier);

  // Set the PWM signal to output the calculated duty cycle
  REG_TCC0_CC3 = (uint16_t) (dutyCycleMultiplier*500);
  while (TCC0->SYNCBUSY.bit.CC3);                // Wait for synchronization

  // Display duty cycle on IoT cloud
  displayPWMDutyCycle = dutyCycleMultiplier*100;
}

// Called when remote control on IoT dashboard is toggled
void onRemoteControlChange() {
  // Do something
  if (remoteControl) {
    onSelectChargingCurrentChange();
  }
  else {
    selectChargingCurrent = calculateCurrent();
    setPWMDutyCycle(selectChargingCurrent);
//    Serial.print("Droop charging current is: ");
//    Serial.println(selectChargingCurrent);
  }
}

// Reads and displays the current on the IoT dashboard
void onSelectChargingCurrentChange() {
  // Do something
  if (remoteControl == true) {
    setPWMDutyCycle(selectChargingCurrent);
//    Serial.print("The selected charging current is: ");
//    Serial.println(selectChargingCurrent);
  }
}

// Displays frequency and RMS voltage on LCD and IoT cloud
void displayOutputs() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Freq: ");lcd.print(freq); lcd.print(" Hz");
  lcd.setCursor(0, 1);
  lcd.print("Vrms: "); lcd.print(vRms); lcd.print(" V");
  displayVrms = vRms;
  displayFreq = freq;
//  Serial.print("Frequency is: ");
//  Serial.println(displayFreq);
//  Serial.print("RMS voltage: ");
//  Serial.println(displayVrms);
}
