#include <Wire.h>
#include "AD5933.h"

#define START_FREQ  (80000)
#define FREQ_INCR   (1000)
#define NUM_INCR    (1)
#define REF_RESIST  (22000)

double gain[NUM_INCR+1];
int phase[NUM_INCR+1];
float sum = 0;

void setup(void)
{
  // Begin I2C
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  // Begin serial at 9600 baud for output
  Serial.begin(9600);
  Serial.println("A0 and PmodIA Test Started!");

  // Perform initial configuration. Fail if any one of these fail.
  if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
        {
            Serial.println("FAILED in initialization!");
            while (true) ;
        }
  digitalWrite(LED_BUILTIN, HIGH);
  // Perform calibration sweep
  if (AD5933::calibrate(gain, phase, REF_RESIST, NUM_INCR+1)){
    Serial.println("Calibrated!");
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(5000);
    digitalWrite(LED_BUILTIN, LOW);
  }
  else
    Serial.println("Calibration failed...");
} //end setup

void loop(void)
{
  sum = 0;
  for(int i =0; i < 100; i++){
      sum += analogRead(A0);
      delay(10);
    }
    
  Serial.print(1024.0/ (sum/100));
  Serial.print(":");
  
  int real, imag, i = 0, cfreq = START_FREQ/1000;

    // Initialize the frequency sweep
    if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // place in standby
          AD5933::setControlMode(CTRL_INIT_START_FREQ) && // init start freq
          AD5933::setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
         {
             Serial.println("Could not initialize frequency sweep...");
         }
    if (AD5933::readStatusRegister() && AD5933::getComplexData(&real, &imag)){
      double magnitude = sqrt(pow(real, 2) + pow(imag, 2));
        double impedance = 1/(magnitude*gain[i]);
        Serial.println(impedance);
    }
   
    // Set AD5933 power mode to standby when finished
    if (!AD5933::setPowerMode(POWER_STANDBY)){ Serial.println("Could not set to standby...");}
  // Delay
  delay(60000);
}

