
#include <Wire.h>
#include "AD5933.h"
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT11


#define START_FREQ  (5000)
#define FREQ_INCR   (5000)
#define NUM_INCR    (1)
#define REF_RESIST  (22000)
DHT dht(DHTPIN, DHTTYPE);

double gain[NUM_INCR+1];
int phase[NUM_INCR+1];
float sum = 0;

unsigned long lastSweep = 0;
int currentSweep = 30000; //1800000;

AD5933 sensor;

void setup(void)
{
  // Begin I2C
  Wire.begin();
  pinMode(LED_BUILTIN, OUTPUT);
  // Begin serial at 9600 baud for output
  Serial.begin(9600);
  dht.begin(); // initialize the sensor
  Serial.println("DHT11, A0 and PmodIA Test Started!");

  // Perform initial configuration. Fail if any one of these fail.
  if (!(sensor.reset() && 
        sensor.setRange(4) && //sensor.CTRL_OUTPUT_RANGE_3 which is 400 mV 0b00000100 == 4
        sensor.setInternalClock(true) &&
        sensor.setStartFrequency(START_FREQ) &&
        sensor.setIncrementFrequency(FREQ_INCR) &&
        sensor.setNumberIncrements(NUM_INCR) &&
        sensor.setPGAGain(PGA_GAIN_X1)))
        {
            Serial.println("FAILED in initialization!");
            while (true) ;
        }
  digitalWrite(LED_BUILTIN, HIGH);
  // Perform calibration sweep
  if (sensor.calibrate(gain, phase, REF_RESIST, NUM_INCR+1)){
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
  else{
    Serial.println("Calibration failed...");
  }
  Serial.print("Setup: ");
  for(int i = 0; i < 1; i++){ // Do we want 0?
      Serial.print("gain:");
      Serial.print(gain[i]);
      Serial.print(", phase:");
      Serial.print(phase[i]);
    }
  Serial.print(", T:");
  Serial.print(dht.readTemperature(true));
  Serial.print(", H:");
  Serial.println(dht.readHumidity());
  Serial.println("");
} //end setup

void loop(){
  if(millis() - lastSweep >= currentSweep){
    freqSweep();
    lastSweep = millis();
  } //end last sweep check
} //end loop
  
void freqSweep(){
  int real, imag, i = 0;

    // Initialize the frequency sweep
    if (!(sensor.setPowerMode(POWER_STANDBY) &&          // place in standby
          sensor.setControlMode(CTRL_INIT_START_FREQ) && // init start freq
          sensor.setControlMode(CTRL_START_FREQ_SWEEP))) // begin frequency sweep
         {
             Serial.println("Could not initialize frequency sweep...");
         }
    if (sensor.readStatusRegister() && sensor.getComplexData(&real, &imag)){
      double magnitude = sqrt((real*real) + (imag*imag));
      double impedance = 1.0/(magnitude*gain[i]);
      Serial.print("I:");
      Serial.print(impedance);
      Serial.print(", A:");
      Serial.print(getA0());
      Serial.print(", T:");
      Serial.print(dht.readTemperature(true));
      Serial.print(", H:");
      Serial.println(dht.readHumidity());    
    } // if pmodIA getdata good
    // Serial.print(":");
    // if (sensor.readStatusRegister() && sensor.getComplexData(&real, &imag)){
    //   double magnitude = sqrt((real*real) + (imag*imag));
    //     double impedance = 1.0/(magnitude*gain[i+1]);
    //     Serial.println(impedance*2);
    // }
  // Delay
  // delay(60000);
} //end freqSweep

float getA0(){
  sum = 0;
  for(int i =0; i < 100; i++){
      sum += analogRead(A0);
      delay(10);
    } //end for accumulator
  return 1024.0/ (sum/100);
} //end getA0
