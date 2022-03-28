//ABN

// Code Part 1
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
#include <Adafruit_SI5351.h>
Adafruit_SI5351 clockgen = Adafruit_SI5351();


/*
  ad5933-test
    Reads impedance values from the AD5933 over I2C and prints them serially.
*/

#include <Wire.h>
#include "AD5933.h"

#define START_FREQ  (5000)
#define FREQ_INCR   (5000)
#define NUM_INCR    (1)  //ABN
#define REF_RESIST  (100000)//10000 ABN 806 505e3

double *gain;//ABN
int phase[NUM_INCR + 1];

void setup(void)
{
  // Begin I2C
  Wire.begin();
  lcd.begin();
  lcd.backlight();
  // Begin serial at 9600 baud for output
  Serial.begin(9600);
  Serial1.begin(9600);           //serial communication
  lcd.println("AD5933 Test Started!");
  lcd.clear();

  gain = new double [NUM_INCR + 1];


//  if (clockgen.begin() != ERROR_NONE)
//  {
//    /* There was a problem detecting the IC ... check your connections */
//    Serial.print("Ooops, no Si5351 detected ... Check your wiring or I2C ADDR!");
//    Serial.print("Ooops, no Si5351 detected ... Check your wiring or I2C ADDR!");
//    while (1);
//  }

  /* Uncomment the next line to speed up the I2C communication */
//  Wire.setClock(400000);

  Serial.println("OK!");
  lcd.println("OK!");
  delay(1000);
  /* INTEGER ONLY MODE --> most accurate output */
  /* Setup PLLA to integer only mode @ 900MHz (must be 600..900MHz) */
  /* Set Multisynth 0 to 112.5MHz using integer only mode (div by 4/6/8) */
  /* 25MHz * 36 = 900 MHz, then 900 MHz / 8 = 112.5 MHz */
//  Serial.println("Set PLLA to 900MHz");
//  lcd.println("Set PLLA to 900MHz");
//  clockgen.setupPLLInt(SI5351_PLL_A, 36); //The cleanest way to run the PLL is to do a straight up integer multiplication 15<m<35
//  Serial.println("Set Output #0 to 112.5MHz");
//  lcd.println("Set Output #0 to 112.5MHz");
//  clockgen.setupMultisynthInt(0, SI5351_PLL_A, SI5351_MULTISYNTH_DIV_8); 

  /* FRACTIONAL MODE --> More flexible but introduce clock jitter */
  /* Setup PLLB to fractional mode @616.66667MHz (XTAL * 24 + 2/3) */
  /* Setup Multisynth 1 to 13.55311MHz (PLLB/45.5) */
  //  clockgen.setupPLLInt(SI5351_PLL_B, 23);
  //  Serial.println("Set Output #1 to 13.553115MHz");
  //  lcd.println("Set Output #1 to 13.553115MHz");
  //  clockgen.setupMultisynth(1, SI5351_PLL_B, 45, 1, 2);

  /* Multisynth 2 is not yet used and won't be enabled, but can be */
  /* Use PLLB @ 616.66667MHz, then divide by 900 -> 685.185 KHz */
  /* then divide by 64 for 10.706 KHz */
  /* configured using either PLL in either integer or fractional mode */

  //  lcd.clear();
  //  Serial.println("Set Output #2 to 10.706 KHz");
  //  lcd.println("Set Output #2 to 10.706 KHz");
  //  clockgen.setupMultisynth(2, SI5351_PLL_B, 900, 0, 1);
  //  clockgen.setupRdiv(2, SI5351_R_DIV_64); 

  /* Enable the clocks */
  //clockgen.enableOutputs(true);



  // Perform initial configuration. Fail if any one of these fail.
  if (!(AD5933::reset() &&
        AD5933::setInternalClock(true) &&
        AD5933::setStartFrequency(START_FREQ) &&
        AD5933::setIncrementFrequency(FREQ_INCR) &&
        AD5933::setNumberIncrements(NUM_INCR) &&
        AD5933::setPGAGain(PGA_GAIN_X1)))
  {
    delay(1000);
    lcd.println("FAILED in initialization!");
    Serial.println("FAILED in initialization!");
    while (true) ;
  }
  AD5933::setSettlingCycles(200);
  // Perform calibration sweep
  if (AD5933::calibrate(gain, phase, REF_RESIST, NUM_INCR + 1)) {
    Serial.println("Calibrated!");
    lcd.println("Calibrated!");
    delay(1000);
    lcd.clear();
    int cfreq = START_FREQ / 1000;              
    for (int i = 0; i < NUM_INCR + 1; i++, cfreq = FREQ_INCR / 1000) {
      // Print raw frequency data
      Serial.print(cfreq);
      Serial.print(": gain=");
      Serial.print(gain[i], 15);
    }
  }
  else {
    Serial.println("Calibration failed...");
    lcd.println("Calibration failed...");
    delay(1000);
    lcd.clear();

  }

}


void loop(void)//ABN (viod)
{
  // Easy to use method for frequency sweep
  delay(5000);
  //frequencySweepEasy();

  //Delay
  // Complex but more robust method for frequency sweep
  frequencySweepRaw();
 
  delay(60000);
  // Delay

}

// Easy way to do a frequency sweep. Does an entire frequency sweep at once and
// stores the data into arrays for processing afterwards. This is easy-to-use,
// but doesn't allow you to process data in real time.
void frequencySweepEasy() {
  // Create arrays to hold the data
  int real[NUM_INCR + 1], imag[NUM_INCR + 1];

  // Perform the frequency sweep                                  
  if (AD5933::frequencySweep(real, imag, NUM_INCR + 1)) {
    // Print the frequency data
    int cfreq = START_FREQ / 1000;
    for (int i = 0; i < NUM_INCR + 1; i++, cfreq += FREQ_INCR / 1000) {
      // Print raw frequency data
      Serial.print(cfreq);
      Serial.print(": R=");
      Serial.print(real[i]);
      Serial.print("/I=");
      Serial.print(imag[i]);
      delay(500);
      lcd.clear();
      lcd.println(cfreq);
      delay(500);
      lcd.println(": R=");
      delay(500);
      lcd.print(real[i]);
      delay(500);
      lcd.println();
      lcd.println("I=");
      delay(500);
      lcd.print(imag[i]);
      delay(500);

      // Compute impedance
      double magnitude = (double) sqrt(pow((double)real[i], 2) + pow((double)imag[i], 2));
      double impedance = 1.0 / (magnitude * gain[i]);
      Serial.print("   |Z|=");
      Serial.println(impedance);
      lcd.clear();
      lcd.println("   |Z|=");
      lcd.println(impedance);
    }
    Serial.println("Frequency sweep complete!");
    lcd.println("   Frequency sweep complete!");
    delay(5000);
  } else {
    Serial.println("Frequency sweep failed...");
    lcd.println("Frequency sweep failed...");
  }
}




// Removes the frequencySweep abstraction from above. This saves memory and
// allows for data to be processed in real time. However, it's more complex.
void frequencySweepRaw() {
  // Create variables to hold the impedance data and track frequency
  int real, imag, i = 0, cfreq = START_FREQ / 1000;//ABN

  // Initialize the frequency sweep
  if (!(AD5933::setPowerMode(POWER_STANDBY) &&          // place in standby
        AD5933::setControlMode(CTRL_INIT_START_FREQ) && // init start freq
        AD5933::setControlMode(CTRL_START_FREQ_SWEEP) &&
        AD5933::setExcitationVoltage(CTRL_1000mV_P_P ))) // begin frequency sweep
  {
    Serial.println("Could not initialize frequency sweep...");
    lcd.println("Could not initialize frequency sweep...");
  }

  // Perform the actual sweep                                                             
  while ((AD5933::readStatusRegister() & STATUS_SWEEP_DONE) != STATUS_SWEEP_DONE) {
    // Get the frequency data for this frequency point
    if (!AD5933::getComplexData(&real, &imag)) {
      Serial.println("Could not get raw frequency data...");
      lcd.println("Could not get raw frequency data...");
    }

    // Print out the frequency data
    Serial.print(cfreq);
    Serial.print(": R=");
    Serial.print(real);
    Serial.print("/I=");
    Serial.print(imag);
    lcd.clear();
    delay(250);
    lcd.print(cfreq);
    lcd.println(": R=");
    lcd.print(real);
    delay(250);
    lcd.println();
    lcd.println("I=");
    lcd.print(imag);
    delay(250);


    // Compute impedance
    double magnitude = sqrt(pow((double)real, 2) + pow((double)imag, 2));
    double impedance = (1.0 / (magnitude * gain[i]));

    Serial.print("  |Z|=");
    Serial.println(impedance, 10);
    delay(250);
    lcd.println("    |Z|=");
    lcd.println(impedance); // impedance

    int j=5;
    if(cfreq==j){
    char Mymessage[10];
    // start to write data serially
    dtostrf(impedance, 10 , 2 , Mymessage);
    if (Serial1.available()) {
      Serial1.write(Mymessage); //Write the serial data
      delay(1000);}
    }
    
    delay(250);
    // Increment the frequency
    i++;
    cfreq += FREQ_INCR / 1000;
    AD5933::setControlMode(CTRL_INCREMENT_FREQ);


  }

  Serial.println("Frequency sweep complete!");

  lcd.println("       Frequency sweep  complete!");

  // Set AD5933 power mode to standby when finished
  if (!AD5933::setPowerMode(POWER_STANDBY)) {
    lcd.println("Could not set to standby...");
    Serial.println("Could not set to standby...");
  }

}
