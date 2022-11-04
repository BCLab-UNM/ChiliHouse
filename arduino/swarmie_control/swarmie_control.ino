//////////////////////////
////Required Libraries////
//////////////////////////

//Built-in Arduino libraries
#include <Wire.h>

//Custom libraries located in arduino directory
#include <L3G.h>
#include <LPS.h>
#include <LSM303.h>
#include <Movement.h>
#include <NewPing.h>
#include <Odometry.h>
#include <Servo.h>

// Constants
// #define PI 3.14159265358979323846
#define RAD2DEG(radianAngle) (radianAngle * 180.0 / PI)
#define DEG2RAD(degreeAngle) (degreeAngle * PI / 180.0)

////////////////
////Settings////
////////////////

//Gripper (HS-485HB Servo)
byte pumpPin0 = 9;
byte pumpPin1 = 12;

//Movement (VNH5019 Motor Driver Carrier)
byte rightDirectionA = A3; //"clockwise" input
byte rightDirectionB = A2; //"counterclockwise" input
byte rightSpeedPin = 11; //PWM input
byte leftDirectionA = A5; //"clockwise" input
byte leftDirectionB = A4; //"counterclockwise" input
byte leftSpeedPin = 10; //PWM input

//Odometry (8400 CPR Encoder)
byte rightEncoderA = 7;
byte rightEncoderB = 8;
byte leftEncoderA = 0;
byte leftEncoderB = 1;

//Serial (USB <--> Intel NUC)
String rxBuffer;
unsigned long watchdogTimer = 1000; //fail-safe in case of communication link failure (in ms)
unsigned long lastCommTime = 0; //time of last communication from NUC (in ms)

//Ultrasound (Ping))))
byte leftSignal = 4;
byte centerSignal = 5;
byte rightSignal = 6;


////////////////////////////
////Class Instantiations////
////////////////////////////

L3G gyroscope;
LSM303 magnetometer_accelerometer;
LPS pressure;
Movement move = Movement(rightSpeedPin, rightDirectionA, rightDirectionB, leftSpeedPin, leftDirectionA, leftDirectionB);
Odometry odom = Odometry(rightEncoderA, rightEncoderB, leftEncoderA, leftEncoderB);
NewPing leftUS(leftSignal, leftSignal, 330);
NewPing centerUS(centerSignal, centerSignal, 330);
NewPing rightUS(rightSignal, rightSignal, 330);


/////////////
////Setup////
/////////////

void setup()
{
  Serial.begin(115200);
  while (!Serial) {} //wait for Serial to complete initialization before moving on

  Wire.begin();

  if (imuStatus()) {
    imuInit();
  }
  pinMode(pumpPin0, OUTPUT);
  pinMode(pumpPin1, OUTPUT);
  rxBuffer = "";
}


/////////////////
////Main Loop////
/////////////////

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == ',' || c == '\n') {
      parse();
      rxBuffer = "";
      lastCommTime = millis();
    }
    else if (c > 0) {
      rxBuffer += c;
    }
  }
  if (millis() - lastCommTime > watchdogTimer) {
    move.stop();
  }
}


////////////////////////
//Parse receive buffer//
////////////////////////

void parse() {
  if (rxBuffer == "v") {
    int speedL = Serial.parseInt();
    int speedR = Serial.parseInt();
    
    if (speedL >= 0 && speedR >= 0) {
      move.forward(speedL, speedR);
    }
    else if (speedL <= 0 && speedR <= 0) {
      move.backward(speedL*-1, speedR*-1);
    }
    else if (speedL <= 0 && speedR >= 0) {
      move.rotateLeft(speedL*-1, speedR);
    }
    else {
      move.rotateRight(speedL, speedR*-1);
    }
  }
  else if (rxBuffer == "s") {
    move.stop();
  }
  else if (rxBuffer == "d") {
    static int ping_state = 0;
    static unsigned long leftUSValue = 300;
    static unsigned long rightUSValue = 300;
    static unsigned long centerUSValue = 300;

    if (imuStatus()) {
      imuInit();
      Serial.println(updateIMU());
    }

    Serial.println("ODOM,1," + updateOdom());

    // Only do one sonar at a time to prevent crosstalk.
    if (ping_state == 0) {
      leftUSValue = leftUS.ping_median(3);
      if(leftUSValue != NO_ECHO)
	Serial.println("USL,1," + String(NewPing::convert_cm(leftUSValue)));
    }
    else if (ping_state == 1) {
      rightUSValue = rightUS.ping_median(3);
      if(rightUSValue != NO_ECHO)
	Serial.println("USR,1," + String(NewPing::convert_cm(rightUSValue)));
    }
    else{
      centerUSValue = centerUS.ping_median(3);
      if(centerUSValue != NO_ECHO)
	Serial.println("USC,1," + String(NewPing::convert_cm(centerUSValue)));
    }
    ping_state = (ping_state + 1) % 3;
  }

  else if (rxBuffer == "f") {
    bool pump_forward = Serial.parseInt();
    if (pump_forward){
      digitalWrite(pumpPin1, HIGH);
      digitalWrite(pumpPin0, LOW);
    } else{
      digitalWrite(pumpPin1, LOW);
    }
  }
  else if (rxBuffer == "b") {
    bool pump_backward = Serial.parseInt();
    if (pump_backward) {
      digitalWrite(pumpPin0, HIGH);
      digitalWrite(pumpPin1, LOW);
    } else{
      digitalWrite(pumpPin0, LOW);
    }
  }
}


//////////////////////////
//Update transmit buffer//
//////////////////////////

String updateIMU() {  
  //Update current sensor values
  gyroscope.read();
  magnetometer_accelerometer.read();

  if (!gyroscope.timeoutOccurred() && !magnetometer_accelerometer.timeoutOccurred()) {
    //Collect updated values
    LSM303::vector<int16_t> acc = magnetometer_accelerometer.a;
    L3G::vector<int16_t> gyro = gyroscope.g;
    LSM303::vector<int16_t> mag = magnetometer_accelerometer.m;

    //Append data to buffer
    String txBuffer = String("IMU,1,") +
               String(acc.x) + "," +
               String(acc.y) + "," +
               String(acc.z) + "," +
               String(mag.x) + "," +
               String(mag.y) + "," +
               String(mag.z) + "," +
               String(gyro.x) + "," +
               String(gyro.y) + "," +
               String(gyro.z);

    return txBuffer;
  }

  return "";
}

String updateOdom() {
  String txBuffer;
  odom.update();

  txBuffer = String(odom.left) + "," +
             String(odom.right) + "," +
             String(odom.clock);

  return txBuffer;
}


////////////////////////////
////Initializer Functions///
////////////////////////////

//Initialize gyroscope, accelerometer, magnetometer, and pressure gauge
void imuInit() {
  gyroscope.init();
  gyroscope.enableDefault();
  gyroscope.setTimeout(1);

  magnetometer_accelerometer.init();
  magnetometer_accelerometer.enableDefault();

  magnetometer_accelerometer.setTimeout(1);

  pressure.init();
  pressure.enableDefault();
}


////////////////////////////
////Diagnostic Functions////
////////////////////////////

//Check for valid I2C connection to verify IMU
bool imuStatus() {
  byte numberOfDevices = 0;

  for(byte address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();

    if (!error) {
      numberOfDevices++;
    }
  }

  if (numberOfDevices > 0) {
    return true;
  }
  else {
    return false;
  }
}
