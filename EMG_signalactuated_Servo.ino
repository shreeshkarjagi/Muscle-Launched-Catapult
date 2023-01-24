#include <Servo.h>

//Threshold for servo motor control with muscle sensor. 
//You can set a threshold according to the maximum and minimum values of the muscle sensor.
#define THRESHOLD 800

//Pin number where the sensor is connected. (Analog 0)
#define EMG_PIN 0

//Pin number where the servo motor is connected. (Digital PWM 9)
#define SERVO_PIN 9

//Define Servo motor
Servo SERVO_1;


// const int EMG_PIN = A0;  // Myoware 2.0 sensor is connected to analog pin A0
float Q = 0.1;  // Process noise covariance
float R = 0.1;  // Measurement noise covariance
float P = 1;  // Estimate error covariance
float K;  // Kalman gain
float value = 0;  // Estimated value
float previousX = 0;  // Previous estimated value
float sensorValue;  // Raw sensor value

/*-------------------------------- void setup ------------------------------------------------*/
int pos = 0;    // variable to store the servo position
bool prevContractedState = false;
bool open = false;

void setup(){
  
  //BAUDRATE set to 115200, remember it to set monitor serial properly. 
  //Used this Baud Rate and "NL&CR" option to visualize the values correctly.
  Serial.begin(115200);
  
  //Set servo motor to digital pin 9
  SERVO_1.attach(SERVO_PIN);
  SERVO_1.write(0);
}

/*--------------------------------  void loop  ------------------------------------------------*/

void loop(){

  
  //Filter Raw sensor value
  sensorValue = analogRead(EMG_PIN);  // Read the raw sensor value
  previousX = value;  // Update the previous estimate
  P = P + Q;  // Update the estimate error covariance
  K = P / (P + R);  // Calculate the Kalman gain
  value = previousX + K * (sensorValue - previousX);  // Calculate the new estimate
  P = (1 - K) * P;  // Update the estimate error covariance
  Serial.println(value);  // Debugging: print the filtered value to the serial monitor
  delay(10);  // Delay for a short time to allow the serial monitor to keep up

  if(value > THRESHOLD && open == false && prevContractedState == false)
  {
    SERVO_1.write(0);
    open = true;
    prevContractedState = true;
  }
  else if(value > THRESHOLD && open == true && prevContractedState == false)
  {
    SERVO_1.write(90);
    open = false;
    prevContractedState = true;
  }
  else if(value < THRESHOLD)
  {
    prevContractedState = false;
  }

  Serial.println(value);
  delay(200);
}

