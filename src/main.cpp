/*
  Name: CDDM_Balancing_Robot_Daan_Heetkamp
  Date: 13-11-2024
  Author: Daan Heetkamp

  Description:
  The program will do the following: 
  Use a ESP32C3 Wroom. Amount of IO pins: 13
  Use two DRV8825PWP stepper motor drivers. Amount of IO pins: 4 total
  Use one wake/deepsleep switch. Amount of IO pins: 1
  Use one LSM6DSO32TR accelerometer. Amount of IO pins: 2
  Use of one Buzzer for sound. Amount of IO pins: 1
  Use of three WS2812-2020 RGB LEDs. Amount of IO pins: 1

  This would defice to the following: 
  1. The robot will balance itself using the accelerometer.
  2. The robot will be able to move forward and backward using the stepper motors.
  3. The robot will be able to wake up and go to sleep using the switch.
  4. The robot will be able to make sounds using the buzzer.
  5. The robot will be able to show status colors using the RGB LEDs.

  The use of the external controller will be implemented as well. 
  This uses a gyroscope and an ESP32C3 Wroom. Using ESP-NOW EASY the two devices will communicate with each other.
  This would defice to the following:
  1. The external controller will be able to control the robot using the gyroscope.
  Moving the controller will move the robot.
  2. The external controller will be able show the status of the robot using the OLED on the controller.

  

  Revision:
  0.1 - First drafts
  0.2 - Working program
  V3(0.3) - Working motors and gyroscope with zerobutton.
  V4(0.4) - Not working properly. The library is not importing. timerAPI is also different from the VSCode one. Arduino timer API and other one.
  V5(0.5) - Working motors and gyroscope with zerobutton. The timer is working as well. ESPNOW is working as well.
  0.6 - Cleanin up the code. Adding comments. TO-DO: PID tuning.
*/

// include libraries
#include <Arduino.h>
#include "ESPNOW-EASY.h"

// // include the MPU6050 library
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

// // include the AccelStepper library
#include <AccelStepper.h>
#include <FastLED.h>


// ----- Declare Constants -----

#define DEBUG_GYRO false
#define DEBUG_MOTOR false
#define DEBUG_ESPNOW true
#define DEBUG_SYSTEM true

#define MCU_BUTTON GPIO_NUM_9

/*************For the ESP32C3 Wroom SOM******************/
#define MCU_SCL GPIO_NUM_6
#define MCU_SDA GPIO_NUM_7

/********For the LED**********/
#define NUM_LEDS 3
#define MCU_RGB_LED GPIO_NUM_10 //3 WS2812-2020 RGB LEDs

// This is an array of leds.  One item for each led in your strip.
CRGB leds[NUM_LEDS];

/*************For the Stepper Motor******************/
#define MCU_STEPPER_1_DIR GPIO_NUM_4
#define MCU_STEPPER_1_STP GPIO_NUM_5

#define MCU_STEPPER_2_DIR GPIO_NUM_1
#define MCU_STEPPER_2_STP GPIO_NUM_0

#define STEPPER_MAX_SPEED 1200
#define STEPPER_ACCELERATION 500

/*************For the PID******************/
#define Kp  40 // proportional gain
#define Kd  0.05 // derivative gain
#define Ki  40 // integral gain
#define sampleTime  0.005 // 5 milliseconds

/*************For ESP-NOW******************/
// Device type (MASTER or SLAVE)
#define DEVICE_TYPE SLAVE
// Debug setting (DEBUG_ON or DEBUG_OFF)
#define DEBUG_SETTING DEBUG_ON


// ----- Declare Objects -----
/*************For the Stepper Motor******************/
AccelStepper StepperR(AccelStepper::DRIVER, MCU_STEPPER_1_STP, MCU_STEPPER_1_DIR);
AccelStepper StepperL(AccelStepper::DRIVER, MCU_STEPPER_2_STP, MCU_STEPPER_2_DIR);


/***************For the interrupt timer*************/
hw_timer_t * timer = NULL; // Timer object for the ESP32 C3.

// Set alarm to call onTimer function every second (value in microseconds).
// #define ONTIMER_ALARM_TIME 1000000 //1 sec
// #define ONTIMER_ALARM_TIME 100000 //0.1 sec
#define ONTIMER_ALARM_TIME 5000 //5 miliseconds

// ----- Declare subroutines and/or functions -----
// void IRAM_ATTR onTimer();
void init_TIMER();
void PID_Calc();
void setMotors(int leftMotorSpeed, int rightMotorSpeed); 

// ----- Declare Global Variables -----

sensors_event_t a, g, temp;

bool buttonState = false;

/***************For the Algorithm******************/
float accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;

float wishedAngle = 0;
float targetAngle = 0;
float zeroAngle = 0;

// unsigned long lastTime = 0;
// unsigned long currentTime = 0;
// long wishedTime = 100;

int speedLeft = 0;
int speedRight = 0;

bool pidFlag = false;


/***********************************On Timer******************************************/
/// @brief This function will be called when the timer interrupt is triggered.
void IRAM_ATTR onTimer() {
  pidFlag = true;
}


// Setup
void setup()
{
  Serial.begin(115200); // Start the serial monitor at 115200 baud

  //Wire
  Wire.begin(MCU_SDA, MCU_SCL, 1000000); // Start the I2C communication

  //Button
  pinMode(MCU_BUTTON, INPUT_PULLUP);

  //FastLED
  FastLED.addLeds<WS2812, MCU_RGB_LED, GRB>(leds, NUM_LEDS);

  FastLED.clear(true);
  
  leds[0] = CRGB::Green;
  leds[1] = CRGB::Blue;
  leds[2] = CRGB::Red;
  FastLED.show();

  // ESPNOW
  if (!initESPNOW(DEVICE_TYPE, DEBUG_SETTING))
  {
    Serial.println("ESP-NOW initialization failed");
    ESP.restart();
  }

  startPairingProcess();
  setReceivedMessageOnMonitor(DEBUG_ESPNOW);

  //Stepper Motor
  StepperR.setMaxSpeed(STEPPER_MAX_SPEED);
  StepperR.setAcceleration(STEPPER_ACCELERATION);
  StepperL.setMaxSpeed(STEPPER_MAX_SPEED);
  StepperL.setAcceleration(STEPPER_ACCELERATION);
  
  //Initialize the MPU6050 and set offset values
  mpu.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  //OnTimer Init
  init_TIMER();

  delay(2000);
  FastLED.clear(true);

}

// Main loop
void loop()
{
  //ESPNOW
  checkPairingModeStatus(5000); // Check the pairing mode status every 5 seconds if pairing mode is active.

  //Check the IO9 Button on the SOM.
  if ((digitalRead(MCU_BUTTON) == LOW) && (buttonState == HIGH)){
    //Reset the zero position of the gyro/accel
    zeroAngle = currentAngle;
  }
  buttonState = digitalRead(MCU_BUTTON);

  //Read the accelerometer
  mpu.getEvent(&a, &g, &temp);

  accY = a.acceleration.y;
  accZ = a.acceleration.z;  
  gyroX = g.gyro.x;


  //Show the LED color depending on the rotation.
  if(accY < -2){
    leds[2] = CRGB::Green; 
  }
  else if (accY > 2){
    leds[2] = CRGB::Red;
  }
  else{
    leds[2] = CRGB::Blue;
  }
  FastLED.show();


  //Check if the timer has pulled the flag up. 
  if(pidFlag)
  {
    PID_Calc(); //Do the calculations.
    pidFlag = false;
  }

  //Constrain the motor power
  motorPower = constrain(motorPower, -255, 255);
  StepperL.setSpeed(motorPower);
  StepperR.setSpeed(motorPower);
  
  StepperL.runSpeed();
  StepperR.runSpeed();


  // if(DEBUG_ESPNOW) {
  //   //Read the incoming data from the external controller
  //   Serial.println("Reading incoming data");
  //   Serial.print("Accel X: ");
  //   Serial.println(mpuReceivingData.accX);
  //   Serial.print("Accel Y: ");
  //   Serial.println(mpuReceivingData.accY);
  //   Serial.println();
  // }
}

//use the incoming sendMpuData and display it on the serial monitor
/*

/***********************************init_TIMER******************************************/
/// @brief This function will initialize the PID loop using the hw_timer_t library.
void init_TIMER() {  

  // Create a timer with a 1 MHz frequency
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 MHz), count up

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  timerAlarmWrite(timer, ONTIMER_ALARM_TIME, true); // 1 second, autoreload true

  // Enable the alarm
  timerAlarmEnable(timer);

  //DEBUG information
  if(DEBUG_SYSTEM) {
    Serial.println("TIMER Initialized");
  }
}

/***********************************PID******************************************/
/// @brief This function will do the PID calculations.
void PID_Calc() {  

  //Calculate the angle of inclination using the accelerometer and gyroscope
  //accY needs to be reversed to create the correct front.
  accAngle = atan2(-accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;  
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * accAngle;
  
  targetAngle = wishedAngle + zeroAngle;

  //Calculate the error, error sum and motor power
  error = currentAngle - targetAngle;
  errorSum += error;   //This += is the same as errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  
  //Calculate the output from P, I and D values
  motorPower = Kp * error + Ki * errorSum * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;

  //DEBUG information
  if(DEBUG_GYRO) {
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);
    Serial.print("Target Angle: ");
    Serial.println(targetAngle);
    Serial.print("Motor Power: ");
    Serial.println(motorPower);
    Serial.println("Error: ");
    Serial.println(error);
    Serial.println("");
  }
}
