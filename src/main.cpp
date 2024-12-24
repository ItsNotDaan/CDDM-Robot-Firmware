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
*/

// include libraries
#include <Arduino.h>

// // include the MPU6050 library
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

// // include the AccelStepper library
#include <AccelStepper.h>
#include <ESPNOW-EASY.h>

// #include <FastLED.h>



Adafruit_MPU6050 mpu;

// ----- Declare Constants -----

#define DEBUG 1

/*************For the ESP32C3 Wroom SOM******************/
#define MCU_SCL GPIO_NUM_8
#define MCU_SDA GPIO_NUM_9

#define MCU_RGB_LED 10 //3 WS2812-2020 RGB LEDs

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
#define targetAngle -2.5 // target angle of inclination

/*************For ESP-NOW******************/
// Device type (MASTER or SLAVE)
#define DEVICE_TYPE MASTER
// Debug setting (DEBUG_ON or DEBUG_OFF)
#define DEBUG_SETTING DEBUG_ON


// ----- Declare Objects -----
/*************For the Stepper Motor******************/
AccelStepper StepperR(AccelStepper::DRIVER, MCU_STEPPER_1_STP, MCU_STEPPER_1_DIR);
AccelStepper StepperL(AccelStepper::DRIVER, MCU_STEPPER_2_STP, MCU_STEPPER_2_DIR);

hw_timer_t *timer = NULL; // Timer object for the ESP32 C3.

// ----- Declare subroutines and/or functions -----
void IRAM_ATTR onTimer();
void setMotors(int leftMotorSpeed, int rightMotorSpeed); 
void init_PID();
void testStepperMotors();

// ----- Declare Global Variables -----
/***************For the Algorithm******************/
int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

unsigned long lastTime = 0;
unsigned long currentTime = 0;
long wishedTime = 1000;

int speedLeft = 0;
int speedRight = 0;


// Setup
void setup()
{
  Serial.begin(115200); // Start the serial monitor

  //Wire
  Wire.begin(MCU_SDA, MCU_SCL, 1000000); // Start the I2C communication

  //ESPNOW
  // if (initESPNOW(DEVICE_TYPE, DEBUG_SETTING) == false)
  // {
  //   Serial.println("ESP-NOW initialization failed");
  //   ESP.restart();
  // }

  // startPairingProcess();
  // setReceivedMessageOnMonitor(true);

  //Stepper Motor
  pinMode(MCU_STEPPER_1_DIR, OUTPUT); // Set the direction pin as an OUTPUT
  pinMode(MCU_STEPPER_1_STP, OUTPUT); // Set the step pin as an OUTPUT
  pinMode(MCU_STEPPER_2_DIR, OUTPUT); // Set the direction pin as an OUTPUT
  pinMode(MCU_STEPPER_2_STP, OUTPUT); // Set the step pin as an OUTPUT

  StepperR.setMaxSpeed(STEPPER_MAX_SPEED);
  StepperR.setAcceleration(STEPPER_ACCELERATION);
  StepperL.setMaxSpeed(STEPPER_MAX_SPEED);
  StepperL.setAcceleration(STEPPER_ACCELERATION);
  
  //Initialize the MPU6050 and set offset values
  //Initialize the MPU6050 and set offset values
  mpu.begin();
  // mpu.initialize();
  // mpu.setYAccelOffset(1593);
  // mpu.setZAccelOffset(963);
  // mpu.setXGyroOffset(40);

  //Set the wake/deepsleep switch
  // pinMode(MCU_WAKE_DEEPSLEEP, INPUT);
}

// Main loop
void loop()
{
  //Read the accelerometer
  // accY = mpu.getAccelerationY();
  // accZ = mpu.getAccelerationZ();  
  // gyroX = mpu.getRotationX();



  //Constrain the motor power
  // motorPower = constrain(motorPower, -255, 255);
  // setMotors(motorPower, motorPower);
  if (millis() - lastTime > wishedTime)
  {
    speedLeft = speedLeft - 20;
    speedLeft = speedRight + 20;

    if (speedLeft < -200)
    {
      speedLeft = 255;
    }

    if (speedRight > 200)
    {
      speedRight = -255;
    }

    lastTime = millis();
  }



  setMotors(speedLeft, speedRight);
}

/***********************************On Timer******************************************/
/// @brief This function will be called when the timer interrupt is triggered.
void IRAM_ATTR onTimer() {

  //Calculate the angle of inclination using the accelerometer and gyroscope
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;  
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * accAngle;
  
  //Calculate the error, error sum and motor power
  error = currentAngle - targetAngle;
  errorSum += error;   //This += is the same as errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);
  
  //Calculate the output from P, I and D values
  motorPower = Kp * error + Ki * errorSum * sampleTime - Kd * (currentAngle - prevAngle) / sampleTime;
  prevAngle = currentAngle;

  //DEBUG information
  if(DEBUG) {
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);
    Serial.print("Motor Power: ");
    Serial.println(motorPower);
  }
  
  // Toggle the RGB LED every second
  // count++;
  // if(count == 200) {
  //   count = 0;
  //   digitalWrite(13, !digitalRead(13));
  // }
}


/***********************************Set Motor Speed******************************************/
/// @brief This function will set the motor speed and direction using the AccelStepper library.
/// @param leftMotorSpeed The speed of the left motor.
/// @param rightMotorSpeed The speed of the right motor.
void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) { 
    StepperR.setSpeed(leftMotorSpeed);
    digitalWrite(MCU_STEPPER_1_DIR, LOW);
  }
  else { 
    StepperR.setSpeed(-leftMotorSpeed);
    digitalWrite(MCU_STEPPER_1_DIR, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    StepperL.setSpeed(rightMotorSpeed);
    digitalWrite(MCU_STEPPER_2_DIR, LOW);
  }
  else {
    StepperL.setSpeed(-rightMotorSpeed);
    digitalWrite(MCU_STEPPER_2_DIR, HIGH);
  }

  //DEBUG information
  if(DEBUG) {
    Serial.print("Left Motor Speed: ");
    Serial.println(leftMotorSpeed);
    Serial.print("Right Motor Speed: ");
    Serial.println(rightMotorSpeed);
  }
}

/***********************************init_PID******************************************/
/// @brief This function will initialize the PID loop using the hw_timer_t library. It will set the sample time to 5ms.
void init_PID() {  
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1us per tick), count up
  timerAttachInterrupt(timer, &onTimer, true); // Attach onTimer function to timer interrupt
  timerAlarmWrite(timer, 5000, true); // Set alarm to 5000 ticks (5ms), repeat
  timerAlarmEnable(timer); // Enable timer alarm

  //DEBUG information
  if(DEBUG) {
    Serial.println("PID Initialized");
  }
}


/***********************************Test Stepper Motors******************************************/
/// @brief This function will test the stepper motors by moving them forward and backward.
void testStepperMotors() {
  //Move steppper motor 1 to 1000 steps for 3 times.
  setMotors(255, 255);
}

/*
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"
#include <NewPing.h>

#define leftMotorPWMPin   6
#define leftMotorDirPin   7
#define rightMotorPWMPin  5
#define rightMotorDirPin  4

#define TRIGGER_PIN 9
#define ECHO_PIN 8
#define MAX_DISTANCE 75

#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

MPU6050 mpu;
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;
int distanceCm;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    analogWrite(leftMotorPWMPin, leftMotorSpeed);
    digitalWrite(leftMotorDirPin, LOW);
  }
  else {
    analogWrite(leftMotorPWMPin, 255 + leftMotorSpeed);
    digitalWrite(leftMotorDirPin, HIGH);
  }
  if(rightMotorSpeed >= 0) {
    analogWrite(rightMotorPWMPin, rightMotorSpeed);
    digitalWrite(rightMotorDirPin, LOW);
  }
  else {
    analogWrite(rightMotorPWMPin, 255 + rightMotorSpeed);
    digitalWrite(rightMotorDirPin, HIGH);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  // set the motor control and PWM pins to output mode
  pinMode(leftMotorPWMPin, OUTPUT);
  pinMode(leftMotorDirPin, OUTPUT);
  pinMode(rightMotorPWMPin, OUTPUT);
  pinMode(rightMotorDirPin, OUTPUT);
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(1593);
  mpu.setZAccelOffset(963);
  mpu.setXGyroOffset(40);
  // initialize PID sampling loop
  init_PID();
}

void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  // measure distance every 100 milliseconds
  if((count%20) == 0){
    distanceCm = sonar.ping_cm();
  }
  if((distanceCm < 20) && (distanceCm != 0)) {
    setMotors(-motorPower, motorPower);
  }
}
// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}
*/