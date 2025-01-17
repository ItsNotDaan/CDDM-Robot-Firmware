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
  0.7 - Values on the robot can be changed using the controller. dataReceivedCheck() function is added.
  0.7.1 - Added I2C issue fix. Now when the I2C is not connected the program will not crash. 
  0.7.2 - Changed the MPU6050 library to the I2Cdev and MPU6050 library. This is because the Adafruit library is not able to change the offsets.
  0.7.3 - Fix: Changed the GyroX value to -GyroX, the slow movement is now fixed.
*/

//Calibration/Offset values
//  XAccel			              YAccel				                ZAccel			                 XGyro			         YGyro			        ZGyro
// [849,849] --> [-16,181]	[-1493,-1492] --> [-9796,11]	[1510,1511] --> [13371,16385]	[33,34] --> [-5,1]	[-32,-31] --> [0,4]	[6,6] --> [0,1]

// The values are as follows:
// XAccel = 849
// YAccel = -1493
// ZAccel = 1510
// XGyro = 33
// YGyro = -32
// ZGyro = 6

// include libraries
#include <Arduino.h>
#include "ESPNOW-EASY.h"

// include the MPU6050 library
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#include "I2Cdev.h"
#include "MPU6050.h"

// include the AccelStepper library
#include <AccelStepper.h>

// include the FastLED library
#include <FastLED.h>

// ----- Declare Constants -----
// Debug Settings (true or false)
#define DEBUG_GYRO false
#define DEBUG_MOTOR true
#define DEBUG_ESPNOW false
#define DEBUG_SYSTEM false
// GPIO Pins
#define MCU_SCL GPIO_NUM_6
#define MCU_SDA GPIO_NUM_7

#define MCU_BUTTON GPIO_NUM_9

#define MCU_RGB_LED GPIO_NUM_10 // 3 WS2812-2020 RGB LEDs

#define MCU_STEPPER_1_DIR GPIO_NUM_4
#define MCU_STEPPER_1_STP GPIO_NUM_5

#define MCU_STEPPER_2_DIR GPIO_NUM_1
#define MCU_STEPPER_2_STP GPIO_NUM_0

// LEDs
#define NUM_LEDS 3

// Stepper Motor Settings
#define STEPPER_MAX_SPEED 1200
#define STEPPER_ACCELERATION 500

// PID Settings
#define PID_SAMPLE_TIME 0.005 // 5 milliseconds (in seconds)

// ESP-NOW settings
#define DEVICE_TYPE SLAVE       // Device type (MASTER or SLAVE)
#define DEBUG_SETTING DEBUG_OFF // Debug setting (DEBUG_ON or DEBUG_OFF)

// Timer settings (Set alarm to call onTimer function every second (value in microseconds).)
#define ONTIMER_ALARM_TIME PID_SAMPLE_TIME * 1000000 // 5 miliseconds

// ----- Declare Objects -----
// Create an object of the MPU6050 Objects
// Adafruit_MPU6050 mpu;
// sensors_event_t a, g, temp;

MPU6050 mpu;

// Create the stepper motor objects
AccelStepper StepperR(AccelStepper::DRIVER, MCU_STEPPER_1_STP, MCU_STEPPER_1_DIR);
AccelStepper StepperL(AccelStepper::DRIVER, MCU_STEPPER_2_STP, MCU_STEPPER_2_DIR);

// Create the LED object
CRGB leds[NUM_LEDS];

// Create the timer object
hw_timer_t *timer = NULL; // Timer object for the ESP32 C3.

// ----- Declare subroutines and/or functions -----
void IRAM_ATTR onTimer();
void init_TIMER();
void PID_Calc();
void readGyroscope();
void checkZeroButton();
void updateMotors();
void dataReceivedCheck();

// ----- Declare Global Variables -----
// I2C initialization variables
bool initMPUDone = false; 

// Button Variable
bool buttonState = false;

// PID Variables
// float accY, accZ, gyroX;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;

float wishedAngle = 0;
float targetAngle = 0;
float zeroAngle = 0;

float PID_KP = 0.0; // proportional gain
float PID_KI = 0.0; // integral gain
float PID_KD = 0.0; // derivative gain

int motorPowerValue = 0;

// Timer Flag
bool pidFlag = false;

/*****************************************SETUP*************************************************/
void setup()
{
  Serial.begin(115200); // Start the serial monitor at 115200 baud

  // Wire
  Wire.begin(MCU_SDA, MCU_SCL); // Start the I2C communication

  // Button
  pinMode(MCU_BUTTON, INPUT_PULLUP);

  // FastLED
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

  // Stepper Motor
  StepperR.setMaxSpeed(STEPPER_MAX_SPEED);
  StepperR.setAcceleration(STEPPER_ACCELERATION);
  StepperL.setMaxSpeed(STEPPER_MAX_SPEED);
  StepperL.setAcceleration(STEPPER_ACCELERATION);

  // Initialize the MPU6050 and set offset values
  mpu.initialize(); //Gyro = +/-250 degrees/sec, Accel = +/-2G

  //Offsets
  mpu.setXAccelOffset(849);
  mpu.setYAccelOffset(-1493);
  mpu.setZAccelOffset(1510);
  mpu.setXGyroOffset(33);
  mpu.setYGyroOffset(-32);
  mpu.setZGyroOffset(6);

  initMPUDone = true;

  // OnTimer Init
  init_TIMER();

  delay(2000);

  FastLED.clear(true);
  Serial.println("Setup done");
}

/*****************************************LOOP*************************************************/
void loop()
{
  // ESPNOW
  if (pairingMode == true)
  {
    // Serial.println("Pairing mode is active");
    leds[0] = CRGB::Red;
  }
  else
  {
    // Serial.println("Pairing mode is not active");
    leds[0] = CRGB::Green;
  }
  FastLED.show();

  // Check the pairing mode status every 5 seconds if pairing mode is active.
  checkPairingModeStatus(5000);

  // Check if the zero button is pressed
  checkZeroButton();

  // Read the accelerometer
  if (initMPUDone)
  {
    readGyroscope();
  }

  // Check if the timer has pulled the flag up.
  if (pidFlag)
  {
    PID_Calc(); // Do the calculations.
    pidFlag = false;
  }

  dataReceivedCheck();

  // Update the motors
  updateMotors();
}
/****************************************END OF LOOP**********************************************/

/***********************************On Timer******************************************/
/// @brief This function will be called when the timer interrupt is triggered.
void IRAM_ATTR onTimer()
{
  pidFlag = true;
}

/***********************************init_TIMER******************************************/
/// @brief This function will initialize the PID loop using the hw_timer_t library.
void init_TIMER()
{

  // Create a timer with a 1 MHz frequency
  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1 MHz), count up

  // Attach onTimer function to our timer.
  timerAttachInterrupt(timer, &onTimer, true);

  // Set alarm to call onTimer function every second (value in microseconds).
  timerAlarmWrite(timer, ONTIMER_ALARM_TIME, true); // 1 second, autoreload true

  // Enable the alarm
  timerAlarmEnable(timer);

  // DEBUG information
  if (DEBUG_SYSTEM)
  {
    Serial.println("TIMER Initialized");
  }
}

/***********************************checkZeroButton******************************************/
/// @brief This function will check if the zero button is pressed and reset the zero angle.
void checkZeroButton()
{
  if ((digitalRead(MCU_BUTTON) == LOW) && (buttonState == HIGH))
  {
    // Reset the zero position of the gyro/accel
    zeroAngle = currentAngle;
  }
  buttonState = digitalRead(MCU_BUTTON);
}

/***********************************PID******************************************/
/// @brief This function will do the PID calculations.
void PID_Calc()
{
  // Angle Complementary Filter (PDF Slide 14)
  // Calculate the angle of inclination using the accelerometer and gyroscope
  // accY needs to be reversed to create the correct front.
  accAngle = atan2(-accY, accZ) * RAD_TO_DEG; //Find the degree of inclination of the robot using the accelerometer
  //The gyro outputs a range of 16-bit (32768 to -32767) values. The gyro rate is mapped to a value between -250 and 250
  gyroRate = map(-gyroX, -32768, 32767, -250, 250); 
  // The gyro angle is calculated by multiplying the gyro rate with the sample time
  gyroAngle = (float)gyroRate * PID_SAMPLE_TIME; // gyroX * PID_SAMPLE_TIME; =  Gyro * dT
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * accAngle;

  targetAngle = wishedAngle + zeroAngle;

  // Calculate the error, error sum and motor power
  error = currentAngle - targetAngle;
  errorSum += error; // This += is the same as errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);

  // Calculate the output from P, I and D values
  motorPower = (PID_KP * error) + (PID_KI * errorSum * PID_SAMPLE_TIME) - (PID_KD * (currentAngle - prevAngle) / PID_SAMPLE_TIME);
  prevAngle = currentAngle;

  // DEBUG information
  if (DEBUG_MOTOR)
  {
    Serial.print("Current Angle: ");
    Serial.println(currentAngle);
    // Serial.print("Target Angle: ");
    // Serial.println(targetAngle);
    // Serial.print("Motor Power: ");
    // Serial.println(motorPower);
    // Serial.println("Error: ");
    // Serial.println(error);
    // Serial.print("Acc angle: ");
    // Serial.println(accAngle);
    // Serial.print("Gyro angle: ");
    // Serial.println(gyroAngle);

    // Serial.println("");
  }
}

/***********************************Read Gyroscope******************************************/
/// @brief This function will read the gyroscope values and store them in the global variables.
void readGyroscope()
{
  // Read the accelerometer
  // mpu.getEvent(&a, &g, &temp);

  // accY = a.acceleration.y;
  // accZ = a.acceleration.z;
  // gyroX = g.gyro.x;

  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();

  if (DEBUG_GYRO)
  {
    Serial.print("AccY: ");
    Serial.println(accY);
    Serial.print("AccZ: ");
    Serial.println(accZ);
    Serial.print("GyroX: ");
    Serial.println(gyroX);
    Serial.println("");
  }
}

/***********************************Update Motors******************************************/
/// @brief This function will update the motor speeds based on the motor power.
void updateMotors()
{
  // Constrain the motor power
  motorPower = constrain(motorPower, -motorPowerValue, motorPowerValue);
  StepperL.setSpeed(motorPower);
  StepperR.setSpeed(-motorPower);

  // Run the motors at the set speed
  StepperL.runSpeed();
  StepperR.runSpeed();
}

/***********************************dataReceivedCheck******************************************/
/// @brief This function will check the incoming data and act on it.
void dataReceivedCheck()
{
  if (newDataReceived)
  {
    if (strcmp(receivingData.dataText, "KD") == 0) // Change the KD value.
    {
      PID_KD = receivingData.dataValue;

      if (DEBUG_SYSTEM)
      {
        Serial.print("The KD value is: ");
        Serial.println(PID_KD);
      }
    }
    else if (strcmp(receivingData.dataText, "KP") == 0) // Change the KP value.
    {
      PID_KP = receivingData.dataValue;

      if (DEBUG_SYSTEM)
      {
        Serial.print("The KP value is: ");
        Serial.println(PID_KP);
      }
    }
    else if (strcmp(receivingData.dataText, "KI") == 0) // Change the KI value.
    {
      PID_KI = receivingData.dataValue;

      if (DEBUG_SYSTEM)
      {
        Serial.print("The KI value is: ");
        Serial.println(PID_KI);
      }
    }
    else if (strcmp(receivingData.dataText, "ZERO") == 0) // Set the zero position.
    {
      // Reset the zero position of the gyro/accel.
      zeroAngle = currentAngle;

      if (DEBUG_SYSTEM)
      {
        Serial.println("The zero position has been reset");
      }
    }
    else if (strcmp(receivingData.dataText, "LED") == 0) // Check for a LED color
    {
      int blueValue = receivingData.dataValue;

      leds[2] = CRGB(0, 0, blueValue);

      FastLED.show();

      if (DEBUG_SYSTEM)
      {
        Serial.print("The LED blue value color is: ");
        Serial.println(blueValue);
      }
    }
    //Motor Power
    else if (strcmp(receivingData.dataText, "MOTOR") == 0)
    {
      motorPowerValue = receivingData.dataValue;

      if (DEBUG_SYSTEM)
      {
        Serial.print("The motor power value is: ");
        Serial.println(motorPowerValue);
      }
    }
    else
    {
      if (DEBUG_SYSTEM)
      {
        Serial.println("No valid data received");
      }
    }

    newDataReceived = false;
  }
}