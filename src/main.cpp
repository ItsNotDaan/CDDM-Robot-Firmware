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

// include the MPU6050 library
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// include the AccelStepper library
#include <AccelStepper.h>

// include the FastLED library
#include <FastLED.h>

// ----- Declare Constants -----
// Debug Settings (true or false)
#define DEBUG_GYRO true
#define DEBUG_MOTOR false
#define DEBUG_ESPNOW false
#define DEBUG_SYSTEM true

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
#define PID_KP 50 // proportional gain
#define PID_KD 0 // derivative gain
#define PID_KI 10 // integral gain
#define PID_SAMPLE_TIME 0.003 // 5 milliseconds (in seconds)

// ESP-NOW settings
#define DEVICE_TYPE SLAVE      // Device type (MASTER or SLAVE)
#define DEBUG_SETTING DEBUG_ON // Debug setting (DEBUG_ON or DEBUG_OFF)

// Timer settings (Set alarm to call onTimer function every second (value in microseconds).)
#define ONTIMER_ALARM_TIME PID_SAMPLE_TIME * 1000000 // 5 miliseconds

// ----- Declare Objects -----
// Create an object of the MPU6050 Objects
Adafruit_MPU6050 mpu;
sensors_event_t a, g, temp;

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

// ----- Declare Global Variables -----
// Button Variable
bool buttonState = false;

// PID Variables
float accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;

float wishedAngle = 0;
float targetAngle = 0;
float zeroAngle = 0;

// Timer Flag
bool pidFlag = false;

/*****************************************SETUP*************************************************/
void setup()
{
  Serial.begin(115200); // Start the serial monitor at 115200 baud

  // Wire
  Wire.begin(MCU_SDA, MCU_SCL, 1000000); // Start the I2C communication

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

  // Serial.println("Pairing mode is not active");
  // leds[0] = CRGB::Red;
  // FastLED.show();

  // checkPairingModeStatus(5000); // Check the pairing mode status every 5 seconds if pairing mode is active.
  
  // leds[0] = CRGB::Green;
  // FastLED.show();


  // Stepper Motor
  StepperR.setMaxSpeed(STEPPER_MAX_SPEED);
  StepperR.setAcceleration(STEPPER_ACCELERATION);
  StepperL.setMaxSpeed(STEPPER_MAX_SPEED);
  StepperL.setAcceleration(STEPPER_ACCELERATION);

  // Initialize the MPU6050 and set offset values
  mpu.begin();

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // OnTimer Init
  init_TIMER();

  delay(2000);

  FastLED.clear(true);
}

/*****************************************LOOP*************************************************/
void loop()
{
  // ESPNOW
  checkPairingModeStatus(5000); // Check the pairing mode status every 5 seconds if pairing mode is active.
  if (pairingMode == true)
  {
    Serial.println("Pairing mode is active");
    leds[0] = CRGB::Red;
    FastLED.show();
  }
  else
  {
    Serial.println("Pairing mode is not active");
    leds[0] = CRGB::Green;
    FastLED.show();
  }

  // Check if the zero button is pressed
  checkZeroButton();

  // Read the accelerometer
  readGyroscope();

  // Check if the timer has pulled the flag up.
  if (pidFlag)
  {
    PID_Calc(); // Do the calculations.
    pidFlag = false;
  }

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

  // Calculate the angle of inclination using the accelerometer and gyroscope
  // accY needs to be reversed to create the correct front.
  accAngle = atan2(-accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * PID_SAMPLE_TIME;
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * accAngle;

  targetAngle = wishedAngle + zeroAngle;

  // Calculate the error, error sum and motor power
  error = currentAngle - targetAngle;
  errorSum += error; // This += is the same as errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);

  // Calculate the output from P, I and D values
  motorPower = PID_KP * error + PID_KI * errorSum * PID_SAMPLE_TIME - PID_KD * (currentAngle - prevAngle) / PID_SAMPLE_TIME;
  prevAngle = currentAngle;

  // DEBUG information
  if (DEBUG_MOTOR)
  {
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

/***********************************Read Gyroscope******************************************/
/// @brief This function will read the gyroscope values and store them in the global variables.
void readGyroscope()
{
  // Read the accelerometer
  mpu.getEvent(&a, &g, &temp);

  accY = a.acceleration.y;
  accZ = a.acceleration.z;
  gyroX = g.gyro.x;

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
  motorPower = constrain(motorPower, -500, 500);
  StepperL.setSpeed(motorPower);
  StepperR.setSpeed(-motorPower);

  // Run the motors at the set speed
  StepperL.runSpeed();
  StepperR.runSpeed();
}