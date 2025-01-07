#include <Arduino.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>


/******************************************************************************************************/
/********************************************VARIABLES*************************************************/
/******************************************************************************************************/

// Global variable to store the slave's and masters MAC address
extern uint8_t MasterMacAddress[6];
extern uint8_t SlaveMacAddress[6];
extern uint8_t BroadcastMacAddress[6];

extern bool receivedMessageOnMonitor;

// Global variable to store the pairing status
extern bool pairingMode;

extern uint8_t localPairingCycle; // This is a variable that will store the local pairing cycle.

// Global variable to store the peer information
extern esp_now_peer_info_t peerInfo;

// Enum for debug setting
enum DebugSetting
{
  DEBUG_ON,
  DEBUG_OFF,
};

// Enum for device type
enum DeviceType
{
  MASTER,
  SLAVE,
};

// Enum for message type
enum MessageType
{
  PAIRING,
  DATA,
};

// Struct for pairing
typedef struct struct_pairing
{ 
  uint8_t msgType;
  uint8_t macAddr[6];
  uint8_t pairingCycle;
  char pairingText[32];
} struct_pairing;

// Struct for message
typedef struct struct_message
{
  uint8_t msgType;
  char dataText[32];
  uint8_t dataValue;
} struct_message;

//Struct for Gyroscope/Accelerometer
typedef struct struct_gyro
{
  uint8_t msgType;
  float accX;
  float accY;
  float accZ;
  float gyroX;
  float gyroY;
  float gyroZ;
} struct_gyro;

extern DeviceType deviceType;
extern DebugSetting debugSetting;
extern MessageType messageType;

// Create 2 struct_message and 1 struct_pairing
extern struct_message sendingData;   // data to send
extern struct_message recievingData; // data received
extern struct_pairing pairingData;   // pairing data



/******************************************************************************************************/
/********************************************FUNCTIONS*************************************************/
/******************************************************************************************************/

/// @brief This function will initialize ESP-NOW depending on the deviceType.
/// @param DEVICE_TYPE
/// @param DEBUG_SETTING
bool initESPNOW(uint8_t DEVICE_TYPE, uint8_t DEBUG_SETTING);

/// @brief  Pairing: This will pair the master and slave depending on the selected device type. Data: This will receive data and save it to the recievingData struct. Outside of the library the user can access the data by using recievingData struct.
/// @param mac_addr 
/// @param incomingData 
/// @param len 
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);

/// @brief Start the pairing process depending on the selected device type.
void startPairingProcess();

/// @brief This function will process the pairing steps if the deviceType is MASTER and return the pairing status.
void pairingProcessMaster();

/// @brief This function will process the pairing steps if the deviceType is SLAVE and return the pairing status.
void pairingProcessSlave();

/// @brief This function write out all the data in the struct_message.
void printDebugData(uint8_t messageType);

/// @brief This function will set the device type.
/// @param deviceType MASTER or SLAVE
bool setDeviceType(uint8_t type);

/// @brief This function will set the debug setting.
/// @param debugSetting DEBUG_ON or DEBUG_OFF
bool setDebugSetting(uint8_t setting);

/// @param WAIT_TIME_MS Must be greater than 1000 milliseconds.
void checkPairingModeStatus(unsigned long WAIT_TIME_MS);

/// @brief This function will tell the program to write the incoming data to the Serial.Monitor.
void setReceivedMessageOnMonitor(bool state);


/// @brief This function will send data to the other device. Using ESP-NOW.
/// @param messageType
/// @param dataText
/// @param dataValue
void sendData(uint8_t messageType, char *dataText, uint8_t dataValue);

/// @brief This function will send the gyroscope/Accel data to the other device. Using ESP-NOW.
/// @param gyroX
/// @param gyroY
/// @param gyroZ
/// @param accX
/// @param accY
/// @param accZ
void sendMPUData(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ);
