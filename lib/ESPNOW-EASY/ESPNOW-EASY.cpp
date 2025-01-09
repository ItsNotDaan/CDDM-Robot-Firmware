#include "ESPNOW-EASY.h"

// Global variable to store the slave's and masters MAC address
uint8_t MasterMacAddress[6] = {0};
uint8_t SlaveMacAddress[6] = {0};
uint8_t BroadcastMacAddress[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

bool receivedMessageOnMonitor = false;

// Global variable to store the pairing status
bool pairingMode = false;

// Global variable to check if new data has been received
bool newDataReceived;

uint8_t localPairingCycle = 0; // This is a variable that will store the local pairing cycle.

// Global variable to store the peer information
esp_now_peer_info_t peerInfo;

DeviceType deviceType;
DebugSetting debugSetting;
MessageType messageType;

// Create 2 struct_message and 1 struct_pairing
struct_message sendingData;   // data to send
struct_message receivingData; // data received
struct_pairing pairingData;   // pairing data

// Create a struct for the gyroscope data
struct_gyro mpuReceivingData;
struct_gyro mpuSendingData;

/***************************************checkPairingModeStatus********************************************/
/// @brief This function will check if the pairing mode is active and reset the pairing process if the WAIT_TIME_MS has passed.
/// @param WAIT_TIME_MS Must be greater than 1000 milliseconds.
bool checkPairingModeStatus(unsigned long WAIT_TIME_MS)
{
  static unsigned long lastEventTime = millis();
  static unsigned long EVENT_INTERVAL_MS;

  // Set the EVENT_INTERVAL_MS to 5 seconds if the WAIT_TIME_MS is less than 1 seconds.
  if (WAIT_TIME_MS < 1000)
  {
    EVENT_INTERVAL_MS = 5000;
  }
  else
  {
    EVENT_INTERVAL_MS = WAIT_TIME_MS;
  }

  // Check if the pairing mode is active
  while (pairingMode == true)
  {
    // Reset pairing if 5 seconds have passed.
    if ((millis() - lastEventTime) > EVENT_INTERVAL_MS)
    {
      Serial.println("Pairing cycle timeout, restarting pairing process");

      lastEventTime = millis();

      // Reset the pairing process and start back at the beginning.
      startPairingProcess();
    }
  }

  return true;
}

/***************************************pairingProcessMaster********************************************/
/// @brief This function will process the pairing steps if the deviceType is MASTER and return the pairing status.
void pairingProcessMaster()
{
  /*CYCLE 2 OF 3 - RECEIVE THE SLAVE'S MAC ADDRESS AND SEND BACK "M-CYCLE-2/3" + LOCAL CYCLE TO THE SLAVE*/
  if (pairingData.pairingCycle == 1 && localPairingCycle == 2)
  {
    if (debugSetting == DEBUG_ON)
    {
      Serial.print("Pairing cycle ");
      Serial.print(localPairingCycle);
      Serial.println(": Slave MAC Address received");
    }

    // Save the MAC address of the slave
    memcpy(SlaveMacAddress, pairingData.macAddr, 6);

    // Remove the global broadcast peer
    esp_now_del_peer(BroadcastMacAddress);

    // Add the new peer with the slave's MAC address
    memcpy(peerInfo.peer_addr, SlaveMacAddress, 6);
    peerInfo.channel = 0; // Use the current Wi-Fi channel
    peerInfo.encrypt = false;

    // Add the peer
    esp_now_add_peer(&peerInfo);

    if (debugSetting == DEBUG_ON)
    {
      Serial.println("Removed global broadcast peer");
      Serial.println("Added new peer with Slave's MAC address");
    }

    // Send Local Pairing Cycle 2 to the slave together with the pairing text.
    pairingData.msgType = PAIRING;
    pairingData.pairingCycle = localPairingCycle;
    strcpy(pairingData.pairingText, "M-CYCLE-2/3"); // Master in cycle 2 of 3.
    memcpy(pairingData.macAddr, MasterMacAddress, 6);

    // Send the OK response to the slave
    esp_now_send(SlaveMacAddress, (const uint8_t *)&pairingData, sizeof(pairingData));

    if (debugSetting == DEBUG_ON)
    {
      Serial.println("Pairing cycle 2: Slave MAC Address saved and M-CYCLE-2/3 sent to slave");
    }

    // Add one to the localPairingCycle to 3.
    localPairingCycle++;
  }

  /*CYCLE 3 OF 3 - RECEIVE THE FINAL RESPONSE FROM THE SLAVE AND SEND BACK "M-CYCLE-3/3" + LOCAL CYCLE TO THE SLAVE AND SET PAIRING MODE TO "PAIRED"*/
  else if (pairingData.pairingCycle == 2 && localPairingCycle == 3)
  {
    if (debugSetting == DEBUG_ON)
    {
      Serial.print("Pairing cycle ");
      Serial.print(localPairingCycle);
      Serial.println(": Final response received from slave");
    }

    // Send last pairing message to the slave
    pairingData.msgType = PAIRING;
    pairingData.pairingCycle = localPairingCycle;
    strcpy(pairingData.pairingText, "M-CYCLE-3/3"); // Master in cycle 3 of 3.
    memcpy(pairingData.macAddr, MasterMacAddress, 6);
    esp_now_send(SlaveMacAddress, (const uint8_t *)&pairingData, sizeof(pairingData));

    // // Turn LED to GREEN indicate pairing mode
    // RGB_LED(true, 0, 255, 0);

    // Set pairing mode to false.
    pairingMode = false;

    if (debugSetting == DEBUG_ON)
    {
      Serial.println("Pairing cycle 3: Final response M-CYCLE-3/3 sent to slave");
      Serial.println("Pairing complete");
    }
  }
  return;
}

/***************************************pairingProcessSlave********************************************/
/// @brief This function will process the pairing steps if the deviceType is SLAVE and return the pairing status.
void pairingProcessSlave()
{
  /*CYCLE 1 OF 3 - RECEIVE THE MASTER'S MAC ADDRESS AND SEND BACK "S-CYCLE-1/3" + LOCAL CYCLE TO THE MASTER*/
  if (pairingData.pairingCycle == 1 && localPairingCycle == 1)
  {
    if (debugSetting == DEBUG_ON)
    {
      Serial.print("Pairing cycle ");
      Serial.print(localPairingCycle);
      Serial.println(": Master MAC Address received");
    }

    // Save the MAC address of the master
    memcpy(MasterMacAddress, pairingData.macAddr, 6);

    // Add the master to the peer list
    memcpy(peerInfo.peer_addr, MasterMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    if (debugSetting == DEBUG_ON)
    {
      Serial.println("Master MAC Address saved");
      Serial.println("Master added to peer list");
    }

    // Send Local Pairing Cycle 1 to the master together with the pairing text.
    pairingData.msgType = PAIRING;
    pairingData.pairingCycle = localPairingCycle;
    strcpy(pairingData.pairingText, "S-CYCLE-1/3"); // Slave in cycle 1 of 3.
    memcpy(pairingData.macAddr, SlaveMacAddress, 6);

    // Send the slave's MAC address to the master
    esp_now_send(MasterMacAddress, (const uint8_t *)&pairingData, sizeof(pairingData));

    if (debugSetting == DEBUG_ON)
    {
      Serial.println("Pairing cycle 1: Master MAC Address saved and S-CYCLE-1/3 sent to master");
    }

    // Add one to the localPairingCycle to 2.
    localPairingCycle++;
  }

  /*CYCLE 2 OF 3 - RECEIVE THE RESPONSE FROM THE MASTER AND SEND BACK "S-CYCLE-2/3" + LOCAL CYCLE TO THE MASTER*/
  else if (pairingData.pairingCycle == 2 && localPairingCycle == 2)
  {
    if (debugSetting == DEBUG_ON)
    {
      Serial.print("Pairing cycle ");
      Serial.print(localPairingCycle);
      Serial.println(": Final response received from master");
    }

    // Send last pairing message to the master
    pairingData.msgType = PAIRING;
    pairingData.pairingCycle = localPairingCycle;
    strcpy(pairingData.pairingText, "S-CYCLE-2/3"); // Slave in cycle 2 of 3.
    memcpy(pairingData.macAddr, SlaveMacAddress, 6);
    esp_now_send(MasterMacAddress, (const uint8_t *)&pairingData, sizeof(pairingData));

    if (debugSetting == DEBUG_ON)
    {
      Serial.println("Pairing cycle 2: Received responce and send S-CYCLE-2/3 sent to master");
    }

    // Add one to the localPairingCycle to 3.
    localPairingCycle++;
  }

  /*CYCLE 3 OF 3 - RECEIVE THE FINAL RESPONSE FROM THE MASTER AND SET PAIRING MODE TO "PAIRED"*/
  else if (pairingData.pairingCycle == 3 && localPairingCycle == 3)
  {
    if (debugSetting == DEBUG_ON)
    {
      Serial.print("Pairing cycle ");
      Serial.print(localPairingCycle);
      Serial.println(": Final response received from master");
    }

    // // Turn LED to GREEN indicate pairing mode
    // RGB_LED(true, 0, 255, 0);

    // Set pairing mode to false.
    pairingMode = false;

    if (debugSetting == DEBUG_ON)
    {
      Serial.println("Pairing complete");
    }
  }

  // If the master is trying to pair again, restart the pairing process.
  else if (pairingData.pairingCycle == 1 && pairingMode == false)
  {
    Serial.println("Master seems to be trying to pair again, restarting pairing process");
    startPairingProcess();
  }
}

/***************************************startPairingProcess********************************************/
/// @brief Start the pairing process depending on the selected device type.
void startPairingProcess()
{
  // // Turn LED to BLUE indicate pairing mode.
  // RGB_LED(true, 0, 0, 255);

  // Set pairing mode to true.
  pairingMode = true;

  // Set the pairing cycle to 1.
  localPairingCycle = 1;

  // Check which device type is selected and start the pairing process accordingly.
  switch (deviceType)
  {
  case MASTER:
    /*CYCLE 1 OF 3 - BROADCAST THE MASTER'S MAC ADDRESS TO THE SLAVE TOGETHER WITH "M-CYCLE-1/3" + LOCAL CYCLE TO THE SLAVE AND SET PAIRING MODE TO "PAIRED"*/

    // Create the first pairing message.
    pairingData.msgType = PAIRING;
    pairingData.pairingCycle = localPairingCycle;
    strcpy(pairingData.pairingText, "M-CYCLE-1/3"); // Master in cycle 1 of 3.
    memcpy(pairingData.macAddr, MasterMacAddress, sizeof(pairingData.macAddr));

    // Create a peer and add it to the peer list. The peer address is the broadcast address.
    memcpy(peerInfo.peer_addr, BroadcastMacAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);

    // Send the pairing message to the slave/global address.
    esp_now_send(BroadcastMacAddress, (const uint8_t *)&pairingData, sizeof(pairingData));

    // FOR DEBUGGING
    if (debugSetting == DEBUG_ON)
    {
      Serial.print("Pairing cycle ");
      Serial.print(localPairingCycle);
      Serial.println(": Master MAC Address broadcasted together with M-CYCLE-1/3");
    }

    // Increase the local pairing cycle. Master has done its first pairing cycle.
    localPairingCycle++;
    break;

  case SLAVE:
    // FOR DEBUGGING
    if (debugSetting == DEBUG_ON)
    {
      Serial.print("Pairing cycle ");
      Serial.print(localPairingCycle);
      Serial.println(": Waiting for Master MAC Address broadcast");
    }
    break;

  default:
    Serial.println("No device type selected \n Please select a device type to start the pairing process.");
    break;
  }

  // // FOR DEBUGGING
  // if (debugSetting == DEBUG_ON)
  // {
  //   Serial.print("Pairing cycle: ");
  //   Serial.println(localPairingCycle);
  // }
}

/***************************************initESPNOW********************************************/
/// @brief This function will initialize ESP-NOW depending on the deviceType.
/// @param DEVICE_TYPE MASTER or SLAVE
/// @param DEBUG_SETTING DEBUG_ON or DEBUG_OFF
bool initESPNOW(uint8_t DEVICE_TYPE, uint8_t DEBUG_SETTING)
{
  bool initSuccess = true;

  // Set the device type and debug setting
  if (!setDeviceType(DEVICE_TYPE))
  {
    initSuccess = false;
  }
  if (!setDebugSetting(DEBUG_SETTING))
  {
    initSuccess = false;
  }

  // Initialize WiFi and register the callback function of ESP-NOW.
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK)
  {
    Serial.println("Error initializing ESP-NOW");
    initSuccess = false;
  }
  esp_now_register_recv_cb(OnDataRecv);

  // Save the mac address of the selected device type to the global variable.
  switch (deviceType)
  {
  case MASTER:
    esp_read_mac(MasterMacAddress, ESP_MAC_WIFI_STA);
    break;
  case SLAVE:
    esp_read_mac(SlaveMacAddress, ESP_MAC_WIFI_STA);
    break;
  default:
    Serial.println("No device type selected \n Please select a device type to save the MAC address.");
    initSuccess = false;
    break;
  }

  // FOR DEBUGGING
  if (debugSetting == DEBUG_ON)
  {
    Serial.println("ESP-NOW initialized");
    Serial.println("The device type is: ");
    Serial.println(deviceType);

    Serial.print("MAC Address: ");
    uint8_t tempMacAddress[6] = {0}; // Temporary MAC address storage
    esp_read_mac(tempMacAddress, ESP_MAC_WIFI_STA);
    for (int i = 0; i < 6; i++)
    {
      Serial.printf("%02X", tempMacAddress[i]);
      if (i < 5)
        Serial.print(":");
    }
    Serial.println();
  }
  return initSuccess;
}

/***************************************OnDataRecv********************************************/
/// @brief  Pairing: This will pair the master and slave depending on the selected device type. Data: This will receive data and save it to the receivingData struct. Outside of the library the user can access the data by using receivingData struct.
/// @param mac_addr
/// @param incomingData
/// @param len
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len)
{
  newDataReceived = true;
  uint8_t type = incomingData[0]; // first message byte is the type of message

  switch (type)
  {
  case PAIRING: // the message is pairing type
    memcpy(&pairingData, incomingData, sizeof(pairingData));

    switch (deviceType)
    {
    case MASTER:
      pairingProcessMaster();
      break;
    case SLAVE:
      pairingProcessSlave();
      break;
    default:
      Serial.println("No device type selected \n Please select a device type to start the pairing process.");
      break;
    }
    break;

  case DATA: // the message is data type
    memcpy(&receivingData, incomingData, sizeof(receivingData));

    // FOR DEBUGGING
    if (debugSetting == DEBUG_ON)
    {
      Serial.print(len);
      Serial.println(" bytes of new data received.");
    }
    if (receivedMessageOnMonitor)
    {
      printDebugData(type);
    }

  case GYRO: // the message is gyroscope/accelerometer type
    memcpy(&mpuReceivingData, incomingData, sizeof(mpuReceivingData));

    // FOR DEBUGGING
    if (receivedMessageOnMonitor)
    {
      printDebugData(type);
    }

    break;

  default:
    Serial.println("Unknown message type");
    break;
  }
}

/***************************************printDebugData********************************************/
/// @brief This function write out all the data in the struct_message.
void printDebugData(uint8_t messageType)
{
  switch (messageType)
  {
  case DATA:
    Serial.println("Data recieved:");
    Serial.print("Data Text: ");
    Serial.println(receivingData.dataText);
    Serial.print("Data Value: ");
    Serial.println(receivingData.dataValue);
    Serial.println();
    break;

  case PAIRING:
    Serial.println("Pairing data recieved:");
    Serial.print("Pairing Cycle: ");
    Serial.println(pairingData.pairingCycle);
    Serial.print("MAC Address: ");
    for (int i = 0; i < 6; i++)
    {
      Serial.printf("%02X", pairingData.macAddr[i]);
      if (i < 5)
        Serial.print(":");
    }
    Serial.println();
    break;

  case GYRO:
    Serial.println("Gyroscope/Accelerometer data recieved:");
    Serial.print("Gyro X: ");
    Serial.println(mpuReceivingData.gyroX);
    Serial.print("Gyro Y: ");
    Serial.println(mpuReceivingData.gyroY);
    Serial.print("Gyro Z: ");
    Serial.println(mpuReceivingData.gyroZ);
    Serial.print("Acc X: ");
    Serial.println(mpuReceivingData.accX);
    Serial.print("Acc Y: ");
    Serial.println(mpuReceivingData.accY);
    Serial.print("Acc Z: ");
    Serial.println(mpuReceivingData.accZ);
    Serial.println();
    break;

  default:
    Serial.println("Unknown message type");
    break;
  }
}

/***********************************Set Device Type******************************************/
/// @brief This function will set the device type. Returns false if no device type is selected.
/// @param deviceType MASTER or SLAVE
bool setDeviceType(uint8_t type)
{
  bool success = true;
  switch (type)
  {
  case MASTER:
    deviceType = MASTER;
    break;
  case SLAVE:
    deviceType = SLAVE;
    break;
  default:
    Serial.println("No device type selected");
    success = false;
    break;
  }
  return success;
}

/***********************************Set Debug Setting******************************************/
/// @brief This function will set the debug setting. Returns false if no debug setting is selected.
/// @param debugSetting DEBUG_ON or DEBUG_OFF
bool setDebugSetting(uint8_t setting)
{
  bool success = true;
  switch (setting)
  {
  case DEBUG_ON:
    debugSetting = DEBUG_ON;
    break;
  case DEBUG_OFF:
    debugSetting = DEBUG_OFF;
    break;
  default:
    Serial.println("No debug setting selected");
    success = false;
    break;
  }
  return success;
}

/*******************************setReceivedMessageOnMonitor*************************************/
/// @brief This function will tell the program to write the incoming data to the Serial.Monitor.
/// @param state true or false (default is false)
void setReceivedMessageOnMonitor(bool state)
{
  receivedMessageOnMonitor = state;
}

/***************************************sendData********************************************/
/// @brief This function will send data to the other device. Using ESP-NOW.
/// @param messageType DATA or PAIRING
/// @param dataText
/// @param dataValue
void sendData(uint8_t messageType, char *dataText, float dataValue)
{
  sendingData.msgType = messageType;
  strcpy(sendingData.dataText, dataText);
  sendingData.dataValue = dataValue;

  switch (deviceType)
  {
  case MASTER:
    esp_now_send(SlaveMacAddress, (const uint8_t *)&sendingData, sizeof(sendingData));
    break;
  case SLAVE:
    esp_now_send(MasterMacAddress, (const uint8_t *)&sendingData, sizeof(sendingData));
    break;
  default:
    Serial.println("No device type selected \n Please select a device type to send data.");
    break;
  }
}

/***************************************sendMpuData********************************************/
/// @brief This function will send the gyroscope/Accel data to the other device. Using ESP-NOW.
/// @param gyroX
/// @param gyroY
/// @param gyroZ
/// @param accX
/// @param accY
/// @param accZ
void sendMpuData(float gyroX, float gyroY, float gyroZ, float accX, float accY, float accZ)
{
  mpuSendingData.msgType = GYRO;
  mpuSendingData.gyroX = gyroX;
  mpuSendingData.gyroY = gyroY;
  mpuSendingData.gyroZ = gyroZ;
  mpuSendingData.accX = accX;
  mpuSendingData.accY = accY;
  mpuSendingData.accZ = accZ;

  switch (deviceType)
  {
  case MASTER:
    esp_now_send(SlaveMacAddress, (const uint8_t *)&mpuSendingData, sizeof(mpuSendingData));
    break;
  case SLAVE:
    esp_now_send(MasterMacAddress, (const uint8_t *)&mpuSendingData, sizeof(mpuSendingData));
    break;
  default:
    Serial.println("No device type selected \n Please select a device type to send data.");
    break;
  }
}
