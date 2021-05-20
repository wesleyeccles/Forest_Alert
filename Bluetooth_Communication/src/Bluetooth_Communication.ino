/*
 * Project Bluetooth_Communication
 * Description: Test  of Bluetooth Com -RECIEVING Device 
 * Author: 
 * Date:
 */
#include "Particle.h"

// This example does not require the cloud so you can run it in manual mode or
// normal cloud-connected mode
 SYSTEM_MODE(MANUAL);

// These UUIDs were defined by Nordic Semiconductor and are now the defacto standard for
// UART-like services over BLE. Many apps support the UUIDs now, like the Adafruit Bluefruit app.
const BleUuid serviceUuid("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid rxUuid("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
const BleUuid txUuid("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

const size_t UART_TX_BUF_SIZE = 20;
const size_t SCAN_RESULT_COUNT = 20;

BleScanResult scanResults[SCAN_RESULT_COUNT];

BleCharacteristic peerTxCharacteristic;
BleCharacteristic peerRxCharacteristic;
BlePeerDevice peer;


uint8_t txBuf[UART_TX_BUF_SIZE];
size_t txLen = 0;

const unsigned long SCAN_PERIOD_MS = 4000;
unsigned long lastScan = 0;



void setup() {
    Serial.begin(9600);
	BLE.on();
    peerTxCharacteristic.onDataReceived(onDataReceived, &peerTxCharacteristic);
}

void loop() {
if (millis() - lastScan >= SCAN_PERIOD_MS) {
  // Time to scan
  lastScan = millis();

  size_t count = BLE.scan(scanResults, SCAN_RESULT_COUNT);
  if (count > 0) {
    for (uint8_t ii = 0; ii < count; ii++) {
      // Our serial peripheral only supports one service, so we only look for one here.
      // In some cases, you may want to get all of the service UUIDs and scan the list
      // looking to see if the serviceUuid is anywhere in the list.
      BleUuid foundServiceUuid;
      size_t svcCount = scanResults[ii].advertisingData().serviceUUID(&foundServiceUuid, 1);
      if (svcCount > 0 && foundServiceUuid == serviceUuid) {
        peer = BLE.connect(scanResults[ii].address());
        if (peer.connected()) {
          peer.getCharacteristicByUUID(peerTxCharacteristic, txUuid);
          peer.getCharacteristicByUUID(peerRxCharacteristic, rxUuid);

          // Could do this instead, but since the names are not as standardized, UUIDs are better
          // peer.getCharacteristicByDescription(peerTxCharacteristic, "tx");
        }
        break;
      }
    }
  }
  }

}


void onDataReceived(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context) {
  uint8_t dataTemp[7];
  uint8_t dataHum[7];

  int humVal;
  float tempVal;

  memcpy(dataTemp, &data[0], 5);        //Parces data to new array starting from position 0 to position 5 of data 
  memcpy(dataHum, &data[6], 5);         //Parces data to new array starting from position 6 to position 11 of data 
  tempVal = (atof((char *)&dataTemp))/100.0;
  humVal = atoi((char *)&dataHum);

  Serial.printf("Receiving Data from Bluetooth! \n");
  Serial.printf("Temp=%.2f\n",tempVal);
  Serial.printf("Humidity= %i\n",humVal);
  // for (size_t ii = 0; ii < len; ii++) {
  //   Serial.printf("%c",data[ii]);
  // }
    Serial.printf("\n");
}





// void Test(){
// if (myDriver.available()) {
//     // Should be a message for us now
//     uint8_t buf[myDriver.maxMessageLength()];
//     uint8_t buf1[myDriver.maxMessageLength()];
//     uint8_t buf2[myDriver.maxMessageLength()];
//     uint8_t len = sizeof(buf);
//     if (myDriver.recv(buf, &len)) {
//       Serial.print("Received: ");
//       Serial.println((char *)&buf);
//       // Serial.printf("The data is %i\n",atof((char *)&buf));
//       memcpy(buf1, &buf[6],5);
//       groundTemperature = (atof((char *)&buf))/100.0;
//       moistVal = atoi((char *)&buf1);
//     }