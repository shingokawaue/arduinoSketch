/*
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleServer.cpp
    Ported to Arduino ESP32 by Evandro Copercini
*/


static const uint32_t PROPERTY_READ = 1 << 0;
static const uint32_t PROPERTY_WRITE = 1 << 1;
static const uint32_t PROPERTY_NOTIFY = 1 << 2;
static const uint32_t PROPERTY_BROADCAST = 1 << 3;
static const uint32_t PROPERTY_INDICATE = 1 << 4;
static const uint32_t PROPERTY_WRITEWITHOUTRESPONSE = 1 << 5;
static const uint32_t PROPERTY_WRITABLEAUXILIARIES = 1 << 6;
static const uint32_t PROPERTY_SIGNEDWRITE = 1 << 7;


#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <esp_gatt_defs.h>

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define DI_SERVICE_UUID        uint16_t(0x180A)
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

void setup() {
    Serial.begin(115200);
    Serial.println("Starting BLE work!");
  Serial.println(ESP_GATT_UUID_HID_SVC);
    BLEDevice::init("ESP32^^");
    BLEServer *pServer = BLEDevice::createServer();
//    BLEService *pService1 = pServer->createService(SERVICE_UUID);
    BLEService *pDIService = pServer->createService(uint16_t(0x180A));
    BLEService *pHIDService = pServer->createService(uint16_t(0x1812));

            
//    BLECharacteristic *pCharacteristic1 = pService1->createCharacteristic(
//                                           CHARACTERISTIC_UUID,
//                                           PROPERTY_READ |
//                                           PROPERTY_WRITE//ビットフラグ
//                                         );

 //deviceInformation                                        
    BLECharacteristic *Manufacture_Name_String_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A29),
                                           PROPERTY_READ
                                         );
     BLECharacteristic *Model_Number_String_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A24),
                                           PROPERTY_READ
                                         );
    BLECharacteristic *Serial_Number_String_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A25),
                                           PROPERTY_READ
                                         );
    BLECharacteristic *Hardware_Revision_String_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A27),
                                           PROPERTY_READ
                                         );                                         
    BLECharacteristic *Firmware_Revision_String_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A26),
                                           PROPERTY_READ
                                         );

    BLECharacteristic *Software_Revision_String_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A28),
                                           PROPERTY_READ
                                         );
    BLECharacteristic *System_ID_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A23),
                                           PROPERTY_READ
                                         );                                         

    BLECharacteristic *IEEE_11073_20601_Regulatory_Certification_Data_List_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A2A),
                                           PROPERTY_READ
                                         );
    BLECharacteristic *PnP_ID_Characteristic = pDIService->createCharacteristic(
                                           uint16_t(0x2A50),
                                           PROPERTY_READ
                                         );
 //deviceInformation

  //HID                                        
    BLECharacteristic *Protocol_Mode_Characteristic = pHIDService->createCharacteristic(
                                           uint16_t(0x2A4E),
                                           PROPERTY_READ |
                                           PROPERTY_WRITEWITHOUTRESPONSE
                                         );
     BLECharacteristic *Report_Characteristic = pHIDService->createCharacteristic(
                                           uint16_t(0x2A4D),
                                           PROPERTY_READ |
                                           PROPERTY_NOTIFY
                                         );
    BLECharacteristic *Report_Map_Characteristic = pHIDService->createCharacteristic(
                                           uint16_t(0x2A4B),
                                           PROPERTY_READ
                                         );
    BLECharacteristic *Boot_Keyboard_Input_Report_Characteristic = pHIDService->createCharacteristic(
                                           uint16_t(0x2A22),
                                           PROPERTY_READ |
                                           PROPERTY_WRITE |
                                           PROPERTY_NOTIFY
                                         );                                         
    BLECharacteristic *Boot_Keyboard_Output_Report_Characteristic = pHIDService->createCharacteristic(
                                           uint16_t(0x2A32),
                                           PROPERTY_READ |
                                           PROPERTY_WRITE |
                                           PROPERTY_WRITEWITHOUTRESPONSE
                                         );

    BLECharacteristic *Boot_Mouse_Input_Report_Characteristic = pHIDService->createCharacteristic(
                                           uint16_t(0x2A33),
                                           PROPERTY_READ |
                                           PROPERTY_WRITE |
                                           PROPERTY_NOTIFY
                                         );
    BLECharacteristic *HID_Information_Characteristic = pHIDService->createCharacteristic(
                                           uint16_t(0x2A4A),
                                           PROPERTY_READ
                                         );
    BLECharacteristic *HID_Control_Point_Characteristic = pHIDService->createCharacteristic(
                                           uint16_t(0x2A4C),
                                           PROPERTY_WRITEWITHOUTRESPONSE
                                         );
 //HID

                                                                                                                                                                                                                                                                                                                                        
//    pCharacteristic1->setValue("Hello World says Neil");
//    pService1->start();
    pDIService->start();
    pHIDService->start();
    BLEAdvertising *pAdvertising = pServer->getAdvertising();
    pAdvertising->start();
    Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(2000);
}
