
/* Auther : AmirHosein Yousefi
   Date : 24/9/2022
   Descriptaion :
    Control Board of Hospital Room. 
    It Has 8 Digital I/O and 4 Analog I/O
    It Can Connect to Another Control Board Via ModBus Protocol ,Ethernet ,WiFi ,Bluetooth.
    It Has Onboard Boostup Voltage for LED 5V String.
    
   MCP23008 i2c Address :
   A2   A1   A0   
   -------------------- 
   L  |  L  |  L    0X20
   L  |  L  |  H    0X21
   L  |  H  |  L    0X22
   L  |  H  |  H    0X23
   H  |  L  |  L    0X24
   H  |  L  |  H    0X25
   H  |  H  |  L    0X26
   H  |  H  |  H    0X27
*/

#include <Arduino.h>
#include <Wire.h>
#include <pu2clr_mcp23008.h>
#include "Preferences.h"
#include <DFRobot_ADS1115.h>
#include "HardwareSerial.h"
#include "ModbusServerRTU.h"
#include <esp_wifi.h>
#include <WiFiClient.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"


// OTA
#define MESH_PREFIX "HopitaMainMesh"
#define MESH_PASSWORD "Officepass"
#define MESH_PORT 5555

// MCP23008 Add
#define DoutMcpAdd 0x20
#define DinMcpAdd 0x24
#define MicroSwitchMcpAdd 0x22

// Timer Constant
#define timerDivider 240
#define timer0Interval 250000

// Some Constant
#define SoftwareVersion 1
#define loopDelay 1000   //100
#define broadcastAdd 0
#define AnalogOutNumber 4
#define AnalogInNumber 4
#define DigitalOutNumber 8
#define DigitalInNumber 8
#define analogInCalib (double) 4.014
#define softwareVersionFlag 0
#define deviceStateFlag 1
#define ModbusAddFlag 2
#define DoutStateFlag 3
#define DinStateFlag 4
#define AoutVoltageFlag 5

// Device Status
#define none    (uint8_t) 0
#define startUp (uint8_t) 1
#define normal  (uint8_t) 2
#define recivedData (uint8_t) 3
#define off (uint8_t) 4

// Pin 
#define MicroSwitchInterruptPin 32
#define DoutInterruptPin 34
#define DinInterruptPin 35
#define SDAPin 21
#define SCLPin 22
#define RGBPin 19
#define PWMin1Pin 27
#define PWMin2Pin 26
#define PWMin3Pin 25
#define PWMin4Pin 33
#define mbTXpin 16
#define mbRXpin 17
#define mbCtrPin 18

#define Receive LOW
#define Trasmit HIGH

#define TAG "TIME"

MCP DoutMcp;
MCP DinMcp;
MCP MicroSwitchMcp;
Preferences NVS;
DFRobot_ADS1115 ads(&Wire);
TaskHandle_t Task1;
ModbusServerRTU MBserver(Serial2, 2000 ,mbCtrPin);
hw_timer_t * timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;
WebServer server(80);
TimerHandle_t tmr;

/*
 * Login page
 */
const char* loginIndex = 
 "<form name='loginForm'>"
    "<table width='20%' bgcolor='A09F9F' align='center'>"
        "<tr>"
            "<td colspan=2>"
                "<center><font size=4><b>ESP32 Login Page</b></font></center>"
                "<br>"
            "</td>"
            "<br>"
            "<br>"
        "</tr>"
        "<td>Username:</td>"
        "<td><input type='text' size=25 name='userid'><br></td>"
        "</tr>"
        "<br>"
        "<br>"
        "<tr>"
            "<td>Password:</td>"
            "<td><input type='Password' size=25 name='pwd'><br></td>"
            "<br>"
            "<br>"
        "</tr>"
        "<tr>"
            "<td><input type='submit' onclick='check(this.form)' value='Login'></td>"
        "</tr>"
    "</table>"
"</form>"
"<script>"
    "function check(form)"
    "{"
    "if(form.userid.value=='admin' && form.pwd.value=='admin')"
    "{"
    "window.open('/serverIndex')"
    "}"
    "else"
    "{"
    " alert('Error Password or Username')/*displays error message*/"
    "}"
    "}"
"</script>";
 
/*
 * Server Index Page
 */
 
const char* serverIndex = 
"<script src='https://ajax.googleapis.com/ajax/libs/jquery/3.2.1/jquery.min.js'></script>"
"<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>"
   "<input type='file' name='update'>"
        "<input type='submit' value='Start Update'>"
    "</form>"
 "<div id='prg'>progress: 0%</div>"
 "<script>"
  "$('form').submit(function(e){"
  "e.preventDefault();"
  "var form = $('#upload_form')[0];"
  "var data = new FormData(form);"
  " $.ajax({"
  "url: '/update',"
  "type: 'POST',"
  "data: data,"
  "contentType: false,"
  "processData:false,"
  "xhr: function() {"
  "var xhr = new window.XMLHttpRequest();"
  "xhr.upload.addEventListener('progress', function(evt) {"
  "if (evt.lengthComputable) {"
  "var per = evt.loaded / evt.total;"
  "$('#prg').html('progress: ' + Math.round(per*100) + '%');"
  "}"
  "}, false);"
  "return xhr;"
  "},"
  "success:function(d, s) {"
  "console.log('success!')" 
 "},"
 "error: function (a, b, c) {"
 "}"
 "});"
 "});"
 "</script>";



bool microSwitchCheck = false;
bool DinCheck = false;
bool AoutCheck = false;
bool saveFlag[6] = {false ,false ,false ,false ,false ,false};
bool getDout = false;
bool gReverseDirection = false;
bool modbusAvalable = false;
bool flip = false;
bool Run_OTA = false;
bool once = false;
bool changePinFlag = false;
bool relayStateOut[8] = {0};
bool Last_relayStateOut[8] = {0};
bool checkModbusData = false;

uint8_t DoutState = 0;
uint8_t DinState = 0;
uint8_t deviceStatus = none;
uint8_t modbusAdd = 0x02;
uint8_t ledCounter = 0;
uint8_t analogShiftBuf[8] = {4 ,4 ,4 ,4 ,4 ,4 ,5 ,3};
uint8_t testState = 0;
uint8_t testRelay = 0;

char reminedData = 0;

int storedSoftwareVersion = 1;
int PWMFreq[4] = {10000 ,10000 ,10000 ,10000};
int id = 1;
int interval = 100;

const int PWMChannel[4] = {0 ,1 ,2 ,3};
const int PWMresolution = 12;

uint16_t AoutVoltage[4] = {0 ,0 ,0 ,0};

unsigned long startLoopTime = 0;
unsigned long modbusReadTime = 0;
unsigned long PWMStartTime = 0;
unsigned long WiFiStartTime = 0;

const char*  softwareVersionAdd = "softwareAdd";
const char*  DoutStateAdd = "DoutStateAdd";
const char*  DinStateAdd = "DinStateAdd";
const char*  deviceStatusAdd = "deviceStatusAdd";
const char*  modbusAddAdd = "modbusAddAdd";
const char*  AoutVoltage1Add = "AoutVoltage1Add";
const char*  AoutVoltage2Add = "AoutVoltage2Add";
const char*  AoutVoltage3Add = "AoutVoltage3Add";
const char*  AoutVoltage4Add = "AoutVoltage4Add";

const char*  OTA_SSID       = "FUMP-ICT";
const char*  OTA_Password   = "12345678";
const char*  host           = "FUMP-ICT";

int AinVoltage[4] = {0};


unsigned char reverse_byte(unsigned char x) {
  static const unsigned char table[] = {
      0x00, 0x80, 0x40, 0xc0, 0x20, 0xa0, 0x60, 0xe0,
      0x10, 0x90, 0x50, 0xd0, 0x30, 0xb0, 0x70, 0xf0,
      0x08, 0x88, 0x48, 0xc8, 0x28, 0xa8, 0x68, 0xe8,
      0x18, 0x98, 0x58, 0xd8, 0x38, 0xb8, 0x78, 0xf8,
      0x04, 0x84, 0x44, 0xc4, 0x24, 0xa4, 0x64, 0xe4,
      0x14, 0x94, 0x54, 0xd4, 0x34, 0xb4, 0x74, 0xf4,
      0x0c, 0x8c, 0x4c, 0xcc, 0x2c, 0xac, 0x6c, 0xec,
      0x1c, 0x9c, 0x5c, 0xdc, 0x3c, 0xbc, 0x7c, 0xfc,
      0x02, 0x82, 0x42, 0xc2, 0x22, 0xa2, 0x62, 0xe2,
      0x12, 0x92, 0x52, 0xd2, 0x32, 0xb2, 0x72, 0xf2,
      0x0a, 0x8a, 0x4a, 0xca, 0x2a, 0xaa, 0x6a, 0xea,
      0x1a, 0x9a, 0x5a, 0xda, 0x3a, 0xba, 0x7a, 0xfa,
      0x06, 0x86, 0x46, 0xc6, 0x26, 0xa6, 0x66, 0xe6,
      0x16, 0x96, 0x56, 0xd6, 0x36, 0xb6, 0x76, 0xf6,
      0x0e, 0x8e, 0x4e, 0xce, 0x2e, 0xae, 0x6e, 0xee,
      0x1e, 0x9e, 0x5e, 0xde, 0x3e, 0xbe, 0x7e, 0xfe,
      0x01, 0x81, 0x41, 0xc1, 0x21, 0xa1, 0x61, 0xe1,
      0x11, 0x91, 0x51, 0xd1, 0x31, 0xb1, 0x71, 0xf1,
      0x09, 0x89, 0x49, 0xc9, 0x29, 0xa9, 0x69, 0xe9,
      0x19, 0x99, 0x59, 0xd9, 0x39, 0xb9, 0x79, 0xf9,
      0x05, 0x85, 0x45, 0xc5, 0x25, 0xa5, 0x65, 0xe5,
      0x15, 0x95, 0x55, 0xd5, 0x35, 0xb5, 0x75, 0xf5,
      0x0d, 0x8d, 0x4d, 0xcd, 0x2d, 0xad, 0x6d, 0xed,
      0x1d, 0x9d, 0x5d, 0xdd, 0x3d, 0xbd, 0x7d, 0xfd,
      0x03, 0x83, 0x43, 0xc3, 0x23, 0xa3, 0x63, 0xe3,
      0x13, 0x93, 0x53, 0xd3, 0x33, 0xb3, 0x73, 0xf3,
      0x0b, 0x8b, 0x4b, 0xcb, 0x2b, 0xab, 0x6b, 0xeb,
      0x1b, 0x9b, 0x5b, 0xdb, 0x3b, 0xbb, 0x7b, 0xfb,
      0x07, 0x87, 0x47, 0xc7, 0x27, 0xa7, 0x67, 0xe7,
      0x17, 0x97, 0x57, 0xd7, 0x37, 0xb7, 0x77, 0xf7,
      0x0f, 0x8f, 0x4f, 0xcf, 0x2f, 0xaf, 0x6f, 0xef,
      0x1f, 0x9f, 0x5f, 0xdf, 0x3f, 0xbf, 0x7f, 0xff,
  };
  return table[x];
}

void OTA_Init(void) {
  /*use mdns for host name resolution*/
  if (!MDNS.begin(host)) { //http://FUMP-ICT.local
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");

  /*return index page which is stored in serverIndex */
  server.on("/", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", loginIndex);
  });
  server.on("/serverIndex", HTTP_GET, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/html", serverIndex);
  });
  /*handling uploading firmware file */
  server.on("/update", HTTP_POST, []() {
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", (Update.hasError()) ? "FAIL" : "OK");
    ESP.restart();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      Serial.printf("Update: %s\n", upload.filename.c_str());
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) { //start with max available size
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      /* flashing firmware to ESP*/
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true)) { //true to set the size to the current progress
        Serial.printf("Update Success: %u\nRebooting...\n", upload.totalSize);
      } else {
        Update.printError(Serial);
      }
    }
  });

  server.begin();
}

// Set and Get Digital Port 
void setPinState(MCP handler ,int pin ,bool state) {
  if (state == HIGH) {
    handler.turnGpioOn(pin);
  } else {
    handler.turnGpioOff(pin);
  }
}

void setRawPinState(MCP handler ,uint8_t state) {
  for (size_t i = 0; i < DigitalOutNumber; i++) {
    uint8_t buf = 0b00000001 << (i);
    buf = buf & state;
    if (buf) {
      setPinState(DoutMcp ,i ,HIGH);
    } else {
      setPinState(DoutMcp ,i ,LOW);
    }
  }
}

bool getPinState(MCP handler ,int pin) {
  uint8_t data = handler.getRegister(REG_GPIO);
  uint8_t buf = 0b00000001 << (pin);

  buf = buf & data;
  if (buf) {
    return HIGH;
  } else {
    return LOW;
  }
}

uint8_t getRawPinState(MCP handler) {
  uint8_t data = handler.getRegister(REG_GPIO);

  return data;
}

// Set and Get Analog Port 
void setAnalogOut(uint8_t PwmChannel ,uint32_t miliVolt) {
  miliVolt = (miliVolt * 4096) / 24;
  ledcWrite(PwmChannel ,miliVolt);
}

uint16_t getAnalogIn(uint8_t channel) {
  return (((uint16_t) ads.readVoltage(channel)) * analogInCalib);
}

// External Interrupt ISR 
void IRAM_ATTR microswitchISR() {
  // Serial.println("ISR --> microswitchISR");
  // microSwitchCheck = true;
}

void IRAM_ATTR DinISR() {
  // Serial.println("ISR --> Digital Input");
  // DinCheck = true;
}

void restoreParameters(void) {
  storedSoftwareVersion = NVS.getInt(softwareVersionAdd);
  Serial.println("ReStore --> storedSoftwareVersion :");
  Serial.println(storedSoftwareVersion);
  
  DoutState = NVS.getInt(DoutStateAdd); 
  Serial.println("ReStore --> DoutState :");
  Serial.println(DoutState);

  deviceStatus = NVS.getInt(deviceStatusAdd); 
  Serial.println("ReStore --> deviceStatus :");
  Serial.println(deviceStatus);

  // modbusAdd = NVS.getInt(modbusAddAdd);
  // // Serial.println("ReStore --> modbusAdd :");
  // // Serial.println(modbusAdd);

  DinState = NVS.getInt(DinStateAdd); 
  Serial.println("ReStore --> DinState :");
  Serial.println(DinState);

  AoutVoltage[0] = NVS.getUInt(AoutVoltage1Add);
  Serial.println("ReStore --> AoutVoltage[0] :");
  Serial.println(AoutVoltage[0]);

  AoutVoltage[1] = NVS.getUInt(AoutVoltage2Add);
  Serial.println("ReStore --> AoutVoltage[1] :");
  Serial.println(AoutVoltage[1]);

  AoutVoltage[2] = NVS.getUInt(AoutVoltage3Add);
  Serial.println("ReStore --> AoutVoltage[2] :");
  Serial.println(AoutVoltage[2]);

  AoutVoltage[3] = NVS.getUInt(AoutVoltage4Add);
  Serial.println("ReStore --> AoutVoltage[3] :");
  Serial.println(AoutVoltage[3]);
}

void saveParameters(void) {
  if (saveFlag[softwareVersionFlag]) {
    Serial.println("Save Funvction --> software Version Is : ");
    Serial.println(storedSoftwareVersion);
    NVS.putInt(softwareVersionAdd ,storedSoftwareVersion);
    saveFlag[softwareVersionFlag] = false;
  }
  
  // if (saveFlag[deviceStateFlag]) {
  //   // Serial.println("Save Funvction --> Device State Is : ");
  //   // Serial.println(deviceStatus);
  //   NVS.putInt(deviceStatusAdd ,deviceStatus);
  //   saveFlag[deviceStateFlag] = false;
  // }
  
  if (saveFlag[ModbusAddFlag]) {
    Serial.println("Save Funvction --> Modbus Address Is : ");
    Serial.println(modbusAdd);
    NVS.putInt(modbusAddAdd ,modbusAdd);
    saveFlag[ModbusAddFlag] = false;
  }

  if (saveFlag[DoutStateFlag]) {
    DoutState = getRawPinState(DoutMcp);
    Serial.println("Save Funvction --> Digital Out State Is : ");
    Serial.println(DoutState ,BIN);
    NVS.putInt(DoutStateAdd ,DoutState);
    saveFlag[DoutStateFlag] = false;
  }

  // if (saveFlag[DinStateFlag]) {
  //   // Serial.println("Save Funvction --> Digital In State Is : ");
  //   // Serial.println(DinState ,BIN);
  //   NVS.putInt(DinStateAdd ,DinState);
  //   saveFlag[DinStateFlag] = false;
  // }

  if (saveFlag[AoutVoltageFlag]) {
    Serial.println("Save Funvction --> Analog Out Voltage Is : ");

    Serial.println(AoutVoltage[0]);
    Serial.println(AoutVoltage[1]);
    Serial.println(AoutVoltage[2]);
    Serial.println(AoutVoltage[3]);

    NVS.putUInt(AoutVoltage1Add ,AoutVoltage[0]);
    NVS.putUInt(AoutVoltage2Add ,AoutVoltage[1]);
    NVS.putUInt(AoutVoltage3Add ,AoutVoltage[2]);
    NVS.putUInt(AoutVoltage4Add ,AoutVoltage[3]);

    saveFlag[AoutVoltageFlag] = false;
  }
}

void mcpInit(void) {
  uint8_t iocon;

  DoutMcp.setup(DoutMcpAdd,0 ,-1 ,200000);

  DinMcp.setup(DinMcpAdd,255);
  iocon = DinMcp.getRegister(REG_IOCON); 
  iocon |= 0B00000110;
  DinMcp.setRegister(REG_IOCON,iocon);
  DinMcp.setRegister(REG_GPPU, 0B11111111);     // Enables the pull up resistors on pins 0,1,2,3    
  DinMcp.setRegister(REG_GPINTEN,0B11111111);   // The pins 0,1,2 and 3 of MCP23008 are now configured to lauch an interrupt
  DinMcp.setRegister(REG_DEFVAL,0B11111111);    // The pins 0,1,2 and 3 will be compared with 1. If it one of them was not equal to 1, than an interrupt will occur;
  DinMcp.setRegister(REG_INTCON,0);             // Pin value is compared against the previous pin value.

  MicroSwitchMcp.setup(MicroSwitchMcpAdd,255);
  // iocon = MicroSwitchMcp.getRegister(REG_IOCON); 
  // iocon |= 0B00000110;
  // MicroSwitchMcp.setRegister(REG_IOCON,iocon);
  // MicroSwitchMcp.setRegister(REG_GPPU, 0B00011110); // Enables the pull up resistors on pins 0,1,2,3    
  // MicroSwitchMcp.setRegister(REG_GPINTEN,0B00011110); // The pins 0,1,2 and 3 of MCP23008 are now configured to lauch an interrupt
  // MicroSwitchMcp.setRegister(REG_DEFVAL,0B00011110);  // The pins 0,1,2 and 3 will be compared with 1. If it one of them was not equal to 1, than an interrupt will occur;
  // MicroSwitchMcp.setRegister(REG_INTCON,0); // Pin value is compared against the previous pin value.
}

void pinInit(void) {
  pinMode(MicroSwitchInterruptPin ,INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(MicroSwitchInterruptPin), microswitchISR, CHANGE);

  pinMode(DoutInterruptPin ,INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(DoutInterruptPin), DoutISR, FALLING);

  pinMode(DinInterruptPin ,INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(DinInterruptPin), DinISR, CHANGE);
  
  ledcSetup(PWMChannel[0], PWMFreq[0], PWMresolution);
  ledcAttachPin(PWMin1Pin, PWMChannel[0]);

  ledcSetup(PWMChannel[1], PWMFreq[1], PWMresolution);
  ledcAttachPin(PWMin2Pin, PWMChannel[1]);

  ledcSetup(PWMChannel[2], PWMFreq[2], PWMresolution);
  ledcAttachPin(PWMin3Pin, PWMChannel[2]);

  ledcSetup(PWMChannel[3], PWMFreq[3], PWMresolution);
  ledcAttachPin(PWMin4Pin, PWMChannel[3]);

  pinMode(mbCtrPin ,OUTPUT);
  digitalWrite(mbCtrPin ,Receive);
}

void ADS1115Init(void) {
  ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS0);    // 0x48
  ads.setGain(eGAIN_ONE);                       // 2/3x gain
  ads.setMode(eMODE_SINGLE);                    // single-shot mode
  ads.setRate(eRATE_128);                       // 128SPS (default)
  ads.setOSMode(eOSMODE_SINGLE);                // Set to start a single-conversion
  ads.init();
}

uint8_t readModbusAdd(void) {
  uint8_t data = MicroSwitchMcp.getRegister(REG_GPIO);
  data = (0b00001111 & data);
  saveFlag[ModbusAddFlag] = true;
  return data;
}

void handler(void) {
  // if (microSwitchCheck) {
  //   modbusAdd = readModbusAdd();
  //   // Serial.println("Loop --> Modbus Address : ");
  //   // Serial.println(modbusAdd);
  //   microSwitchCheck = false;
  // }

  // if (DinCheck) {
  //   DinState = getRawPinState(DinMcp);
  //   // saveFlag[DinStateFlag] = true;
  //   // Serial.println("Loop --> Digital In State : ");
  //   // Serial.println(DinState);
  //   DinCheck = false;
  // }

  if ((millis() - PWMStartTime) > 500) {
    DinState = getRawPinState(DinMcp);

    AinVoltage[0] = getAnalogIn(0);
    AinVoltage[1] = getAnalogIn(1);
    AinVoltage[2] = getAnalogIn(2);
    AinVoltage[3] = getAnalogIn(3);

    analogShiftBuf[0] = (AinVoltage[0] >> 8);
    analogShiftBuf[1] = (uint8_t) (AinVoltage[0]);

    analogShiftBuf[2] = (AinVoltage[1] >> 8);
    analogShiftBuf[3] = (uint8_t) (AinVoltage[1]);

    analogShiftBuf[4] = (AinVoltage[2] >> 8);
    analogShiftBuf[5] = (uint8_t) (AinVoltage[2]);

    analogShiftBuf[6] = (AinVoltage[3] >> 8);
    analogShiftBuf[7] = (uint8_t) (AinVoltage[3]);

    PWMStartTime = millis();
  }
  
  saveParameters();
}

void ping( TimerHandle_t xTimer )
{
  for (size_t i = 0; i < 8; i++)
  {
    if (Last_relayStateOut[i] != relayStateOut[i])
    {
      setPinState(DoutMcp ,i ,relayStateOut[i]);
      Last_relayStateOut[i] = relayStateOut[i];
    }
  }
}

// Timer Function 
void IRAM_ATTR timerISR500() {
  portENTER_CRITICAL_ISR(&timerMux);
  
   DinCheck = true;
  
  portEXIT_CRITICAL_ISR(&timerMux);
}

void timerInit(void) {
  timer = timerBegin(0, timerDivider, true);
  timerAttachInterrupt(timer, &timerISR500, true);
  timerAlarmWrite(timer, timer0Interval, true);
  timerAlarmEnable(timer);
}

// Modbus FC Callback
// Read_Coil ***
ModbusMessage FC01(ModbusMessage request) {
  uint16_t address;       
  uint16_t words;             
  ModbusMessage response;   

  // Serial.println("Read_Coil");

  request.get(2, address);
  request.get(4, words);

  // Serial.print("address"); Serial.println(address);
  // Serial.print("words"); Serial.println(words);

  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(1));

  uint8_t outByte = 0b00000000;
  for (size_t i = address; i < (address + words); i++) {
    // Serial.print("i"); Serial.println(i);
    uint8_t bitCount = 0b00000001 << (7 - i);
    // Serial.print("bitCount"); Serial.println(bitCount);
    outByte = outByte | (DoutState & bitCount);
    // Serial.print("outByte"); Serial.println(outByte);
  }

  uint8_t buff = 0b00000000;
  uint8_t shift = 0x00000001;
  for (size_t i = 0; i < 8; i++) {
    uint8_t bitCount = 0b10000000 >> i;
    if ((outByte & bitCount)) {
      buff = buff | shift;
    }
    shift = shift << 1;
  }

  // Serial.print("buff"); Serial.println(buff);

  response.add(buff);

  return response;
}
// READ_DISCR_INPUT ***
ModbusMessage FC02(ModbusMessage request) {
  uint16_t address;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back

  // get request values
  request.get(2, address);
  request.get(4, words);

  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(1));

  uint8_t outByte = 0b00000000;
  for (size_t i = address; i < (address + words); i++) {
    uint8_t bitCount = 0b00000001 << i;
    outByte = outByte | (DinState & bitCount);
  }
  
  response.add(outByte);

  return response;
}
// READ_HOLD_REGISTER ***
ModbusMessage FC03(ModbusMessage request) {
  uint16_t address;        
  uint16_t words;          
  ModbusMessage response;

  request.get(2, address);
  request.get(4, words);

  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  for (uint16_t i = address; i < (address + words); i++) {
    response.add(AoutVoltage[i]);
  }

  return response;
}
// READ_INPUT_REGISTER ***
ModbusMessage FC04(ModbusMessage request) {
  uint16_t address;      
  uint16_t words;          
  ModbusMessage response;  

  // Serial.println("READ_INPUT_REGISTER");

  request.get(2, address);
  request.get(4, words);

  // Serial.print("address"); Serial.println(address);
  // Serial.print("words"); Serial.println(words);

  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  for (uint16_t i = address; i < (address + words); i++) {
    response.add((uint16_t )AinVoltage[i]);
  }

  return response;
}
// WRITE_COIL ***
ModbusMessage FC05(ModbusMessage request) {
  // Serial.println("WRITE_COIL");

  uint16_t address;
  uint16_t state;             
  ModbusMessage response; 

  request.get(2, address);
  request.get(4, state);

  // Serial.print("Relay "); Serial.print((7 - address)); Serial.print("going to "); Serial.println(state);
  relayStateOut[(7 - address)] = state;

  // setPinState(DoutMcp ,(7 - address) ,state);
  // setPinState(DoutMcp ,(7 - address) ,state);
  saveFlag[DoutStateFlag] = true;

  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(state);

  return response;
}
// WRITE_HOLD_REGISTER ***
ModbusMessage FC06(ModbusMessage request) {
  uint16_t address;      
  uint16_t value;       
  ModbusMessage response;  

  request.get(2, address);
  request.get(4, value);

  setAnalogOut(address ,value);
  saveFlag[AoutVoltageFlag] = true;

  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(value);

  return response;
}
// WRITE_MULT_COILS ***
ModbusMessage FC0F(ModbusMessage request) {
  uint16_t address;      
  uint16_t coilNum;      
  uint8_t state;            
  ModbusMessage response;   

  bool relayState[8] = {0}; 

  // Serial.print("DoutState"); Serial.println(DoutState);

  uint8_t bitCount = 0b10000000;
  for (uint8_t i = 0; i < 8; i++) {
    relayState[i] = (bitCount & DoutState);
    // Serial.print("i"); Serial.println(i);
    // Serial.print("relayState"); Serial.println(relayState[i]);
    bitCount = bitCount >> 1;
  }
  
  request.get(2, address);
  request.get(4, coilNum);
  request.get(7, state);

  // Serial.println("");
  // Serial.println("Modbus --> Recieved **WRITE_MULT_COILS**");
  // Serial.println("Start Addr      Number Of Coil      Relay State"); 
  // Serial.println(String(address) + "              " + String(coilNum) + "                   " + String(state));

  // Serial.println("Modbus --> Save Unchenged State.");
  // Serial.print("Modbus --> Current State : "); Serial.println(DoutState);

  for (uint8_t i = address; i < (address + coilNum); i++) {
    relayState[i] = state % 2;
    // Serial.print("i"); Serial.println(i);
    // Serial.print("relayState"); Serial.println(relayState[i]);
    state = state >> 1;
  }

  // bitCount = 0b00000001;
  // DoutState = 0;
  // for (uint8_t i = 0; i < 8; i++) {
  //   if (relayState[i]) {
  //     DoutState = DoutState | bitCount;
  //     // Serial.print("DoutState"); Serial.println(DoutState);
  //     // Serial.print("bitCount"); Serial.println(bitCount);
  //   }

  //   bitCount = bitCount << 1;
  // }
  for (size_t i = 0; i < 8; i++)
  {
    relayStateOut[i] = relayState[7 - i];
  }
  
  
  // DoutState = reverse_byte(DoutState);

  // Serial.print("DoutState"); Serial.println(DoutState);
  
  // setRawPinState(DoutMcp ,DoutState);


  saveFlag[DoutStateFlag] = true;

  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(coilNum);

  return response;
}
// WRITE_MULT_REGISTERS ***
ModbusMessage FC10(ModbusMessage request) {
  uint16_t address;        
  uint16_t regCount;        
  uint16_t value;           
  ModbusMessage response;  

  // Serial.println("WRITE_MULT_REGISTERS");
  request.get(2, address);
  request.get(4, regCount);

  // Serial.print("address"); Serial.println(address);
  // Serial.print("regCount"); Serial.println(regCount);

  for (uint16_t i = address; i < (address + regCount); i++) {
    request.get((7 + ((i - address) * 2)), value);
    // Serial.print("value"); Serial.println(value);
    AoutVoltage[i] = value;
    setAnalogOut(address ,value);
    // delay(50);
  }

  saveFlag[AoutVoltageFlag] = true;

  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(regCount);

  return response;
}

void modbusInit(void) {
  Serial2.begin(19200, SERIAL_8N1, GPIO_NUM_16, GPIO_NUM_17);

  MBserver.registerWorker(0x01, READ_COIL, &FC01);
  MBserver.registerWorker(0x01, WRITE_COIL, &FC05);
  MBserver.registerWorker(0x01, WRITE_MULT_COILS, &FC0F);

  MBserver.registerWorker(0x01, READ_HOLD_REGISTER, &FC03);
  MBserver.registerWorker(0x01, WRITE_HOLD_REGISTER, &FC06);
  MBserver.registerWorker(0x01, WRITE_MULT_REGISTERS, &FC10);

  MBserver.registerWorker(0x01, READ_DISCR_INPUT, &FC02);

  MBserver.registerWorker(0x01, READ_INPUT_REGISTER, &FC04);

  MBserver.start(-1 ,100);
}

void Task1code( void * parameter) {
  for(;;) {
    if ((millis() - startLoopTime) > loopDelay) {
      handler();
      startLoopTime = millis();
    }
    
    vTaskDelay(1);
  }
}

void setup() {
  Serial.begin(115200 ,SERIAL_8N1);
  while (!Serial);
  Serial.println("Setup --> Initializing Device");

  modbusInit();

  // xTaskCreatePinnedToCore(
  //     Task1code, /* Function to implement the task */
  //     "Task1", /* Name of the task */
  //     10000,  /* Stack size in words */
  //     NULL,  /* Task input parameter */
  //     0,  /* Priority of the task */
  //     &Task1,  /* Task handle. */
  //     0); /* Core where the task should run */

  NVS.begin("SM1001", false);
  pinInit();
  mcpInit();
  // timerInit();
  restoreParameters();
  ADS1115Init();

  // tmr = xTimerCreate("MyTimer", pdMS_TO_TICKS(interval), pdTRUE, ( void * )id, &ping);
  // if( xTimerStart(tmr, 10 ) != pdPASS ) {
  //   printf("Timer start error");
  // }

  if (storedSoftwareVersion != SoftwareVersion) {
    Serial.println("New Programm -> ReInitializing Variable");

    // Set Digital Out State As LOW
    DoutState = 0;
    for (size_t i = 0; i < 8; i++) {
      Last_relayStateOut[i] = 0;
    }
    
    saveFlag[DoutStateFlag] = true;
    setRawPinState(DoutMcp ,DoutState);

    // Set Analog Out As 0
    for (size_t i = 0; i < AnalogOutNumber; i++) {
      AoutVoltage[i] = 0;
      setAnalogOut(PWMChannel[i] ,AoutVoltage[i]);
    }
    saveFlag[AoutVoltageFlag] = true;

    // modbusAdd = readModbusAdd();

    // Get Digital In State
    DinState = getRawPinState(DinMcp);
    saveFlag[DinStateFlag] = true;

    // Get Analog In Value
    AinVoltage[0] = getAnalogIn(0);
    AinVoltage[1] = getAnalogIn(1);
    AinVoltage[2] = getAnalogIn(2);
    AinVoltage[3] = getAnalogIn(3);

    analogShiftBuf[0] = (AinVoltage[0] >> 8);
    analogShiftBuf[1] = (uint8_t) (AinVoltage[0]);

    analogShiftBuf[2] = (AinVoltage[1] >> 8);
    analogShiftBuf[3] = (uint8_t) (AinVoltage[1]);

    analogShiftBuf[4] = (AinVoltage[2] >> 8);
    analogShiftBuf[5] = (uint8_t) (AinVoltage[2]);

    analogShiftBuf[6] = (AinVoltage[3] >> 8);
    analogShiftBuf[7] = (uint8_t) (AinVoltage[3]);

    storedSoftwareVersion = SoftwareVersion;
    saveFlag[softwareVersionFlag] = true;
  } else {
    // Set Digital Out State
    Serial.println("Come Back With Old Variable Value");
    setRawPinState(DoutMcp ,DoutState);

    uint8_t count = 0b00000001;
    for (size_t i = 0; i < 8; i++) {
      Last_relayStateOut[i] = DoutState & count;
      relayStateOut[i] = DoutState & count;
      count = count << 1;
    }

    // Get Digital In State
    DinState = getRawPinState(DinMcp);
    saveFlag[DinStateFlag] = true;

    // Set Analog Out Value
    for (size_t i = 0; i < AnalogOutNumber; i++)
      setAnalogOut(PWMChannel[i] ,AoutVoltage[i]);

    // Get Analog In Value
    AinVoltage[0] = getAnalogIn(0);
    AinVoltage[1] = getAnalogIn(1);
    AinVoltage[2] = getAnalogIn(2);
    AinVoltage[3] = getAnalogIn(3);

    analogShiftBuf[0] = (AinVoltage[0] >> 8);
    analogShiftBuf[1] = (uint8_t) (AinVoltage[0]);

    analogShiftBuf[2] = (AinVoltage[1] >> 8);
    analogShiftBuf[3] = (uint8_t) (AinVoltage[1]);

    analogShiftBuf[4] = (AinVoltage[2] >> 8);
    analogShiftBuf[5] = (uint8_t) (AinVoltage[2]);

    analogShiftBuf[6] = (AinVoltage[3] >> 8);
    analogShiftBuf[7] = (uint8_t) (AinVoltage[3]);
  }

  startLoopTime = millis();
  PWMStartTime = millis();

  // WiFi.mode(WIFI_STA);
  // WiFi.begin(OTA_SSID ,OTA_Password);
  // WiFiStartTime = millis();

  Run_OTA = true;
  once = true;
}

void loop() {
  if ((millis() - startLoopTime) > loopDelay) {
    handler();
    startLoopTime = millis();
  }

  for (size_t i = 0; i < 8; i++)
  {
    if (Last_relayStateOut[i] != relayStateOut[i])
    {
      setPinState(DoutMcp ,i ,relayStateOut[i]);
      Last_relayStateOut[i] = relayStateOut[i];
    }
  }

  // if (Run_OTA == true)  {
  //   if (once == true) {
  //     if (WiFi.status() == WL_CONNECTED) {
  //       OTA_Init();
  //       once = false;
  //     } else if((millis() - WiFiStartTime) > 5000) {
  //       WiFi.disconnect();
  //       Run_OTA = false;
  //       once = false;
  //       WiFiStartTime = millis();
  //     }
  //   } else {
  //     server.handleClient();
  //   }
  // }   
}