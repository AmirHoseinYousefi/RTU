#include <Arduino.h>
#include <Wire.h>
#include <pu2clr_mcp23008.h>
#include "Preferences.h"
#include <DFRobot_ADS1115.h>
#include "HardwareSerial.h"
#include <SPI.h>
#include <Ethernet.h>
#include <ESP32Scheduler.h>
#include <Task.h>

// W5500 reset pin 
#define RESET_P GPIO_NUM_4
#define COIL_NUMBER 8
#define W5500_CS_PIN GPIO_NUM_15

// MCP23008 Add
#define DoutMcpAdd 0x20
#define DinMcpAdd 0x24
#define MicroSwitchMcpAdd 0x22

// Some Constant
#define SoftwareVersion 1
#define loopDelay 200   //100
#define AnalogOutNumber 4
#define DigitalOutNumber 8
#define analogInCalib (double) 4.014

#define softwareVersionFlag 0
#define ModbusAddFlag 2
#define DoutStateFlag 3
#define DinStateFlag 4
#define AoutVoltageFlag 5

// Pin 
#define MicroSwitchInterruptPin 32
#define DoutInterruptPin 34
#define DinInterruptPin 35
#define RGBPin 19
#define PWMin1Pin 27
#define PWMin2Pin 26
#define PWMin3Pin 25
#define PWMin4Pin 33
#define mbTXpin 17
#define mbRXpin 16
#define mbCtrPin 18

#define Receive LOW
#define Trasmit HIGH


uint8_t mac[] = { 0x30, 0x2B, 0x2D, 0x2F, 0x61, 0xE2 }; 
IPAddress staticIP = {192, 168, 100, 120};          // IP address of modbus server
IPAddress localIP;
IPAddress localSubnet = {255, 255, 255, 0};
IPAddress localGateway = {192, 168, 100, 1};
IPAddress localDNS = {8, 8, 8, 8};

MCP DoutMcp;
MCP DinMcp;
MCP MicroSwitchMcp;
Preferences NVS;
DFRobot_ADS1115 ads(&Wire);
TaskHandle_t Task1;

int serverID = 5;
uint16_t port = 502;                      // port of modbus server                      

#include "ModbusServerEthernet.h"

ModbusServerEthernet MB;

const uint16_t PORT(502);

bool AoutCheck = false;
bool saveFlag[6] = {false ,false ,false ,false ,false ,false};
bool getDout = false;
bool gReverseDirection = false;
bool modbusAvalable = false;
bool flip = false;
bool changePinFlag = false;
bool coilState[8] = {1 ,1 ,1 ,1 ,1 ,1 ,1 ,1};
bool lastCoilState[8] = {0 ,0 ,0 ,0 ,0 ,0 ,0 ,0};
bool checkModbusData = false;

uint8_t DoutState = 0;
uint8_t DinState = 0;
uint8_t modbusAdd = 0x02;
uint8_t ledCounter = 0;
uint8_t analogShiftBuf[8] = {4 ,4 ,4 ,4 ,4 ,4 ,5 ,3};
uint8_t testState = 0;
uint8_t testRelay = 0;

char reminedData = 0;

int storedSoftwareVersion = 1;
int PWMFreq[4] = {500 ,500 ,500 ,500};
int modbusId = 1;
int interval = 100;

const int PWMChannel[4] = {0 ,2 ,4 ,6};
int pwmPin[4] = {27 ,26 ,25 ,33};
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

int AinVoltage[4] = {0 ,0 ,0 ,0};


uint8_t readModbusAdd(void) {
  uint8_t data = MicroSwitchMcp.getRegister(REG_GPIO);
  data = ((0b00011110 & data) >> 1) + 1;
  return data;
}

// Set and Get Digital Port 
void setCoil(MCP handler ,int pin ,bool state) {
  if (state == HIGH) {
    handler.setRegister(REG_GPIO ,handler.getRegister(REG_GPIO) | (1 << pin));
  } else {
    handler.setRegister(REG_GPIO ,handler.getRegister(REG_GPIO) & (~(1 << pin)));
  }
}

void setAllCoils(MCP handler ,uint8_t state) {
  handler.setRegister(REG_GPIO ,state);
}

bool getCoil(MCP handler ,int pin) {
  uint8_t data = handler.getRegister(REG_GPIO);
  uint8_t buf = 0b00000001 << (pin);

  buf = buf & data;
  if (buf) {
    return HIGH;
  } else {
    return LOW;
  }
}

uint8_t getAllCoils(MCP handler) {
  uint8_t data = handler.getRegister(REG_GPIO);

  return data;
}

// Set and Get Analog Port 
void setAnalogOut(uint8_t PwmChannel ,double miliVolt) {
  ledcWrite(PwmChannel ,(int)(miliVolt * 0.3820) );    // 3820(maximum duty) / 10000 (maximum input)
}

uint16_t getAnalogIn(uint8_t channel) {
  return (((uint16_t) ads.readVoltage(channel)) * analogInCalib);
}

void restoreParameters(void) {
  storedSoftwareVersion = NVS.getInt(softwareVersionAdd ,0);
  Serial.print("ReStore --> storedSoftwareVersion : ");
  Serial.println(storedSoftwareVersion);

  if (storedSoftwareVersion != SoftwareVersion) {
    // Set Digital Out State As LOW
    for (size_t i = 0; i < 8; i++) {
      coilState[i] = 0;
    }
    saveFlag[DoutStateFlag] = true;

    // Set Analog Out As 0
    for (size_t i = 0; i < AnalogOutNumber; i++) {
      AoutVoltage[i] = 0;
      setAnalogOut(PWMChannel[i] ,AoutVoltage[i]);
    }
    saveFlag[AoutVoltageFlag] = true;

    modbusAdd = readModbusAdd();
    saveFlag[ModbusAddFlag] = true;

    // Get Digital In State
    DinState = getAllCoils(DinMcp);
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

    DoutState = NVS.getInt(DoutStateAdd); 

    uint8_t b = 0b00000001;
    for (size_t i = 0; i < 8; i++) {
      coilState[7 - i] = (b << i) & DoutState ;
    }
    
    Serial.print("ReStore --> DoutState : ");
    Serial.println(DoutState);

    AoutVoltage[0] = NVS.getUInt(AoutVoltage1Add);
    Serial.print("ReStore --> AoutVoltage[0] : ");
    Serial.println(AoutVoltage[0]);

    AoutVoltage[1] = NVS.getUInt(AoutVoltage2Add);
    Serial.print("ReStore --> AoutVoltage[1] : ");
    Serial.println(AoutVoltage[1]);

    AoutVoltage[2] = NVS.getUInt(AoutVoltage3Add);
    Serial.print("ReStore --> AoutVoltage[2] : ");
    Serial.println(AoutVoltage[2]);

    AoutVoltage[3] = NVS.getUInt(AoutVoltage4Add);
    Serial.print("ReStore --> AoutVoltage[3] : ");
    Serial.println(AoutVoltage[3]);

    // Set Analog Out Value
    for (size_t i = 0; i < AnalogOutNumber; i++) {
      setAnalogOut(PWMChannel[i] ,AoutVoltage[i]);
    }


    // Get Digital In State
    DinState = getAllCoils(DinMcp);
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
  }
  
}

void saveParameters(void) {
  if (saveFlag[softwareVersionFlag]) {
    Serial.println("Save Funvction --> software Version Is : ");
    Serial.println(storedSoftwareVersion);
    NVS.putInt(softwareVersionAdd ,storedSoftwareVersion);
    saveFlag[softwareVersionFlag] = false;
  }

  if (saveFlag[DoutStateFlag]) {
    DoutState = getAllCoils(DoutMcp);
    Serial.println("Save Funvction --> Digital Out State Is : ");
    Serial.println(DoutState ,BIN);
    NVS.putInt(DoutStateAdd ,DoutState);
    saveFlag[DoutStateFlag] = false;
  }

  if (saveFlag[AoutVoltageFlag]) {
    Serial.println("Save Funvction --> Analog Out Voltage Is : ");

    Serial.println(AoutVoltage[0]);
    Serial.println(AoutVoltage[1]);
    Serial.println(AoutVoltage[2]);
    Serial.println(AoutVoltage[3]);

    NVS.putInt(AoutVoltage1Add ,AoutVoltage[0]);
    NVS.putInt(AoutVoltage2Add ,AoutVoltage[1]);
    NVS.putInt(AoutVoltage3Add ,AoutVoltage[2]);
    NVS.putInt(AoutVoltage4Add ,AoutVoltage[3]);

    saveFlag[AoutVoltageFlag] = false;
  }

  if (saveFlag[ModbusAddFlag]) {
    Serial.println("Save Funvction -->modbus address Is : ");
    NVS.putUInt(modbusAddAdd ,modbusAdd);
  
    saveFlag[ModbusAddFlag] = false;
  }
}

void mcpInit(void) {
  // Digital input initializing 
  DoutMcp.setup(DoutMcpAdd,0);


  // Digital input initializing 
  DinMcp.setup(DinMcpAdd,0xFF);
  // Config all pins as input
  DinMcp.setRegister(REG_IODIR ,0xFF);
  // reverse input polarite (High reads as LOW)
  DinMcp.setRegister(REG_IPOL ,0xFF);
  // enable interrupt-on-change of all pins
  DinMcp.setRegister(REG_GPINTEN ,0xFF);
  // config how to occure interrupt : compare with DEFVAL register
  DinMcp.setRegister(REG_INTCON ,0x00);
  // set Default value for interrupt check
  DinMcp.setRegister(REG_DEFVAL ,0x00);
  // 0 ,0 ,Sequential Operation ,Slew Rate ,0 ,INT output ,INT polarite
  DinMcp.setRegister(REG_IOCON ,0b00100000);


  // microswitch initializing 
  MicroSwitchMcp.setup(MicroSwitchMcpAdd,255);
  // Config all pins as input
  MicroSwitchMcp.setRegister(REG_IODIR ,0xFF);
  // reverse input polarite (High reads as LOW)
  MicroSwitchMcp.setRegister(REG_IPOL ,0x00);
  // enable interrupt-on-change of all pins
  MicroSwitchMcp.setRegister(REG_GPINTEN ,0xFF);
  // config how to occure interrupt : compare with DEFVAL register
  MicroSwitchMcp.setRegister(REG_INTCON ,0xFF);
  // set Default value for interrupt check
  MicroSwitchMcp.setRegister(REG_DEFVAL ,0xFF);
  // 0 ,0 ,Sequential Operation ,Slew Rate ,0 ,INT output ,INT polarite
  MicroSwitchMcp.setRegister(REG_IOCON ,0b00101000);

  
  //   mcpn.setup(DinMcpAdd);
  //   mcpn.setRegister(REG_IODIR ,0xFF);
  //   mcpn.setRegister(REG_IPOL ,0xFF);
  //   mcpn.setRegister(REG_GPINTEN ,0xFF);
  //   mcpn.setRegister(REG_INTCON ,0xFF);
  //   mcpn.setRegister(REG_DEFVAL ,0x00);
  //   mcpn.setRegister(REG_IOCON ,0b00101000);
  //   mcpn.setRegister(REG_IOCON ,0xFF);
}

void pinInit(void) {
  // input interrupt pins
  pinMode(MicroSwitchInterruptPin ,INPUT_PULLUP);
  pinMode(DoutInterruptPin ,INPUT_PULLUP);
  pinMode(DinInterruptPin ,INPUT_PULLUP);

  // Rs485 modbus ctl Trasmit/Receive
  pinMode(mbCtrPin ,OUTPUT);
  digitalWrite(mbCtrPin ,Receive);
  
  // Analog output 1
  ledcSetup(PWMChannel[0], PWMFreq[0], PWMresolution);
  ledcAttachPin(pwmPin[0], PWMChannel[0]);

  // Analog output 2
  ledcSetup(PWMChannel[1], PWMFreq[1], PWMresolution);
  ledcAttachPin(pwmPin[1], PWMChannel[1]);

  // Analog output 3
  ledcSetup(PWMChannel[2], PWMFreq[2], PWMresolution);
  ledcAttachPin(pwmPin[2], PWMChannel[2]);

  // Analog output 4
  ledcSetup(PWMChannel[3], PWMFreq[3], PWMresolution);
  ledcAttachPin(pwmPin[3], PWMChannel[3]);
}

void ADS1115Init(void) {
  ads.setAddr_ADS1115(ADS1115_IIC_ADDRESS0);    // 0x48
  ads.setGain(eGAIN_ONE);                       // 2/3x gain
  ads.setMode(eMODE_SINGLE);                    // single-shot mode
  ads.setRate(eRATE_128);                       // 128SPS (default)
  ads.setOSMode(eOSMODE_SINGLE);                // Set to start a single-conversion
  ads.init();
}

void readInputData(void) {
  DinState = getAllCoils(DinMcp);

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

void logAnalogOut(void) {
  Serial.println("");
  Serial.println("--> ********************************** <--");
  Serial.println("System Log : Analog Output -->");
  for (size_t i = 0; i < 4; i++) {
    Serial.printf("Channel %d : " ,i); Serial.println(AoutVoltage[i]);
  }
  Serial.println("--> ********************************** <--");
  Serial.println("");
}

void logAnalogIn(void) {
  Serial.println("");
  Serial.println("--> ********************************** <--");
  Serial.println("System Log : Analog Input -->");
  for (size_t i = 0; i < 4; i++) {
    Serial.printf("Channel %d : " ,i); Serial.println(AinVoltage[i]);
  }
  Serial.println("--> ********************************** <--");
  Serial.println("");
}

void logDigitalOut(void) {
  Serial.println("");
  Serial.println("--> ********************************** <--");
  Serial.println("System Log : Digital Output -->");
  for (size_t i = 0; i < 8; i++) {
    Serial.printf("Channel %d : " ,i); Serial.println(lastCoilState[i]);
  }
  Serial.println("--> ********************************** <--");
  Serial.println("");
}

void logDigitalIn(void) {
  Serial.println("");
  Serial.println("--> ********************************** <--");
  Serial.println("System Log : Digital Input -->");
  Serial.printf("Raw Data is : %d" ,DinState); 
  Serial.println("--> ********************************** <--");
  Serial.println("");
}

inline uint8_t bit_set(uint8_t number, uint8_t n) {
    return number | ((uint8_t)1 << n);
}

inline uint8_t bit_clear(uint8_t number, uint8_t n) {
    return number & ~((uint8_t)1 << n);
}

void WizReset(void) {
  Serial.print("Resetting Wiz W5500 Ethernet Board...  ");
  pinMode(RESET_P, OUTPUT);
  digitalWrite(RESET_P, HIGH);
  delay(250);
  digitalWrite(RESET_P, LOW);
  delay(50);
  digitalWrite(RESET_P, HIGH);
  delay(350);
  Serial.print("Done.\n");
}

void ethernetInit(void) {
  Ethernet.init(W5500_CS_PIN);
  
  WizReset();
  // Ethernet.setGatewayIP(localGateway);
  // Ethernet.setSubnetMask(localSubnet);
  // Ethernet.begin(mac ,staticIP ,localDNS ,localGateway ,localSubnet);

  // start the Ethernet connection:
  // line 104 of w5100.cpp changed to SPI.begin(14 ,12 ,13 ,15);
  if (Ethernet.begin(mac) == 0) {
    // No. DHCP did not work.
    Serial.print("Failed to configure Ethernet using DHCP\n");
    // Check for Ethernet hardware present
    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
      Serial.print("Ethernet shield was not found.  Sorry, can't run without hardware. :(\n");
    } else {
      if (Ethernet.linkStatus() == LinkOFF) {
        Serial.print("Ethernet cable is not connected.\n");
      }
    }
    while (1) {}  // Do nothing any more
  }
  // Ethernet.setLocalIP(staticIP);

  // print local IP address:
  localIP = Ethernet.localIP();
  Serial.printf("My IP address: %u.%u.%u.%u\n", localIP[0], localIP[1], localIP[2], localIP[3]);
}

// Modbus FC Callback
// Read_Coil 
ModbusMessage TCP_FC01(ModbusMessage request) {
  uint16_t startCoil;       
  uint16_t quantity;             
  ModbusMessage responsePkt;   

  // start reading from coil number "startCoil"
  request.get(2, startCoil);   
  // read the next "quantity" coils.
  request.get(4, quantity);     

  // create respond packet
  responsePkt.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(1));  

  uint8_t coilStatebin = 0b00000000;
  // read all coil state to config "coilState"
  for (size_t i = startCoil; i < (startCoil + quantity); i++) {
    if (coilState[i]) {
      coilStatebin = bit_set(coilStatebin ,i);
    } else {
      coilStatebin = bit_clear(coilStatebin ,i);
    }
  }

  // create respond packet
   responsePkt.add(coilStatebin);  

  return responsePkt;
}
// READ_DISCR_INPUT #
ModbusMessage TCP_FC02(ModbusMessage request) {
  uint16_t address;           // requested register address
  uint16_t words;             // requested number of registers
  ModbusMessage response;     // response message to be sent back

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
// READ_HOLD_REGISTER #
ModbusMessage TCP_FC03(ModbusMessage request) {
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
// READ_INPUT_REGISTER #
ModbusMessage TCP_FC04(ModbusMessage request) {
  uint16_t address;      
  uint16_t words;          
  ModbusMessage response;  


  request.get(2, address);
  request.get(4, words);


  response.add(request.getServerID(), request.getFunctionCode(), (uint8_t)(words * 2));

  for (uint16_t i = address; i < (address + words); i++) {
    response.add((uint16_t )AinVoltage[i]);
  }

  return response;
}
// WRITE_COIL #
ModbusMessage TCP_FC05(ModbusMessage request) {
  uint16_t address;
  uint16_t state;             
  ModbusMessage response; 

  // single coil address
  request.get(2, address);
  // single coil state
  request.get(4, state);

  // set value to set/save coil in loop
  coilState[address] = state;
  saveFlag[DoutStateFlag] = true;

  // create respond packet
  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(state);

  return response;
}
// WRITE_HOLD_REGISTER #
ModbusMessage TCP_FC06(ModbusMessage request) {
  uint16_t address;      
  uint16_t value;       
  ModbusMessage response;  

  request.get(2, address);
  request.get(4, value);

  AoutVoltage[address] = value;
  setAnalogOut(PWMChannel[address] ,AoutVoltage[address]);
  saveFlag[AoutVoltageFlag] = true;

  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(value);

  return response;
}
// WRITE_MULT_COILS #
ModbusMessage TCP_FC0F(ModbusMessage request) {
  uint16_t address;      
  uint16_t coilNum;      
  uint8_t state;            
  ModbusMessage response;   
  
  // start writing on relay number "address"
  request.get(2, address);
  // write the next "coilNum" coils.
  request.get(4, coilNum);
  // multi coil states
  request.get(7, state);

  // set value to set/save coils in loop
  uint8_t cnt = 0;
  for (uint8_t i = address; i < (address + coilNum); i++) {
    coilState[i] = (state & (1 << cnt));
    cnt++;
  }
  saveFlag[DoutStateFlag] = true;

  // create respond packet
  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(coilNum);

  return response;
}
// WRITE_MULT_REGISTERS 
ModbusMessage TCP_FC10(ModbusMessage request) {
  uint16_t address;        
  uint16_t regCount;        
  uint16_t value;           
  ModbusMessage response;  

  request.get(2, address);
  request.get(4, regCount);


  for (uint16_t i = address; i < (address + regCount); i++) {
    request.get((7 + ((i - address) * 2)), value);
    AoutVoltage[i] = value;
    setAnalogOut(PWMChannel[i] ,AoutVoltage[i]);
  }

  saveFlag[AoutVoltageFlag] = true;

  response.add(request.getServerID(), request.getFunctionCode());
  response.add(address);
  response.add(regCount);

  return response;
}
// REPORT_SERVER_ID_SERIAL 
ModbusMessage RTU_FC11(ModbusMessage request) {
  ModbusMessage response;  
  response.add(request.getServerID(), request.getFunctionCode());
  response.add(0xF);
  response.add(modbusId);

  return response;
}

void modbusInit(void) {
  MB.registerWorker(0x01, READ_COIL, &TCP_FC01);
  MB.registerWorker(0x01, READ_DISCR_INPUT, &TCP_FC02);
  MB.registerWorker(0x01, READ_HOLD_REGISTER, &TCP_FC03);
  MB.registerWorker(0x01, READ_INPUT_REGISTER, &TCP_FC04);
  MB.registerWorker(0x01, WRITE_COIL, &TCP_FC05);
  MB.registerWorker(0x01, WRITE_HOLD_REGISTER, &TCP_FC06);
  MB.registerWorker(0x01, WRITE_MULT_COILS, &TCP_FC0F);
  MB.registerWorker(0x01, WRITE_MULT_REGISTERS, &TCP_FC10);
  MB.start(PORT, 2, 5000 ,0);

  // MB.registerWorker(0x01, REPORT_SERVER_ID_SERIAL, &RTU_FC11);
}

class LoopTask : public Task {
protected:
    void loop()  {
          if ((millis() - startLoopTime) > loopDelay) {
          readInputData();

          uint8_t address = readModbusAdd();
          if (modbusAdd != address) {
            modbusAdd = address;
            Serial.println(modbusAdd);
            saveFlag[ModbusAddFlag] = true;
          }

          saveParameters();

          startLoopTime = millis();
        }

        for (size_t i = 0; i < 8; i++) {
          if (lastCoilState[i] != coilState[i]) {
            setCoil(DoutMcp ,(7 - i) ,coilState[i]);
            lastCoilState[i] = coilState[i];
          }
        }
    }
} loop_task;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  Serial.println("__ OK __");

  NVS.begin("SM1001", false);
  pinInit();
  mcpInit();
  ADS1115Init();
  ethernetInit();
  modbusInit();
  restoreParameters();

  startLoopTime = millis();

  Scheduler.start(&loop_task);
  Scheduler.begin();
}

void loop() {

}

