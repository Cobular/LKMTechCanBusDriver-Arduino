/*
 * Adafruit MCP2515 FeatherWing CAN Receiver Example
 */

#include <Adafruit_MCP2515.h>
#include "controlCommands.h"

#ifdef ESP8266
   #define CS_PIN    2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S3)
   #define CS_PIN    14
#elif defined(TEENSYDUINO)
   #define CS_PIN    8
#elif defined(ARDUINO_STM32_FEATHER)
   #define CS_PIN    PC5
#elif defined(ARDUINO_NRF52832_FEATHER)  /* BSP 0.6.5 and higher! */
   #define CS_PIN    27
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define CS_PIN    P3_2
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040)
   #define CS_PIN    7
#elif defined(ARDUINO_ADAFRUIT_FEATHER_RP2040_CAN)
   #define CS_PIN    PIN_CAN_CS
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W) // PiCowbell CAN Bus
   #define CS_PIN    20
#else
    // Anything else, defaults!
   #define CS_PIN    5
#endif

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)
#define SPEED_LIMIT ((uint16_t)240)

Adafruit_MCP2515 mcp(CS_PIN);
LKMTechMotorController motorController;

String inputString = "";      // a String to hold incoming data
bool stringComplete = false;  // whether the string is complete

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10);

  Serial.println("MCP2515 Receiver test!");

  if (!mcp.begin(CAN_BAUDRATE)) {
    Serial.println("Error initializing MCP2515.");
    while(1) delay(10);
  }
  Serial.println("MCP2515 chip found");

  motorController.initalize(&mcp);

  inputString.reserve(200);
}

int currentId = 1;
uint8_t dataBuffer[8];


void initialResponseParser(uint8_t *arr) {
  Serial.println("No packet handler connected");
}

ResponseParser parser = &initialResponseParser;

bool hasSwept = false;
bool firstPass = true;

void loop() {
  if (firstPass) {
    Serial.println("Sweeping all IDs");
    firstPass = false;
  }
  if (!hasSwept && currentId < 40) {
    motorController.sendMotorOff(currentId);
    currentId++;
  } else if (!hasSwept) {
    hasSwept = true;
    Serial.println("Finished sweep");
  }

  if (stringComplete) {
    Serial.print("got string ");
    Serial.print(inputString);
    if (inputString == "STOP\n") {
        Serial.println("stopping...");
        parser = motorController.sendMotorStop(1);
    } else if (inputString == "ON\n") {
        Serial.println("Turning motor on");
        parser = motorController.sendMotorOn(1);
    } else if (inputString == "OFF\n") {
      Serial.println("Turning motor off");
      parser = motorController.sendMotorOff(1);
    } else if (inputString.startsWith("TORQUE")) {
      int power = 0;
      sscanf(inputString.c_str(), "TORQUE%02d", &power);
      Serial.print("Running motor with set torque");
      Serial.println(power);
      parser = motorController.sendTorqueClosedLoopControl(1, power);
    } else if (inputString.startsWith("SPEED")) {
      int speed = 0;
      sscanf(inputString.c_str(), "SPEED%08d", &speed);
      Serial.print("Running motor with set speed");
      Serial.println(speed);
      parser = motorController.sendSpeedClosedLoopControl(1, speed);
    } else if (inputString.startsWith("ABS_ANGLE_DEG")) {
      int angle = 0;
      sscanf(inputString.c_str(), "ABS_ANGLE_DEG%08d", &angle);
      Serial.print("Setting absolute angle");
      Serial.println(angle);
      parser = motorController.sendAbsoluteAngleControlSpeedLimit(1, angle * 1000, SPEED_LIMIT);
    } else if (inputString.startsWith("REL_ANGLE_CW")) {
      int angle = 0;
      sscanf(inputString.c_str(), "REL_ANGLE_CW%08d", &angle);
      Serial.print("Setting relative angle");
      Serial.println(angle);
      parser = motorController.sendRelativeAngleControlSpeedLimit(1, 0x00, angle, SPEED_LIMIT);
    } else if (inputString.startsWith("REL_ANGLE_CCW")) {
      int angle = 0;
      sscanf(inputString.c_str(), "REL_ANGLE_CCW%08d", &angle);
      Serial.print("Setting relative angle");
      Serial.println(angle);
      parser = motorController.sendRelativeAngleControlSpeedLimit(1, 0x01, angle, SPEED_LIMIT);
    } else {
      Serial.print("Did not understand command ");
      Serial.println(inputString);
    }
    stringComplete = false;
    inputString = "";
  }

  // try to parse packet
  int packetSize = mcp.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received ");

    if (mcp.packetExtended()) {
      Serial.print("extended ");
    }

    if (mcp.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(mcp.packetId(), HEX);

    if (mcp.packetId() > 0x140) {
      Serial.print(" for a motor with ID");
      Serial.print(mcp.packetId() - 0x140, DEC);
    }

    if (mcp.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(mcp.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      int i = 0;
      while (mcp.available()) {
        dataBuffer[i] = mcp.read();
        Serial.print(dataBuffer[i], HEX);
        i++;
      }
      Serial.println();

      parser(dataBuffer);
    }

    Serial.println();
  }
}

void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}