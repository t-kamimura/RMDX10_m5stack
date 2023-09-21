/*
  Please add MCP_CAN_LIB to your library first........
  MCP_CAN_LIB file in M5stack lib examples -> modules -> COMMU -> MCP_CAN_lib.rar
*/

#include <M5Stack.h>
#include <mcp_can_m5.h>
// #include "m5_logo.h"

/**
 * variable for loop
 */

byte data[8] = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/**
 * consts & variable for CAN
 */
const uint16_t MOTOR_ADDRESS = 0x141; //0x140 + ID(1~32)

long unsigned int rxId; // デバイスのアドレス読み込み結果
unsigned char len = 0;  // 受け取ったデータの長さ
unsigned char rxBuf[8]; // 読み込んだデータを蓄えるバッファ
char msgString[128];    // Array to store serial string

#define CAN0_INT 15  // Set INT to pin 2
MCP_CAN_M5 CAN0(12); // Set CS to pin 10

void init_can();
void read_can();
void write_can();

void setup()
{
  M5.begin();
  M5.Power.begin();
  Serial.begin(115200);
  // Serial2.begin(115200, SERIAL_8N1, 16, 17);

  delay(500);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(GREEN, BLACK);
  M5.Lcd.setTextSize(2);

  init_can();
  Serial.println("Test CAN...");
}

void loop()
{
  if (M5.BtnA.wasPressed())
  {
    M5.Lcd.clear();
    M5.Lcd.printf("CAN Test A!\n");
    // M5.Lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
    init_can();
    Serial.println("Test CAN...");
  }
  write_can();
  delay(1);
  read_can();
  delay(1);
  M5.update();
}

void init_can()
{
  M5.Lcd.setTextSize(1);
  M5.Lcd.setCursor(0, 10);
  //   M5.Lcd.pushImage(0, 0, 320, 240, (uint16_t *)gImage_logoM5);
  delay(500);

  M5.Lcd.printf("CAN Test!\n");

  // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    Serial.println("MCP2515 Initialized Successfully!");
  else
    Serial.println("Error Initializing MCP2515...");

  CAN0.setMode(MCP_NORMAL); // Set operation mode to normal so the MCP2515 sends acks to received data.

  pinMode(CAN0_INT, INPUT); // Configuring pin for /INT input

  Serial.println("MCP2515 Library Receive Example...");
}

void read_can()
{
  if (!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)

    if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
    {
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d \n Data:", (rxId & 0x1FFFFFFF), len);
    }
    else
    {
      sprintf(msgString, "Standard ID: 0x%.3lX  DLC: %1d \n Data:", rxId, len);
    }

    Serial.print(msgString);
    M5.Lcd.printf(msgString);

    if ((rxId & 0x40000000) == 0x40000000)
    { // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    }
    else
    {
      for (byte i = 0; i < len; i++)
      {
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
        M5.Lcd.printf(msgString);
      }
    }

    M5.Lcd.printf("\n");
    Serial.println();
  }
}

void write_can()
{
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  byte sndStat = CAN0.sendMsgBuf(MOTOR_ADDRESS, 0, 8, data);
  if (sndStat == CAN_OK)
  {
    Serial.println("Message Sent Successfully!");
    M5.Lcd.printf("Message Sent Successfully!\n");
  }
  else
  {
    Serial.println("Error Sending Message...");
    M5.Lcd.printf("Error Sending Message...\n");
  }
  delay(200); // send data per 200ms
}
