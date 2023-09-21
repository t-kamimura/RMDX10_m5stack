/* 電流制御テスト */
#include <M5Stack.h>
#include <mcp_can_m5.h>
#include <SPI.h>

/*SAMD core*/
#ifdef ARDUINO_SAMD_VARIANT_COMPLIANCE
#define SERIAL serialUSB
#else
#define SERIAL Serial
#endif

#define BAUDRATE 115200 // シリアル通信がボトルネックにならないよう，速めに設定しておく
#define LOOPTIME 10      //[ms]
#define ENDTIME 10000   //[ms]
#define TEXTSIZE 2
#define KP 1.0
#define KD 0.1
#define TGT_ANGLE 0

unsigned char len = 0;
unsigned char cmd_buf[8], reply_buf[8];
long unsigned int rxId;
unsigned char rxBuf[8];
char msgString[128];
unsigned long timer[3];
byte pos_byte[4];

const uint16_t MOTOR_ADDRESS = 0x141; // 0x140 + ID(1~32)
const int SPI_CS_PIN = 12;

#define CAN0_INT 15          // Set INT to pin 2
MCP_CAN_M5 CAN0(SPI_CS_PIN); // Set CS to pin 10

int32_t present_pos = 0; // raw data
double angle = 0;        // horn angle [deg]
double pos_buf = 0;
double vel = 0;
int16_t tgt_cur = 0;

void init_can();
void write_can();
void read_can();

void setup()
{
    M5.begin();
    M5.Power.begin();
    dacWrite(25, 0); // Speaker OFF
    Serial.begin(BAUDRATE);
    delay(1000);
    M5.Lcd.fillScreen(BLACK);
    M5.Lcd.setTextColor(GREEN, BLACK);
    M5.Lcd.setTextSize(TEXTSIZE);

    init_can();
    Serial.println("Test current control...\n");

    // read multi turn angle
    cmd_buf[0] = 0x92;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = 0x00;
    cmd_buf[5] = 0x00;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;
    write_can();
    read_can();

    present_pos = 0;
    present_pos = reply_buf[4] + (reply_buf[5] << 8) + (reply_buf[6] << 16) + (reply_buf[7] << 24);
    angle = present_pos * 0.01;
    pos_buf = angle;

    SERIAL.print("Initial angle: ");
    SERIAL.println(angle);
    SERIAL.println("Please wait...");
    delay(5000);
    
    timer[0] = millis();
}

void loop()
{
    while (millis() - timer[0] < ENDTIME)
    {
        timer[1] = millis();
        tgt_cur = -KP * (angle - TGT_ANGLE) - KD * vel; //[0.01A/LSB]

        // current control command
        cmd_buf[0] = 0xA1;
        cmd_buf[1] = 0x00;
        cmd_buf[2] = 0x00;
        cmd_buf[3] = 0x00;
        cmd_buf[4] = tgt_cur & 0xFF;
        cmd_buf[5] = (tgt_cur >> 8) & 0xFF;
        cmd_buf[6] = 0x00;
        cmd_buf[7] = 0x00;
        write_can();
        read_can();

        M5.update();

        // print
        SERIAL.print("TIM: ");
        SERIAL.print(timer[1] - timer[0]);
        SERIAL.print(" TGT: ");
        SERIAL.print(tgt_cur);

        for (int i = 0; i++; i < 8)
        {
            reply_buf[i] = 0x00;
        }

        // read multi turn angle
        cmd_buf[0] = 0x92;
        cmd_buf[1] = 0x00;
        cmd_buf[2] = 0x00;
        cmd_buf[3] = 0x00;
        cmd_buf[4] = 0x00;
        cmd_buf[5] = 0x00;
        cmd_buf[6] = 0x00;
        cmd_buf[7] = 0x00;
        write_can();
        read_can();

        M5.update();

        pos_buf = angle;
        present_pos = 0;
        if (reply_buf[0] == 0x92)
        {
            present_pos = reply_buf[4] + (reply_buf[5] << 8) + (reply_buf[6] << 16) + (reply_buf[7] << 24);
        }
        else if (reply_buf[0] == 0xA1)
        {
            Serial.print(" Error! PosCmd Reply...");
        }
        else
        {
            Serial.print(" Read Error!");
        }
        angle = present_pos * 0.01;
        vel = (angle - pos_buf) / (LOOPTIME * 0.01);

        Serial.print(" POS: ");
        Serial.print(angle);
        Serial.print(" VEL: ");
        Serial.print(vel);
        Serial.println();

        timer[2] = millis() - timer[1];
        if (timer[2] < LOOPTIME)
        {
            delay(LOOPTIME - timer[2]);
        }
        else
        {
            Serial.print("TIME SHORTAGE: ");
            Serial.println(timer[2] - LOOPTIME);
        }
    }

    // stop command
    tgt_cur = 0;
    cmd_buf[0] = 0xA1;
    cmd_buf[1] = 0x00;
    cmd_buf[2] = 0x00;
    cmd_buf[3] = 0x00;
    cmd_buf[4] = tgt_cur & 0xFF;
    cmd_buf[5] = (tgt_cur >> 8) & 0xFF;
    cmd_buf[6] = 0x00;
    cmd_buf[7] = 0x00;
    write_can();
    read_can();
    delay(500);
    SERIAL.println("Program finish!");
    M5.Lcd.setCursor(0, 160);
    M5.Lcd.printf("Program finish!");
    while (true)
    {
        delay(100);
    }
}

void init_can()
{
    M5.Lcd.setTextSize(TEXTSIZE);
    M5.Lcd.setCursor(0, 10);
    M5.Lcd.printf("Current Control Test!\n");
    M5.Lcd.setCursor(0, 40);
    M5.Lcd.printf("KP: %4.2f", KP);
    M5.Lcd.setCursor(0, 70);
    M5.Lcd.printf("KD: %4.2f", KD);

    // Initialize MCP2515 running at 16MHz with a baudrate of 500kb/s and the masks and filters disabled.
    if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
    {
        Serial.println("MCP2515 Initialized Successfully!");
    }
    else
    {
        Serial.println("Error Initializing MCP2515...");
    }

    CAN0.setMode(MCP_NORMAL); // Change to normal mode to allow messages to be transmitted
}

void write_can()
{
    // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
    byte sndStat = CAN0.sendMsgBuf(MOTOR_ADDRESS, 0, 8, cmd_buf);
    if (sndStat == CAN_OK)
    {
        // Serial.println("Message Sent Successfully!");
        // M5.Lcd.printf("Message Sent Successfully!\n");
    }
    else
    {
        Serial.println("Error Sending Message...");
        // M5.Lcd.printf("Error Sending Message...\n");
    }
    delay(2);
}

void read_can()
{
    if (!digitalRead(CAN0_INT)) // If CAN0_INT pin is low, read receive buffer
    {
        CAN0.readMsgBuf(&rxId, &len, reply_buf); // Read data: len = data length, buf = data byte(s)

        // if ((rxId & 0x80000000) == 0x80000000) // Determine if ID is standard (11 bits) or extended (29 bits)
        // {
        //     sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d \n Data:", (rxId & 0x1FFFFFFF), len);
        // }
        // else
        // {
        //     sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d \n Data:", rxId, len);
        // }

        // Serial.print(msgString);
        // // M5.Lcd.printf(msgString);

        // if ((rxId & 0x40000000) == 0x40000000)
        // { // Determine if message is a remote request frame.
        //     sprintf(msgString, " REMOTE REQUEST FRAME");
        //     Serial.print(msgString);
        // }
        // else
        // {
        //     for (byte i = 0; i < len; i++)
        //     {
        //         sprintf(msgString, " 0x%.2X", reply_buf[i]);
        //         Serial.print(msgString);
        //         // M5.Lcd.printf(msgString);
        //     }
        // }

        // M5.Lcd.printf("\n");
        Serial.println();
    }
    delay(2);
}