// WIFI_AP settings.
const char* AP_SSID = "ESP32_DEV";
const char* AP_PWD  = "12345678";

// WIFI_STA settings.
const char* STA_SSID = "OnePlus 8";
const char* STA_PWD  = "40963840";

// the MAC address of the device you want to ctrl.
uint8_t broadcastAddress[] = {0x08, 0x3A, 0xF2, 0x93, 0x5F, 0xA8};
// uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};


typedef struct struct_message {
  int ID_send;
  int POS_send;
  int Spd_send;
} struct_message;

// Create a struct_message called myData
struct_message myData;


// set the default role here.
// 0 as normal mode.
// 1 as leader, ctrl other device via ESP-NOW.
// 2 as follower, can be controled via ESP-NOW.
#define DEFAULT_ROLE 0


// set the default wifi mode here.
// 1 as [AP] mode, it will not connect other wifi.
// 2 as [STA] mode, it will connect to know wifi.
#define DEFAULT_WIFI_MODE 1

// the uart used to control servos.
// GPIO 18 - S_RXD, GPIO 19 - S_TXD, as default.
#define S_RXD 18
#define S_TXD 19

// the IIC used to control OLED screen.
// GPIO 21 - S_SDA, GPIO 22 - S_SCL, as default.
#define S_SCL 22
#define S_SDA 21

// the GPIO used to control RGB LEDs.
// GPIO 23, as default.
#define RGB_LED   23
#define NUMPIXELS 10

// set the max ID.
int MAX_ID = 20;

int servoID = 1; // ID of the servo motor to control
int speed = 800; // Speed
int position = 1000; // Initial position set to 90 degrees
int direction = -1; // Initial direction set to move towards 45 degrees

// modeSelected.
// set the SERIAL_FORWARDING as true to control the servos with USB.
bool SERIAL_FORWARDING = false;

// OLED Screen Dispaly.
// Row1: MAC address.
// Row2: VCC --- IP address.
// Row3: MODE:Leader/Follower  [AP]/[STA][RSSI]
//       DEFAULT_ROLE: 1-Leader(L)/ 2-Follower(F).
//       DEFAULT_WIFI_MODE: 1-[AP]/ 2-[STA][RSSI] / 3-[TRY:SSID].
//       (no matter what wifi mode you select, you can always ctrl it via ESP-NOW.)
// Row4: the position of servo 1, 2 and 3.
String MAC_ADDRESS;
IPAddress IP_ADDRESS;
byte   SERVO_NUMBER;
byte   DEV_ROLE;
byte   WIFI_MODE;
int    WIFI_RSSI;

// set the interval of the threading.
#define threadingInterval 600
#define clientInterval    10

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif

#include "RGB_CTRL.h"
#include "STSCTRL.h"
#include "CONNECT.h"
#include "BOARD_DEV.h"

float readEncoderAngle()
{
    // 1. 엔코더의 출력을 읽습니다.
    int encoder_output = posRead[listID[activeNumInList]];

    // 2. 출력 값을 각도로 변환합니다.
    
    float angle = map(encoder_output, 0, 4095, 0, 360);

    // 3. 각도를 반환합니다.
    return angle;
}


void setup() {
  Serial.begin(115200);
  while(!Serial) {}

  InitRGB();

  espNowInit();

  getMAC();
  
  boardDevInit();

  RGBcolor(0, 64, 255);

  servoInit();

  wifiInit();

  webServerSetup();

  RGBoff();

  delay(1000);
  pingAll(true);

  threadInit();
}

bool isFirstExecution = true;
void loop() {
  if (isFirstExecution) {
        // Execute only on the first iteration
        st.WritePosEx(servoID, position, speed, 0);
        isFirstExecution = false; // Set the flag to false after the first execution
    }
  int encoder_output = posRead[listID[activeNumInList]];

  float angle = map(encoder_output, 63, 4159, -180, 180);


  Serial.print("angle:");
  Serial.println(angle);


    // Reverse direction at the boundaries of 45 and 135 degrees
    if (posRead[listID[activeNumInList]] >= 2560) {
        st.WritePosEx(servoID, 1486, speed, 0); // Set position to 135 degrees to prevent overshooting
    } else if (posRead[listID[activeNumInList]] <= 1536) {
        st.WritePosEx(servoID, 2610, speed, 0); // Set position to 45 degrees to prevent undershooting
    }

  delay(10);
}
