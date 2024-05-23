#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

// encoder pin
const int encoderPinA = 8;
const int encoderPinB = 9;
unsigned long timeVal = 0;  // 현재 시간 값 저장 변수
unsigned long previousVal = 0; // 이전 시간 값 저장 변수

int encoderPos = 0;
int preM = 0;
int mS = 0;
float speed_val = 0;

void doEncoderA() {
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? 1 : -1;
}

void doEncoderB() {
  encoderPos += (digitalRead(encoderPinA) == digitalRead(encoderPinB)) ? -1 : 1;
}

void speedcal() {
  preM = mS;
  mS = encoderPos;
  speed_val = (mS - preM) / 4.0 * 0.0117 * 3.0;
}

void setup() {
  pinMode(encoderPinA, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), doEncoderA, CHANGE);

  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), doEncoderB, CHANGE);

  Serial.begin(115200);
}

void loop() {
  timeVal = millis();
  if (timeVal - previousVal >= 1000) {
    speedcal();
    previousVal = timeVal;

    // 속도 값 시리얼 모니터에 출력
    Serial.print("speed:");
    Serial.print(speed_val);
  }
}
