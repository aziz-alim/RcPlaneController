/*
 * RC Plane Controller - Alici (Receiver)
 * Arduino Nano + NRF24L01 + MPU6050 + ESC + 4 Servo
 *
 * NRF24L01 (tek basina) -> Arduino Nano:
 *   VCC  -> 3.3V  (VCC-GND arasi 10-100uF kondansator SART)
 *   GND  -> GND
 *   CE   -> D9
 *   CSN  -> D10
 *   SCK  -> D13
 *   MOSI -> D11
 *   MISO -> D12
 *
 * NRF24L01 (adaptor shield ile) -> Arduino Nano:
 *   VCC  -> 5V    (shield uzerinde 5V->3.3V regulator var)
 *   GND  -> GND
 *   CE   -> D9
 *   CSN  -> D10
 *   SCK  -> D13
 *   MOSI -> D11
 *   MISO -> D12
 *
 * MPU6050 -> Arduino Nano:
 *   VCC -> 5V
 *   GND -> GND
 *   SCL -> A5
 *   SDA -> A4
 *
 * Cikislar (Dual Aileron):
 *   D3  -> ESC (Gaz/Throttle)
 *   D5  -> Sag Kanat Aileron  (aileron ile dogru orantili)
 *   D4  -> Sol Kanat Aileron  (aileron ters, otomatik mix)
 *   D6  -> Elevator (Kuyruk Yatay)
 *   D7  -> Rudder   (Kuyruk Dikey)
 *
 * ESC -> Motor -> Pil baglantisi:
 *   ESC 3x mavi kablo  -> Brushless motor 3 kablosu
 *   ESC kirmizi/siyah   -> LiPo pil
 *   ESC sinyal kablosu  -> Arduino D3
 *   ESC GND             -> Arduino GND
 *   ESC 5V (kirmizi)    -> USB varsa BOS, yoksa VIN
 */

#include <SPI.h>
#include <RF24.h>
#include <Servo.h>
#include <Wire.h>
#include <MPU6050_light.h>

RF24 radio(9, 10);

const byte rxAddress[6] = "00001";

struct __attribute__((packed)) ControlPacket {
  uint16_t throttle;
  uint16_t aileron;
  uint16_t elevator;
  uint16_t rudder;
  uint16_t aux;
};

ControlPacket packet;

Servo esc;
Servo servoRightAileron;
Servo servoLeftAileron;
Servo servoElevator;
Servo servoRudder;

const int PIN_ESC            = 3;
const int PIN_RIGHT_AILERON  = 5;
const int PIN_LEFT_AILERON   = 4;
const int PIN_ELEVATOR       = 6;
const int PIN_RUDDER         = 7;
const int PIN_LED            = LED_BUILTIN;

// Seri porta durum yazdirma (tezgah testi icin). Ucusta false birak.
const bool DEBUG_PRINT = true;
const unsigned long DEBUG_PERIOD_MS = 250;
unsigned long lastDebugPrint = 0;

// Kontrol yuzeyi yumusatma: her ~5 ms'de en fazla bu kadar us.
// 40 us/5 ms ~ tam stroke (1000us) icin ~125 ms.
const uint16_t SURFACE_STEP  = 40;
const uint16_t THROTTLE_STEP = 10;   // 1000->2000 ~ 0.5 sn
const unsigned long STEP_PERIOD_MS = 5;
unsigned long lastStep = 0;

// Gaz kilidi: baglanti kurulduktan (veya koptuktan) sonra gaz ancak
// once dusuk konumda (<= THROTTLE_ARM_MAX) gorulurse acilir.
const uint16_t THROTTLE_ARM_MAX = 1050;
bool throttleArmed = false;

const unsigned long FAILSAFE_MS = 1500;
unsigned long lastPacketTime = 0;
unsigned long lastResetAttempt = 0;
bool armed = false;
bool linkLost = false;
int resetAttempts = 0;

uint16_t currentThrottle = 1000;
uint16_t currentAileron = 1500;
uint16_t currentElevator = 1500;
uint16_t currentRudder = 1500;

uint16_t targetThrottle = 1000;
uint16_t targetAileron = 1500;
uint16_t targetElevator = 1500;
uint16_t targetRudder = 1500;

uint16_t stickAileron = 1500;
uint16_t stickElevator = 1500;

MPU6050 mpu(Wire);
unsigned long lastMpuUpdate = 0;
float roll = 0, pitch = 0, yaw = 0;
bool stabEnabled = false;

// ── Kanal kurali ("pilot kurali", control.py ile ayni) ──
//   aileron  > 1500 = saga yatis
//   elevator > 1500 = burun yukari
//   rudder   > 1500 = burun saga
// Bir yuzey ters gidiyorsa ilgili REV_* sabitini degistir.
// Varsayilanlar eski control.py davranisini korur; tezgahta dogrula.
const bool REV_AILERON      = true;   // sag kanat aileron (D5)
const bool REV_AILERON_LEFT = false;  // sol kanat, sag kanada GORE ters mi (D4)
const bool REV_ELEVATOR     = true;
const bool REV_RUDDER       = false;

// Mekanik merkez duzeltmesi (us). Kumandadaki trimden ayri; kalici ayar.
const int SUBTRIM_AIL_R = 0;
const int SUBTRIM_AIL_L = 0;
const int SUBTRIM_ELE   = 0;
const int SUBTRIM_RUD   = 0;

// ── Stabilizasyon ──
// MPU6050: cip yuzu yukari, X burna, Y sol kanada.
//   roll  = getAngleX()  : + = sag kanat asagi
//   pitch = getAngleY()  : + = burun ASAGI (burun yukari = -)
// Kazanclar us/derece ve us/(derece/sn); tezgahta ayarlanacak.
const float MAX_TARGET_ANGLE = 45.0;
const float ROLL_KP  = 1.5;
const float ROLL_KD  = 0.5;
const float PITCH_KP = 1.5;
const float PITCH_KD = 0.5;

void resetRadio() {
  radio.stopListening();
  radio.begin();
  radio.openReadingPipe(0, rxAddress);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPayloadSize(sizeof(packet));
  radio.setAutoAck(false);
  radio.startListening();
}

void centerServos() {
  stabEnabled = false;
  throttleArmed = false;
  stickAileron = 1500;
  stickElevator = 1500;
  targetThrottle = 1000;
  targetAileron = 1500;
  targetElevator = 1500;
  targetRudder = 1500;
  currentThrottle = 1000;
  currentAileron = 1500;
  currentElevator = 1500;
  currentRudder = 1500;
  esc.writeMicroseconds(1000);
  servoRightAileron.writeMicroseconds(1500 + SUBTRIM_AIL_R);
  servoLeftAileron.writeMicroseconds(1500 + SUBTRIM_AIL_L);
  servoElevator.writeMicroseconds(1500 + SUBTRIM_ELE);
  servoRudder.writeMicroseconds(1500 + SUBTRIM_RUD);
}

uint16_t smoothMove(uint16_t current, uint16_t target, uint16_t maxStep) {
  if (current < target) {
    return min(target, current + maxStep);
  } else if (current > target) {
    return max(target, current - maxStep);
  }
  return current;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  Wire.begin();
  byte mpuStatus = mpu.begin();
  if (mpuStatus != 0) {
    Serial.print(F("MPU6050 bulunamadi! Kod: "));
    Serial.println(mpuStatus);
    while (1) {
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
      delay(200);
    }
  }
  Serial.println(F("MPU6050 kalibre ediliyor... ucagi sabit tutun"));
  digitalWrite(PIN_LED, HIGH);
  delay(1000);
  mpu.calcOffsets();
  digitalWrite(PIN_LED, LOW);
  Serial.println(F("MPU6050 hazir"));

  if (!radio.begin()) {
    Serial.println(F("NRF24L01 baslatilamadi!"));
    while (1);
  }

  radio.openReadingPipe(0, rxAddress);
  radio.setPALevel(RF24_PA_LOW);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setPayloadSize(sizeof(packet));
  radio.setAutoAck(false);
  radio.startListening();

  esc.attach(PIN_ESC, 1000, 2000);
  servoRightAileron.attach(PIN_RIGHT_AILERON);
  servoLeftAileron.attach(PIN_LEFT_AILERON);
  servoElevator.attach(PIN_ELEVATOR);
  servoRudder.attach(PIN_RUDDER);

  Serial.println(F("ESC arm ediliyor... 3 sn"));
  esc.writeMicroseconds(1000);
  servoRightAileron.writeMicroseconds(1500 + SUBTRIM_AIL_R);
  servoLeftAileron.writeMicroseconds(1500 + SUBTRIM_AIL_L);
  servoElevator.writeMicroseconds(1500 + SUBTRIM_ELE);
  servoRudder.writeMicroseconds(1500 + SUBTRIM_RUD);
  delay(3000);

  armed = true;
  Serial.println(F("=== RC Alici Hazir ==="));
  Serial.println(F("D3=ESC D5=SagAil D4=SolAil D6=Elev D7=Rudd"));
  Serial.println(F("AUX>1500 = Stabilizasyon AKTIF"));
  Serial.println();
}

// Pilot kuralindaki degeri servo darbesine cevir (yon + subtrim)
uint16_t toServo(uint16_t cmd, bool reverse, int subtrim) {
  int d = (int)cmd - 1500;
  if (reverse) d = -d;
  return constrain(1500 + d + subtrim, 1000, 2000);
}

uint16_t clampCh(uint16_t v) {
  return constrain(v, 1000, 2000);
}

void loop() {
  unsigned long now = millis();

  if (radio.available()) {
    radio.read(&packet, sizeof(packet));
    lastPacketTime = now;

    if (linkLost) {
      linkLost = false;
      resetAttempts = 0;
      digitalWrite(PIN_LED, HIGH);
      Serial.println(F("++ Baglanti yeniden kuruldu (gaz kilitli, once gazi sifirla)"));
    }

    uint16_t thr = clampCh(packet.throttle);
    if (!throttleArmed && thr <= THROTTLE_ARM_MAX) {
      throttleArmed = true;
      Serial.println(F("++ Gaz kilidi acildi"));
    }

    targetThrottle = throttleArmed ? thr : 1000;
    stickAileron   = clampCh(packet.aileron);
    stickElevator  = clampCh(packet.elevator);
    targetRudder   = clampCh(packet.rudder);
    stabEnabled    = (packet.aux > 1500);

    if (!stabEnabled) {
      targetAileron  = stickAileron;
      targetElevator = stickElevator;
    }
  }

  mpu.update();
  if (now - lastMpuUpdate > 10) {
    roll  = mpu.getAngleX();
    pitch = mpu.getAngleY();
    yaw   = mpu.getAngleZ();
    lastMpuUpdate = now;
  }

  if (stabEnabled && armed && !linkLost) {
    // Hedef acilar pilot kuralinda: + = saga yatis / burun yukari
    float targetRoll  = ((float)((int)stickAileron  - 1500) / 500.0) * MAX_TARGET_ANGLE;
    float targetPitch = ((float)((int)stickElevator - 1500) / 500.0) * MAX_TARGET_ANGLE;

    float rollNow   = roll;               // + = saga yatik
    float rollRate  = mpu.getGyroX();     // + = saga donuyor
    float pitchUp   = -pitch;             // + = burun yukari
    float pitchRate = -mpu.getGyroY();    // + = burun kalkiyor

    // Cikti da pilot kuralinda: + = saga yatis / burun yukari komutu
    float rollCorr  = ROLL_KP  * (targetRoll  - rollNow) - ROLL_KD  * rollRate;
    float pitchCorr = PITCH_KP * (targetPitch - pitchUp) - PITCH_KD * pitchRate;

    targetAileron  = constrain(1500 + (int)rollCorr,  1000, 2000);
    targetElevator = constrain(1500 + (int)pitchCorr, 1000, 2000);
  }

  if (now - lastStep >= STEP_PERIOD_MS) {
    lastStep = now;
    currentThrottle = smoothMove(currentThrottle, targetThrottle, THROTTLE_STEP);
    currentAileron  = smoothMove(currentAileron,  targetAileron,  SURFACE_STEP);
    currentElevator = smoothMove(currentElevator, targetElevator, SURFACE_STEP);
    currentRudder   = smoothMove(currentRudder,   targetRudder,   SURFACE_STEP);

    esc.writeMicroseconds(currentThrottle);
    servoRightAileron.writeMicroseconds(toServo(currentAileron, REV_AILERON, SUBTRIM_AIL_R));
    servoLeftAileron.writeMicroseconds(toServo(currentAileron, REV_AILERON != REV_AILERON_LEFT, SUBTRIM_AIL_L));
    servoElevator.writeMicroseconds(toServo(currentElevator, REV_ELEVATOR, SUBTRIM_ELE));
    servoRudder.writeMicroseconds(toServo(currentRudder, REV_RUDDER, SUBTRIM_RUD));
  }

  // Failsafe: sinyal yoksa gazi kes, yuzeyleri ortala, radyoyu periyodik resetle
  if (armed && now - lastPacketTime > FAILSAFE_MS) {
    if (!linkLost) {
      linkLost = true;
      centerServos();
      lastResetAttempt = now;
      Serial.println(F("!! Sinyal kesildi - gaz kesildi, servolar merkezlendi !!"));
    }

    static unsigned long lastBlink = 0;
    if (now - lastBlink > 200) {
      lastBlink = now;
      digitalWrite(PIN_LED, !digitalRead(PIN_LED));
    }

    if (now - lastResetAttempt > 5000) {
      if (resetAttempts < 3) {
        Serial.print(F("!! NRF yeniden baslatiliyor ("));
        Serial.print(resetAttempts + 1);
        Serial.println(F("/3)"));
        resetAttempts++;
      }
      resetRadio();
      lastResetAttempt = now;
    }
  }

  if (DEBUG_PRINT && now - lastDebugPrint > DEBUG_PERIOD_MS) {
    lastDebugPrint = now;
    Serial.print(F("T:"));  Serial.print(currentThrottle);
    Serial.print(F(" A:")); Serial.print(currentAileron);
    Serial.print(F(" E:")); Serial.print(currentElevator);
    Serial.print(F(" R:")); Serial.print(currentRudder);
    Serial.print(F(" STAB:")); Serial.print(stabEnabled ? 'A' : 'P');
    Serial.print(F(" GAZ:")); Serial.print(throttleArmed ? F("ACIK") : F("KILITLI"));
    Serial.print(F(" LINK:")); Serial.print(linkLost ? F("YOK") : F("OK"));
    Serial.print(F(" roll:")); Serial.print(roll, 1);
    Serial.print(F(" pitch:")); Serial.println(pitch, 1);
  }
}
