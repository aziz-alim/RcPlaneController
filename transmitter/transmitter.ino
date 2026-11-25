/*
 * RC Plane Controller - Verici (Transmitter)
 * Arduino Nano + NRF24L01
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
 * Seri protokol (PC -> verici), 13 bayt:
 *   0xAA 0x55 | 5 x uint16 LE (throttle, aileron, elevator, rudder, aux) | XOR checksum
 * Checksum tutmayan veya 1000-2000 disinda deger iceren cerceve atilir.
 *
 * Telemetri (verici -> PC), 16 bayt, her gonderimden sonra:
 *   0xBB 0x66 | linkPct, fresh, Telemetry (11 bayt) | XOR checksum
 *   linkPct: ACK alinan gonderim orani (0-100, yumusatilmis)
 *   fresh  : 1 = bu gonderimde yeni telemetri geldi
 * Telemetri alicidan NRF24 ACK payload ile gelir; alici da ayni surum olmali.
 * Python control.py ile kullanilir.
 */

#include <SPI.h>
#include <RF24.h>

RF24 radio(9, 10);

// Guc seviyesi: PCB antenli modulde RF24_PA_LOW, antenli PA+LNA modulde
// (temiz 3.3V besleme + kondansator ile) RF24_PA_MAX kullan.
const uint8_t RADIO_PA = RF24_PA_LOW;

const byte txAddress[6] = "00001";
const byte SYNC1 = 0xAA;
const byte SYNC2 = 0x55;
const byte DATA_BYTES = 10;  // 5 kanal x 2 byte

struct __attribute__((packed)) ControlPacket {
  uint16_t throttle;
  uint16_t aileron;
  uint16_t elevator;
  uint16_t rudder;
  uint16_t aux;
};

ControlPacket packet;

// receiver.ino ile ayni, 11 bayt
struct __attribute__((packed)) Telemetry {
  uint8_t  seq;
  int16_t  roll10;
  int16_t  pitch10;
  uint16_t battMv;
  uint8_t  flags;
  uint8_t  rxRate;
  uint16_t throttle;
};

Telemetry telem;
float linkQuality = 0;   // 0-100
const byte TSYNC1 = 0xBB;
const byte TSYNC2 = 0x66;

// Cerceve ayristirici durumu
byte frameState = 0;       // 0: SYNC1 bekle, 1: SYNC2 bekle, 2: veri, 3: checksum
byte frameBuf[DATA_BYTES];
byte frameIdx = 0;
unsigned long badFrames = 0;

void setup() {
  Serial.begin(115200);

  if (!radio.begin()) {
    while (1);
  }

  packet.throttle = 1000;
  packet.aileron  = 1500;
  packet.elevator = 1500;
  packet.rudder   = 1500;
  packet.aux      = 1000;

  radio.openWritingPipe(txAddress);
  radio.setPALevel(RADIO_PA);
  radio.setDataRate(RF24_250KBPS);
  radio.setChannel(108);
  radio.setAutoAck(true);
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  // 250 kbps'te ACK payload icin en az 1500 us bekleme; 3 tekrar.
  radio.setRetries(5, 3);
  radio.stopListening();
}

void sendTelemetryToPc(bool fresh) {
  byte out[2 + sizeof(Telemetry)];
  out[0] = (byte)(linkQuality + 0.5);
  out[1] = fresh ? 1 : 0;
  memcpy(out + 2, &telem, sizeof(Telemetry));
  byte chk = 0;
  for (byte i = 0; i < sizeof(out); i++) chk ^= out[i];
  Serial.write(TSYNC1);
  Serial.write(TSYNC2);
  Serial.write(out, sizeof(out));
  Serial.write(chk);
}

void transmit() {
  bool ok = radio.write(&packet, sizeof(packet));
  bool fresh = false;
  if (ok) {
    while (radio.available()) {
      uint8_t len = radio.getDynamicPayloadSize();
      if (len == sizeof(Telemetry)) {
        radio.read(&telem, sizeof(telem));
        fresh = true;
      } else if (len > 0 && len <= 32) {
        uint8_t junk[32];
        radio.read(junk, len);
      } else {
        radio.flush_rx();
        break;
      }
    }
  }
  linkQuality = linkQuality * 0.9 + (ok ? 100.0 : 0.0) * 0.1;
  sendTelemetryToPc(fresh);
}

static bool inRange(uint16_t v) {
  return v >= 1000 && v <= 2000;
}

void processByte(byte b) {
  switch (frameState) {
    case 0:
      if (b == SYNC1) frameState = 1;
      break;
    case 1:
      if (b == SYNC2)      { frameState = 2; frameIdx = 0; }
      else if (b != SYNC1) { frameState = 0; }
      break;
    case 2:
      frameBuf[frameIdx++] = b;
      if (frameIdx >= DATA_BYTES) frameState = 3;
      break;
    case 3: {
      frameState = 0;
      byte chk = 0;
      for (byte i = 0; i < DATA_BYTES; i++) chk ^= frameBuf[i];
      if (chk != b) { badFrames++; return; }

      ControlPacket p;
      p.throttle = frameBuf[0] | (frameBuf[1] << 8);
      p.aileron  = frameBuf[2] | (frameBuf[3] << 8);
      p.elevator = frameBuf[4] | (frameBuf[5] << 8);
      p.rudder   = frameBuf[6] | (frameBuf[7] << 8);
      p.aux      = frameBuf[8] | (frameBuf[9] << 8);
      if (!inRange(p.throttle) || !inRange(p.aileron) || !inRange(p.elevator) ||
          !inRange(p.rudder) || !inRange(p.aux)) { badFrames++; return; }

      packet = p;
      transmit();   // ACK + otomatik tekrar; eski cift gonderim gereksiz
      break;
    }
  }
}

void loop() {
  while (Serial.available() > 0) {
    processByte(Serial.read());
  }
}
