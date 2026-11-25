# RC Plane Controller

2× Arduino Nano + NRF24L01+PA+LNA + MPU6050 ile Steam Deck'ten kumanda edilen RC uçak.

```
Steam Deck (control.py) ──USB seri──> Verici Nano ──NRF24──> Alıcı Nano ──> ESC + 4 servo
                                                               └── MPU6050 (stabilizasyon)
```

## Proje Yapısı

```
├── control.py                    # Steam Deck yer istasyonu (tam ekran arayüz)
├── transmitter/transmitter.ino   # Verici: seri çerçeve doğrulama + NRF24 gönderim
├── receiver/receiver.ino         # Alıcı: gaz kilidi, failsafe, servo yön/subtrim, stabilizasyon
├── launch.sh                     # Steam Deck başlatma
└── requirements.txt
```

## Kanallar ve "pilot kuralı"

Kumanda ve alıcı aynı kuralı kullanır; servo yönleri yalnız alıcıda ayarlanır.

| Kanal | Pin | 1500'ün üstü demek | Kumanda |
|---|---|---|---|
| Throttle | D3 (ESC) | daha fazla gaz | Sol stick Y (artımlı) |
| Aileron | D5 sağ, D4 sol | sağa yatış | Sağ stick X |
| Elevator | D6 | burun yukarı | Sağ stick Y (geri çek = burun yukarı) |
| Rudder | D7 | burun sağa | Sol stick X |
| AUX | — | 2000 = stabilizasyon açık | A |

Bir yüzey ters gidiyorsa `receiver.ino` içinde ilgili sabiti değiştir:

```cpp
const bool REV_AILERON      = true;   // sağ kanat aileron (D5)
const bool REV_AILERON_LEFT = false;  // sol kanat, sağ kanada göre ters mi (D4)
const bool REV_ELEVATOR     = true;
const bool REV_RUDDER       = false;
const int  SUBTRIM_AIL_R = 0, SUBTRIM_AIL_L = 0, SUBTRIM_ELE = 0, SUBTRIM_RUD = 0;  // µs
```

Varsayılanlar eski control.py davranışını korur; tezgâhta doğrulanmalı.

---

## Güç ve Bağlantı (doğrulanmış, 3S LiPo)

```
LiPo 3S ─┬─ ESC ── motor
         │    ├─ kırmızı (dahili BEC 5V/2A) ──[1000µF]── 4× SG90 +
         │    └─ kahverengi (GND) ───────────────────── 4× SG90 −
         │                                                  │
         └─ LM2596 IN   OUT+ (5.0–5.1V) ─┬─ Nano 5V pini    │
                                         ├─ MPU6050 VCC     │
                                         └─ NRF adaptör VCC ─[100µF]
                        OUT− ────────────┬─ Nano GND, MPU GND, NRF GND
                                         └──────────────────┘  ortak GND kablosu
```

Kurallar:
- **Nano VIN boş kalır.** VIN kartın regülatöründen geçer, ~7 V ister; 5 V verilirse hat ~4 V'a düşer.
- **İki 5V hattı (ESC BEC ve LM2596) birbirine bağlanmaz**, yalnız GND'leri ortaktır.
- Servo GND'si Nano GND'sine **doğrudan** bağlı olmalı; pil kabloları üzerinden dolaşan toprak servo titremesi yapar.
- LM2596 çıkışı Nano bağlanmadan önce **multimetreyle** 5.0–5.1 V'a ayarlanır; trimpot sıcak silikonla sabitlenir.
- 3S'te ESC'nin lineer BEC'i 4 SG90 ile ısınabilir: pervanesiz 10 dk servo testiyle ısıyı ve voltajı (≥ 4.8 V) kontrol et. Yetmezse servolar için ayrı anahtarlamalı UBEC (ESC kırmızı teli yalıtılır).

### NRF24L01+PA+LNA (adaptör kartla, iki tarafta aynı)

| Adaptör | Nano |
|---|---|
| VCC | 5V (alıcıda LM2596 OUT+) + 100 µF |
| GND | GND |
| CE | D9 |
| CSN | D10 |
| SCK | D13 |
| MOSI | D11 |
| MISO | D12 |

- Modülün kendi pinlerine 5 V verilmez (en fazla 3.6 V); besleme her zaman adaptörden.
- Tezgâhta (modüller < 1 m) `RADIO_PA = RF24_PA_LOW`; PA+LNA yakında en yüksek güçte alıcıyı doyurur. Uçuşta vericide `RF24_PA_MAX`.

### MPU6050

| MPU6050 | Nano |
|---|---|
| VCC | 5V |
| GND | GND |
| SCL | A5 |
| SDA | A4 |

**Montaj:** çip yüzü yukarı, X oku burna, Y oku sol kanada. Titreşime karşı sünger/çift taraflı bantla. Açılışta kalibrasyon için uçak düz ve hareketsiz durmalı.

---

## Kurulum

```bash
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
```

Arduino IDE → Kütüphane Yöneticisi: **RF24** (TMRh20) ve **MPU6050_light**.
`transmitter/transmitter.ino` → verici Nano, `receiver/receiver.ino` → alıcı Nano.

## Kullanım (Steam Deck)

```bash
./launch.sh            # tam ekran
./launch.sh --windowed # pencere
python3 control.py --tty   # arayüzsüz
```

Arayüz seri portu ve kumandayı otomatik bulur, koparsa yeniden bağlanır. Kumanda yokken güvenli değerler (gaz 1000, yüzeyler orta) gönderilir.

| Tuş | İşlev |
|---|---|
| **Sol stick Y** | Gaz: yukarı tut = artır, aşağı tut = azalt, bırak = kalır (tam stick ≈ %60/sn) |
| **L1 + R1 (1 sn)** | ARM / DISARM. ARM yalnız gaz sıfırken |
| **L2** | Acil kes: gaz 0 + disarm |
| **A** | Stabilizasyon aç/kapa |
| **X / Y** | Rate: YUMUŞAK (%60, expo 0.3) / NORMAL (%100, expo 0.2). Açılış: YUMUŞAK |
| **D-pad** | Trim: sol/sağ = aileron, aşağı/yukarı = elevator (5 µs adım, ±100). `trims.json`'a kaydedilir |
| **Select (1 sn)** | Trimleri sıfırla |
| **Start (1 sn)** | Çıkış (çıkarken gaz 0 gönderilir) |

Stabilizasyon açıkken stick "hedef açı" anlamına gelir ve trim uygulanmaz.

## Seri protokol (PC → verici)

13 bayt: `0xAA 0x55` + 5 × uint16 LE (throttle, aileron, elevator, rudder, aux) + XOR checksum.
Checksum'ı tutmayan ya da 1000–2000 dışındaki çerçeve atılır.

## Telemetri (uçak → Deck)

Alıcı her kontrol paketine NRF24 **ACK payload** ile cevap verir; cevabın içinde uçağın durumu vardır.
Verici bunu USB üzerinden Deck'e iletir, `control.py` ortadaki panelde gösterir:

- **Yapay ufuk**: yatış (roll, + = sağa yatık) ve burun açısı (pitch, + = burun yukarı), MPU6050'den.
- **Pil**: A0'daki gerilim bölücüden (aşağıya bak), hücre başı 3.7 V altı sarı, 3.5 V altı kırmızı.
- **Bağlantı**: vericinin ACK aldığı gönderim oranı (%) ve alıcının saniyede aldığı paket sayısı.
- 1 sn yeni veri gelmezse ufuk grileşir, "TELEMETRİ YOK" yazar.

**Verici ve alıcı aynı sürümle yüklenmeli** (ikisi de autoAck + dinamik payload kullanıyor; eski alıcı ile yeni verici konuşmaz).
Yönü tezgâhta doğrula: uçağı elle sağa yatır → ekranda ufkun sağ ucu kalkmalı; burnu kaldır → ufuk aşağı inmeli.

Verici → PC çerçevesi, 16 bayt: `0xBB 0x66` + `linkPct u8, fresh u8, seq u8, roll10 i16, pitch10 i16, battMv u16, flags u8, rxRate u8, throttle u16` + XOR checksum.

### Pil ölçümü (isteğe bağlı)

```
pil + (ana kablo ya da denge soketinin en dış ucu) ──[10k]──┬──[1k]──[1k]── GND
                                                            └── A0
```

10k = kahve-siyah-turuncu, 1k = kahve-siyah-kırmızı (iki tane seri = 2k). 12.6 V'ta A0 ≈ 2.1 V olur;
çözünürlük ~30 mV, sürekli çekiş ~1 mA. Dirençleri multimetreyle ölçüp `BATT_R_TOP/BOT`'a gerçek değerleri yazarsan ölçüm daha doğru olur. Bağladıktan sonra `receiver.ino`'da `BATT_SENSE = true` yap,
Nano'nun 5V pinini multimetreyle ölçüp `BATT_VREF_MV`'ye yaz. Bölücü bağlı değilken `true` yapma.

### Buzzer (isteğe bağlı, D8)

`D8 ──[100Ω]── buzzer +`, `buzzer − ── GND`. `BUZZER_MODE`: `BUZZER_ACTIVE` (sabit tonlu) veya `BUZZER_PASSIVE` (`tone()`).
Açılışta kısa bip; sinyal 5 sn'den uzun yoksa 2 sn'de bir bulucu bip (kayıp uçak); pil düşükse 4 sn'de bir kısa bip.
Varsayılan `BUZZER_NONE`.

## Güvenlik

- **Gaz kilidi (alıcı):** Bağlantı kurulunca veya koptuktan sonra gaz, önce ≤1050 görülmeden açılmaz.
- **Failsafe:** 1.5 sn sinyal yoksa gaz kesilir, yüzeyler ortalanır, stabilizasyon kapanır, NRF yeniden başlatılır.
- Alıcı tüm kanalları 1000–2000'e sınırlar. Seri monitörde (115200) 250 ms'de bir durum satırı: `GAZ:KILITLI/ACIK`, `LINK`, roll/pitch.
- ESC bir kez 1000–2000 µs aralığına kalibre edilmeli.
- **Pervaneyi takmadan önce** her yüzeyin yönünü ve failsafe'i test et.

## Stabilizasyon

PD kontrolcü, pilot kuralında: `komut = KP·(hedef − açı) − KD·açısal hız`. Max hedef açı ±45°.
`ROLL_KP/KD`, `PITCH_KP/KD` (µs/°, µs/(°/s)) tezgâhta ayarlanmalı; ilk uçuşta stabilizasyon kapalı.
Tezgâh kontrolü: stab açıkken uçağı sağa yatır → aileron sola yatış yönünde düzeltmeli; burnu kaldır → elevator burun aşağı yönünde.
