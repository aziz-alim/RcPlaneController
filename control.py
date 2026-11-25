#!/usr/bin/env python3
"""
RC Plane Controller - Steam Deck yer istasyonu

Kumanda (Steam Deck / Xbox duzeni, Mode 2):
  Sol stick Y   : gaz (artimli: yukari tut = artir, asagi tut = azalt, birak = kalir)
  Sol stick X   : rudder
  Sag stick X   : aileron
  Sag stick Y   : elevator (geri cek = burun yukari)
  L1+R1 (1 sn)  : ARM / DISARM  (ARM yalniz gaz sifirdayken)
  L2            : ACIL KES (gaz 0 + disarm)
  A             : stabilizasyon ac/kapa
  X / Y         : rate (YUMUSAK / NORMAL)
  D-pad         : trim (sol/sag = aileron, yukari/asagi = elevator)
  Select (1 sn) : trimleri sifirla
  Start (1 sn)  : cikis

Kanal kurali ("pilot kurali", alici da ayni kurali kullanir):
  aileron  > 1500 = saga yatis
  elevator > 1500 = burun yukari
  rudder   > 1500 = burun saga
Servo yonleri alicidaki REV_* sabitleriyle ayarlanir, burada degil.

Seri cerceve (PC -> verici): 0xAA 0x55 + 5 x uint16 LE + XOR checksum (13 bayt)
Kanal sirasi: throttle, aileron, elevator, rudder, aux

Telemetri (verici -> PC): 0xBB 0x66 + 13 bayt + XOR checksum (16 bayt)
  linkPct u8, fresh u8, seq u8, roll10 i16, pitch10 i16, battMv u16,
  flags u8, rxRate u8, throttle u16   (roll + = saga yatik, pitch + = burun yukari)
"""

import argparse
import glob
import json
import math
import os
import select as sel
import struct
import sys
import time

# ═══════════════════════════════════════════════════════════════
# Ayarlar
# ═══════════════════════════════════════════════════════════════

BAUD = 115200
SEND_PERIOD = 0.030          # ~33 Hz
LOOP_HZ = 60
DEADZONE = 0.08

THROTTLE_RATE = 0.6          # tam stick'te saniyede %60 (0 -> %100 ~1.7 sn)
ARM_HOLD_S = 1.0
EXIT_HOLD_S = 1.0
TRIM_RESET_HOLD_S = 1.0

TRIM_STEP = 5                # us
TRIM_LIMIT = 100             # us
RATES = {                    # ad: (oran, expo)
    "YUMUSAK": (0.60, 0.30),
    "NORMAL":  (1.00, 0.20),
}
DEFAULT_RATE = "YUMUSAK"

CH_MIN, CH_MAX, CH_MID = 1000, 2000, 1500
SYNC = b'\xaa\x55'

TELEM_SYNC = b'\xbb\x66'
TELEM_FMT = "<BBBhhHBBH"
TELEM_LEN = struct.calcsize(TELEM_FMT)   # 13
TELEM_STALE_S = 1.0
TF_STAB, TF_THR_ARMED, TF_BATT_LOW, TF_BATT_OK = 0x01, 0x02, 0x04, 0x08
BATT_CELLS = 3
BATT_WARN_V_CELL = 3.7
BATT_LOW_V_CELL = 3.5

TRIM_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "trims.json")


def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def build_frame(throttle, aileron, elevator, rudder, aux):
    vals = [clamp(int(v), CH_MIN, CH_MAX)
            for v in (throttle, aileron, elevator, rudder, aux)]
    payload = struct.pack("<HHHHH", *vals)
    chk = 0
    for b in payload:
        chk ^= b
    return SYNC + payload + bytes([chk])


def deadzone(v):
    if abs(v) <= DEADZONE:
        return 0.0
    s = 1.0 if v > 0 else -1.0
    return s * (abs(v) - DEADZONE) / (1.0 - DEADZONE)


def shape(v, rate, expo):
    return rate * ((1.0 - expo) * v + expo * v * v * v)


# ═══════════════════════════════════════════════════════════════
# Telemetri (verici -> PC)
# ═══════════════════════════════════════════════════════════════

class TelemetryParser:
    """Bayt akisindan telemetri cercevelerini ayiklar (bozuklari atar)."""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        total = len(TELEM_SYNC) + TELEM_LEN + 1
        while True:
            i = self.buf.find(TELEM_SYNC)
            if i < 0:
                # yarim sync bayti kalabilir
                del self.buf[:max(0, len(self.buf) - 1)]
                break
            if i:
                del self.buf[:i]
            if len(self.buf) < total:
                break
            body = bytes(self.buf[2:2 + TELEM_LEN])
            chk = 0
            for b in body:
                chk ^= b
            if chk != self.buf[2 + TELEM_LEN]:
                self.bad += 1
                del self.buf[:1]          # sonraki sync'i ara
                continue
            del self.buf[:total]
            (link, fresh, seq, roll10, pitch10, batt_mv,
             flags, rx_rate, thr) = struct.unpack(TELEM_FMT, body)
            out.append({"link": link, "fresh": bool(fresh), "seq": seq,
                        "roll": roll10 / 10.0, "pitch": pitch10 / 10.0,
                        "batt_mv": batt_mv, "flags": flags,
                        "rx_rate": rx_rate, "throttle": thr})
        return out


class Telemetry:
    """Son bilinen ucak durumu."""

    def __init__(self):
        self.link = 0              # verici ACK orani %
        self.last_frame = -1e9     # vericiden son cerceve
        self.last_fresh = -1e9     # ucaktan son yeni veri
        self.roll = self.pitch = 0.0
        self.batt_mv = 0
        self.flags = 0
        self.rx_rate = 0
        self.throttle = 1000

    def apply(self, frames, now):
        for f in frames:
            self.last_frame = now
            self.link = f["link"]
            if f["fresh"]:
                self.last_fresh = now
                self.roll, self.pitch = f["roll"], f["pitch"]
                self.batt_mv = f["batt_mv"]
                self.flags = f["flags"]
                self.rx_rate = f["rx_rate"]
                self.throttle = f["throttle"]

    def ok(self, now):
        return now - self.last_fresh < TELEM_STALE_S

    def tx_ok(self, now):
        return now - self.last_frame < TELEM_STALE_S

    @property
    def batt_sense(self):
        return bool(self.flags & TF_BATT_OK) and self.batt_mv > 0

    @property
    def batt_v(self):
        return self.batt_mv / 1000.0

    def batt_level(self):
        """'ok' / 'warn' / 'low' (hucre basi)."""
        cell = self.batt_v / BATT_CELLS
        if cell < BATT_LOW_V_CELL:
            return "low"
        if cell < BATT_WARN_V_CELL:
            return "warn"
        return "ok"


# ═══════════════════════════════════════════════════════════════
# Girdi: evdev gamepad (Linux / Steam Deck)
# ═══════════════════════════════════════════════════════════════

try:
    import evdev
    from evdev import ecodes
    HAS_EVDEV = True
except ImportError:
    evdev = None
    ecodes = None
    HAS_EVDEV = False


class GamepadState:
    """Kontrol mantiginin okudugu normalize girdi. Testlerde elle doldurulur."""

    def __init__(self):
        self.lx = self.ly = self.rx = self.ry = 0.0   # -1..1, Y yukari = -1
        self.l2 = self.r2 = 0.0                      # 0..1
        self.hat_x = self.hat_y = 0                  # -1, 0, 1
        self.buttons = set()                         # "A","B","X","Y","L1","R1","START","SELECT"


class EvdevGamepad:
    PRIORITY = ("steam deck", "x-box", "xbox", "microsoft", "360", "gamepad",
                "controller", "joystick")

    def __init__(self):
        if not HAS_EVDEV:
            raise RuntimeError("evdev yok (yalniz Linux / Steam Deck)")
        self.dev = self._find()
        self.name = self.dev.name
        self.state = GamepadState()
        self._abs = {}
        self._btn_map = {
            ecodes.BTN_SOUTH: "A", ecodes.BTN_EAST: "B",
            ecodes.BTN_NORTH: "X", ecodes.BTN_WEST: "Y",   # xpad X/Y karisik; ikisi de rate
            ecodes.BTN_TL: "L1", ecodes.BTN_TR: "R1",
            ecodes.BTN_START: "START", ecodes.BTN_SELECT: "SELECT",
        }
        try:
            self.dev.grab()
        except Exception:
            pass

    def _find(self):
        cands = []
        for path in evdev.list_devices():
            try:
                d = evdev.InputDevice(path)
                caps = d.capabilities()
                if ecodes.EV_ABS in caps and ecodes.EV_KEY in caps:
                    cands.append(d)
                else:
                    d.close()
            except Exception:
                continue
        if not cands:
            raise RuntimeError("gamepad bulunamadi")
        for kw in self.PRIORITY:
            for d in cands:
                if kw in d.name.lower():
                    return d
        return cands[0]

    def _norm(self, code, val):
        info = self.dev.absinfo(code)
        rng = info.max - info.min
        if rng <= 0:
            return 0.0
        return -1.0 + 2.0 * (val - info.min) / rng

    def poll(self):
        st = self.state
        while True:
            r, _, _ = sel.select([self.dev.fd], [], [], 0)
            if not r:
                break
            for ev in self.dev.read():
                if ev.type == ecodes.EV_ABS:
                    c = ev.code
                    if c == ecodes.ABS_HAT0X:
                        st.hat_x = ev.value
                    elif c == ecodes.ABS_HAT0Y:
                        st.hat_y = ev.value
                    elif c in (ecodes.ABS_Z, ecodes.ABS_RZ):
                        info = self.dev.absinfo(c)
                        rng = max(1, info.max - info.min)
                        v = clamp((ev.value - info.min) / rng, 0.0, 1.0)
                        if c == ecodes.ABS_Z:
                            st.l2 = v
                        else:
                            st.r2 = v
                    elif c == ecodes.ABS_X:
                        st.lx = self._norm(c, ev.value)
                    elif c == ecodes.ABS_Y:
                        st.ly = self._norm(c, ev.value)
                    elif c == ecodes.ABS_RX:
                        st.rx = self._norm(c, ev.value)
                    elif c == ecodes.ABS_RY:
                        st.ry = self._norm(c, ev.value)
                elif ev.type == ecodes.EV_KEY and ev.code in self._btn_map:
                    name = self._btn_map[ev.code]
                    if ev.value:
                        st.buttons.add(name)
                    else:
                        st.buttons.discard(name)
        return st

    def close(self):
        try:
            self.dev.ungrab()
        except Exception:
            pass
        try:
            self.dev.close()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════
# Kontrol mantigi (donanimdan bagimsiz, test edilebilir)
# ═══════════════════════════════════════════════════════════════

class FlightLogic:
    def __init__(self, trims=None):
        self.armed = False
        self.throttle_pct = 0.0
        self.stab = False
        self.rate_name = DEFAULT_RATE
        self.trims = {"ail": 0, "ele": 0}
        if trims:
            self.trims.update({k: int(v) for k, v in trims.items() if k in self.trims})
        self.trims_dirty = False

        self.ail_cmd = self.ele_cmd = self.rud_cmd = 0.0   # -1..1, rate/expo sonrasi
        self.message = ""
        self.message_until = 0.0
        self.arm_hold = 0.0
        self.exit_hold = 0.0
        self.trim_reset_hold = 0.0
        self.quit = False

        self._arm_latch = False
        self._exit_latch = False
        self._reset_latch = False
        self._prev_buttons = set()
        self._prev_hat = (0, 0)

    def _say(self, text, now, dur=2.0):
        self.message = text
        self.message_until = now + dur

    def update(self, st, dt, now):
        pressed = st.buttons - self._prev_buttons
        self._prev_buttons = set(st.buttons)

        # --- ACIL KES ---
        if st.l2 > 0.5:
            if self.armed or self.throttle_pct > 0:
                self._say("ACIL KES", now)
            self.armed = False
            self.throttle_pct = 0.0

        # --- ARM / DISARM: L1+R1 basili tut ---
        if "L1" in st.buttons and "R1" in st.buttons:
            if not self._arm_latch:
                self.arm_hold += dt
                if self.arm_hold >= ARM_HOLD_S:
                    self._arm_latch = True
                    self.arm_hold = 0.0
                    if self.armed:
                        self.armed = False
                        self.throttle_pct = 0.0
                        self._say("DISARM", now)
                    elif self.throttle_pct > 0 or deadzone(-st.ly) > 0:
                        self._say("ARM REDDEDILDI: GAZI SIFIRLA", now)
                    else:
                        self.armed = True
                        self._say("ARMED", now)
        else:
            self.arm_hold = 0.0
            self._arm_latch = False

        # --- GAZ (artimli) ---
        if self.armed:
            g = deadzone(-st.ly)  # yukari = +
            self.throttle_pct = clamp(self.throttle_pct + g * THROTTLE_RATE * dt, 0.0, 1.0)
        else:
            self.throttle_pct = 0.0

        # --- STAB / RATE ---
        if "A" in pressed:
            self.stab = not self.stab
            self._say("STAB " + ("ACIK" if self.stab else "KAPALI"), now)
        if pressed & {"X", "Y"}:
            names = list(RATES)
            self.rate_name = names[(names.index(self.rate_name) + 1) % len(names)]
            self._say("RATE " + self.rate_name, now)

        # --- TRIM (D-pad, kenar tetikli) ---
        hat = (st.hat_x, st.hat_y)
        if hat != self._prev_hat:
            if st.hat_x and st.hat_x != self._prev_hat[0]:
                self._trim("ail", TRIM_STEP * st.hat_x, now)
            if st.hat_y and st.hat_y != self._prev_hat[1]:
                # D-pad asagi (hat_y=+1) = burun yukari trim
                self._trim("ele", TRIM_STEP * st.hat_y, now)
            self._prev_hat = hat

        if "SELECT" in st.buttons:
            if not self._reset_latch:
                self.trim_reset_hold += dt
                if self.trim_reset_hold >= TRIM_RESET_HOLD_S:
                    self._reset_latch = True
                    self.trims = {"ail": 0, "ele": 0}
                    self.trims_dirty = True
                    self._say("TRIMLER SIFIRLANDI", now)
        else:
            self.trim_reset_hold = 0.0
            self._reset_latch = False

        # --- CIKIS ---
        if "START" in st.buttons:
            if not self._exit_latch:
                self.exit_hold += dt
                if self.exit_hold >= EXIT_HOLD_S:
                    self._exit_latch = True
                    self.quit = True
        else:
            self.exit_hold = 0.0
            self._exit_latch = False

        # --- STICK'LER ---
        rate, expo = RATES[self.rate_name]
        self.ail_cmd = shape(deadzone(st.rx), rate, expo)
        self.ele_cmd = shape(deadzone(st.ry), rate, expo)   # geri cek (+) = burun yukari
        self.rud_cmd = shape(deadzone(st.lx), rate, expo)

        if now > self.message_until:
            self.message = ""

    def _trim(self, key, delta, now):
        self.trims[key] = clamp(self.trims[key] + delta, -TRIM_LIMIT, TRIM_LIMIT)
        self.trims_dirty = True
        self._say(f"TRIM {key.upper()} {self.trims[key]:+d}", now, 1.0)

    def channels(self):
        """(throttle, aileron, elevator, rudder, aux) us."""
        thr = CH_MIN + int(round(self.throttle_pct * 1000)) if self.armed else CH_MIN
        # Stab acikken stick hedef aci demek; trim eklenmez (hedefi kaydirmasin)
        ta = 0 if self.stab else self.trims["ail"]
        te = 0 if self.stab else self.trims["ele"]
        ail = CH_MID + int(round(self.ail_cmd * 500)) + ta
        ele = CH_MID + int(round(self.ele_cmd * 500)) + te
        rud = CH_MID + int(round(self.rud_cmd * 500))
        aux = 2000 if self.stab else 1000
        return (clamp(thr, CH_MIN, CH_MAX), clamp(ail, CH_MIN, CH_MAX),
                clamp(ele, CH_MIN, CH_MAX), clamp(rud, CH_MIN, CH_MAX), aux)

    def safe_channels(self):
        return (CH_MIN, CH_MID, CH_MID, CH_MID, 1000)


def load_trims():
    try:
        with open(TRIM_FILE) as f:
            return json.load(f)
    except Exception:
        return None


def save_trims(trims):
    try:
        with open(TRIM_FILE, "w") as f:
            json.dump(trims, f)
    except Exception:
        pass


# ═══════════════════════════════════════════════════════════════
# Seri baglanti (otomatik yeniden baglanma)
# ═══════════════════════════════════════════════════════════════

def find_port():
    ports = (glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*") +
             glob.glob("/dev/cu.usbserial*") + glob.glob("/dev/cu.wchusbserial*") +
             glob.glob("/dev/cu.usbmodem*"))
    return ports[0] if ports else None


class SerialLink:
    RETRY_S = 2.0

    def __init__(self, port=None):
        self.fixed_port = port
        self.port = None
        self.ser = None
        self.next_try = 0.0
        self.ready_at = 0.0
        self.frames = 0

    @property
    def connected(self):
        return self.ser is not None

    def tick(self, now):
        if self.ser is None and now >= self.next_try:
            self.next_try = now + self.RETRY_S
            port = self.fixed_port or find_port()
            if port:
                try:
                    import serial
                    self.ser = serial.Serial(port, BAUD, timeout=0, write_timeout=0.05)
                    self.port = port
                    self.ready_at = now + 1.5   # Nano USB acilinca resetlenir
                    try:
                        self.ser.reset_input_buffer()
                    except Exception:
                        pass
                except Exception:
                    self.ser = None

    def send(self, frame, now):
        if self.ser is None or now < self.ready_at:
            return
        try:
            self.ser.write(frame)
            self.frames += 1
        except Exception:
            self.close()

    def read(self):
        if self.ser is None:
            return b""
        try:
            n = self.ser.in_waiting
            return self.ser.read(n) if n else b""
        except Exception:
            self.close()
            return b""

    def close(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.port = None


# ═══════════════════════════════════════════════════════════════
# Tam ekran arayuz (pygame, 1280x800 Steam Deck)
# ═══════════════════════════════════════════════════════════════

W, H = 1280, 800

C = {
    "bg": (14, 16, 22), "panel": (24, 27, 36), "line": (48, 53, 68),
    "text": (220, 224, 235), "dim": (125, 132, 150),
    "accent": (0, 200, 175), "amber": (255, 186, 60),
    "green": (70, 205, 110), "red": (235, 70, 70), "blue": (90, 150, 255),
    "sky": (52, 120, 200), "ground": (130, 86, 46),
}


class Dashboard:
    def __init__(self, windowed=False):
        import pygame
        self.pg = pygame
        pygame.init()
        pygame.display.set_caption("RC Plane")
        if windowed:
            self.screen = pygame.display.set_mode((W, H))
        else:
            self.screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
        pygame.mouse.set_visible(windowed)
        self.canvas = pygame.Surface((W, H))
        f = pygame.font.Font
        self.f_sm = f(None, 26)
        self.f_md = f(None, 34)
        self.f_lg = f(None, 52)
        self.f_xl = f(None, 96)

    # -- yardimcilar --
    def _txt(self, s, font, col, pos, anchor="topleft"):
        img = font.render(s, True, col)
        r = img.get_rect(**{anchor: pos})
        self.canvas.blit(img, r)
        return r

    def _panel(self, rect, radius=14):
        self.pg.draw.rect(self.canvas, C["panel"], rect, border_radius=radius)

    def _badge(self, text, x, y, on, col_on, anchor="topright"):
        img = self.f_sm.render(text, True, (10, 10, 14) if on else C["dim"])
        w, h = img.get_width() + 24, 38
        r = self.pg.Rect(0, 0, w, h)
        setattr(r, anchor, (x, y))
        self.pg.draw.rect(self.canvas, col_on if on else C["line"], r, border_radius=8)
        self.canvas.blit(img, img.get_rect(center=r.center))
        return r

    def _hbar(self, x, y, w, label, us, trim=None):
        pg = self.pg
        self._txt(label, self.f_sm, C["dim"], (x, y))
        self._txt(f"{us}", self.f_md, C["text"], (x + w, y - 4), "topright")
        by = y + 32
        pg.draw.rect(self.canvas, C["bg"], (x, by, w, 16), border_radius=8)
        mid = x + w // 2
        pos = x + int((us - CH_MIN) / 1000 * w)
        a, b = sorted((mid, pos))
        pg.draw.rect(self.canvas, C["accent"], (a, by, max(2, b - a), 16), border_radius=8)
        pg.draw.line(self.canvas, C["dim"], (mid, by - 4), (mid, by + 20), 2)
        if trim:
            tx = mid + int(trim / 1000 * w)
            pg.draw.polygon(self.canvas, C["amber"],
                            [(tx, by + 20), (tx - 7, by + 32), (tx + 7, by + 32)])

    def _stick(self, cx, cy, ax, ay, title, sub, r=62):
        pg = self.pg
        pg.draw.circle(self.canvas, C["bg"], (cx, cy), r)
        pg.draw.circle(self.canvas, C["line"], (cx, cy), r, 2)
        pg.draw.line(self.canvas, C["line"], (cx - r, cy), (cx + r, cy), 1)
        pg.draw.line(self.canvas, C["line"], (cx, cy - r), (cx, cy + r), 1)
        dx, dy = cx + int(clamp(ax, -1, 1) * (r - 10)), cy + int(clamp(ay, -1, 1) * (r - 10))
        pg.draw.circle(self.canvas, C["accent"], (dx, dy), 10)
        self._txt(title, self.f_sm, C["text"], (cx - r - 16, cy - 2), "bottomright")
        self._txt(sub, self.f_sm, C["dim"], (cx - r - 16, cy + 2), "topright")

    def _horizon(self, cx, cy, r, roll, pitch, live):
        """Yapay ufuk. roll + = saga yatik, pitch + = burun yukari (derece)."""
        pg = self.pg
        size = 2 * r
        surf = pg.Surface((size, size))
        surf.fill(C["sky"])
        ppd = r / 40.0                      # piksel / derece (+-40 derece gorunur)
        phi = math.radians(roll)
        dx, dy = math.cos(phi), -math.sin(phi)   # ufuk yonu (saga yatista sag uc yukari)
        nx, ny = math.sin(phi), math.cos(phi)    # yere dogru normal
        off = clamp(pitch, -80, 80) * ppd        # burun yukari -> ufuk asagi
        hx, hy = r + nx * off, r + ny * off
        L = size * 2
        ground = [(hx - dx * L, hy - dy * L), (hx + dx * L, hy + dy * L),
                  (hx + dx * L + nx * L, hy + dy * L + ny * L),
                  (hx - dx * L + nx * L, hy - dy * L + ny * L)]
        pg.draw.polygon(surf, C["ground"], ground)
        pg.draw.line(surf, (240, 240, 240), ground[0], ground[1], 3)
        # Yunuslama merdiveni (10 derecede bir)
        for deg in (-30, -20, -10, 10, 20, 30):
            o = off - deg * ppd
            px, py = r + nx * o, r + ny * o
            hw = r * (0.28 if deg % 20 else 0.4)
            a = (px - dx * hw, py - dy * hw)
            b = (px + dx * hw, py + dy * hw)
            pg.draw.line(surf, (235, 235, 235), a, b, 2)
            lbl = self.f_sm.render(str(abs(deg)), True, (235, 235, 235))
            surf.blit(lbl, lbl.get_rect(center=(b[0] + dx * 18, b[1] + dy * 18)))
        # Daire maskesi
        mask = pg.Surface((size, size))
        mask.fill(C["panel"])
        mask.set_colorkey((255, 0, 255))
        pg.draw.circle(mask, (255, 0, 255), (r, r), r)
        surf.blit(mask, (0, 0))
        if not live:
            dim = pg.Surface((size, size), pg.SRCALPHA)
            pg.draw.circle(dim, (14, 16, 22, 190), (r, r), r)
            surf.blit(dim, (0, 0))
        self.canvas.blit(surf, (cx - r, cy - r))
        pg.draw.circle(self.canvas, C["line"], (cx, cy), r, 3)
        # Yatis olcegi (ust)
        for deg in (-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60):
            a = math.radians(deg - 90)
            r1 = r - (14 if deg in (0, -30, 30, -60, 60) else 8)
            pg.draw.line(self.canvas, (235, 235, 235),
                         (cx + math.cos(a) * r1, cy + math.sin(a) * r1),
                         (cx + math.cos(a) * r, cy + math.sin(a) * r), 2)
        # Yatis isareti (gokyuzuyle doner)
        a = math.radians(-roll - 90)
        tip = (cx + math.cos(a) * (r - 16), cy + math.sin(a) * (r - 16))
        base = r - 32
        b1 = (cx + math.cos(a - 0.07) * base, cy + math.sin(a - 0.07) * base)
        b2 = (cx + math.cos(a + 0.07) * base, cy + math.sin(a + 0.07) * base)
        pg.draw.polygon(self.canvas, C["amber"], [tip, b1, b2])
        # Sabit ucak simgesi
        am = C["amber"]
        pg.draw.line(self.canvas, am, (cx - 70, cy), (cx - 22, cy), 6)
        pg.draw.line(self.canvas, am, (cx - 22, cy), (cx - 22, cy + 12), 6)
        pg.draw.line(self.canvas, am, (cx + 22, cy), (cx + 70, cy), 6)
        pg.draw.line(self.canvas, am, (cx + 22, cy), (cx + 22, cy + 12), 6)
        pg.draw.circle(self.canvas, am, (cx, cy), 5)
        if not live:
            self._txt("TELEMETRI YOK", self.f_md, C["red"], (cx, cy + 40), "midtop")

    def _meter(self, x, y, w, frac, col):
        pg = self.pg
        pg.draw.rect(self.canvas, C["bg"], (x, y, w, 14), border_radius=7)
        fw = int(w * clamp(frac, 0.0, 1.0))
        if fw > 0:
            pg.draw.rect(self.canvas, col, (x, y, max(14, fw), 14), border_radius=7)

    def _telemetry_panel(self, tp, tel, now):
        live = tel.ok(now)
        self._panel(tp)
        self._horizon(tp.x + 170, tp.centery, 140, tel.roll, tel.pitch, live)

        x0 = tp.x + 350
        x1 = tp.x + 540
        col = C["text"] if live else C["dim"]
        self._txt("YATIS", self.f_sm, C["dim"], (x0, tp.y + 22))
        self._txt(f"{tel.roll:+.0f}\u00b0", self.f_lg, col, (x0, tp.y + 48))
        self._txt("BURUN", self.f_sm, C["dim"], (x1, tp.y + 22))
        self._txt(f"{tel.pitch:+.0f}\u00b0", self.f_lg, col, (x1, tp.y + 48))

        # Pil
        y = tp.y + 120
        self._txt("PIL", self.f_sm, C["dim"], (x0, y))
        w = tp.right - 24 - x0
        if live and tel.batt_sense:
            lvl = tel.batt_level()
            bcol = {"ok": C["green"], "warn": C["amber"], "low": C["red"]}[lvl]
            cell = tel.batt_v / BATT_CELLS
            self._txt(f"{tel.batt_v:.1f} V   ({cell:.2f} V/hucre)", self.f_md, bcol,
                      (tp.right - 24, y - 4), "topright")
            self._meter(x0, y + 34, w, (cell - 3.3) / (4.2 - 3.3), bcol)
            if lvl == "low" and int(now * 2) % 2 == 0:
                self._txt("PIL DUSUK - IN!", self.f_md, C["red"], (x0, y + 54))
        else:
            self._txt("olcum yok" if live else "-", self.f_md, C["dim"],
                      (tp.right - 24, y - 4), "topright")
            self._meter(x0, y + 34, w, 0, C["dim"])

        # Baglanti
        y = tp.y + 210
        self._txt("BAGLANTI", self.f_sm, C["dim"], (x0, y))
        txok = tel.tx_ok(now)
        q = tel.link if txok else 0
        qcol = C["green"] if q >= 80 else C["amber"] if q >= 50 else C["red"]
        info = f"%{q}   {tel.rx_rate} paket/sn" if live else f"%{q}"
        self._txt(info, self.f_md, qcol if txok else C["dim"],
                  (tp.right - 24, y - 4), "topright")
        self._meter(x0, y + 34, w, q / 100.0, qcol)

        # Ucak tarafi durum
        y = tp.y + 272
        if not txok:
            s, c = "verici cevap vermiyor", C["dim"]
        elif not live:
            s, c = "UCAKTAN CEVAP YOK", C["red"]
        else:
            gaz = "GAZ ACIK" if tel.flags & TF_THR_ARMED else "GAZ KILITLI"
            stab = "STAB" if tel.flags & TF_STAB else "MANUEL"
            s, c = f"ucak: {gaz} - {stab} - ESC {tel.throttle}", C["dim"]
        self._txt(s, self.f_sm, c, (x0, y))

    # -- cizim --
    def draw(self, logic, st, link, pad_name, ch, tel):
        pg = self.pg
        cv = self.canvas
        cv.fill(C["bg"])
        thr, ail, ele, rud, aux = ch

        # Ust bar
        pg.draw.rect(cv, C["panel"], (0, 0, W, 64))
        self._txt("RC PLANE", self.f_lg, C["accent"], (24, 14))
        x = W - 20
        r = self._badge(f"SERI: {link.port}" if link.connected else "SERI YOK",
                        x, 13, link.connected, C["green"]); x = r.left - 10
        r = self._badge(pad_name[:22] if pad_name else "KUMANDA YOK",
                        x, 13, bool(pad_name), C["green"]); x = r.left - 10
        r = self._badge(f"RATE {logic.rate_name}", x, 13, logic.rate_name == "NORMAL", C["blue"]); x = r.left - 10
        self._badge("STAB ACIK" if logic.stab else "STAB KAPALI", x, 13, logic.stab, C["green"])

        # ARM banti
        band = pg.Rect(24, 80, W - 48, 96)
        if logic.armed:
            blink = int(time.monotonic() * 2) % 2 == 0
            pg.draw.rect(cv, C["green"] if blink else (40, 150, 80), band, border_radius=16)
            self._txt("ARMED  -  MOTOR AKTIF", self.f_lg, (10, 10, 14), band.center, "center")
        else:
            pg.draw.rect(cv, (70, 26, 30), band, border_radius=16)
            pg.draw.rect(cv, C["red"], band, 3, border_radius=16)
            self._txt("KILITLI", self.f_lg, C["red"], (band.x + 30, band.centery), "midleft")
            self._txt("L1 + R1 basili tut (1 sn)  ->  ARM", self.f_md, C["text"],
                      (band.right - 30, band.centery), "midright")
        if logic.arm_hold > 0:
            w = int((band.width - 8) * min(1.0, logic.arm_hold / ARM_HOLD_S))
            pg.draw.rect(cv, C["amber"], (band.x + 4, band.bottom - 12, w, 8), border_radius=4)

        # Gaz paneli
        gp = pg.Rect(24, 196, 230, 480)
        self._panel(gp)
        self._txt("GAZ", self.f_md, C["dim"], (gp.centerx, gp.y + 16), "midtop")
        bar = pg.Rect(gp.centerx - 45, gp.y + 60, 90, 330)
        pg.draw.rect(cv, C["bg"], bar, border_radius=10)
        fill = int(bar.height * logic.throttle_pct)
        if fill > 0:
            pg.draw.rect(cv, C["amber"] if logic.armed else C["dim"],
                         (bar.x, bar.bottom - fill, bar.width, fill), border_radius=10)
        self._txt(f"%{int(round(logic.throttle_pct * 100))}", self.f_lg, C["text"],
                  (gp.centerx, bar.bottom + 14), "midtop")
        self._txt(f"{thr} us", self.f_sm, C["dim"], (gp.centerx, bar.bottom + 58), "midtop")

        # Ucak telemetrisi + yapay ufuk
        self._telemetry_panel(pg.Rect(274, 196, 716, 318), tel, time.monotonic())

        # Stick'ler (kucuk)
        self._stick(470, 600, st.lx, st.ly, "SOL STICK", "rud / gaz")
        self._stick(860, 600, st.rx, st.ry, "SAG STICK", "ail / elev")

        # Kanal paneli
        kp = pg.Rect(1010, 196, 246, 480)
        self._panel(kp)
        self._txt("KANALLAR", self.f_md, C["dim"], (kp.centerx, kp.y + 16), "midtop")
        self._hbar(kp.x + 18, kp.y + 70, kp.width - 36, "AILERON", ail, logic.trims["ail"])
        self._hbar(kp.x + 18, kp.y + 150, kp.width - 36, "ELEVATOR", ele, logic.trims["ele"])
        self._hbar(kp.x + 18, kp.y + 230, kp.width - 36, "RUDDER", rud)
        self._txt(f"TRIM  AIL {logic.trims['ail']:+d}   ELE {logic.trims['ele']:+d}",
                  self.f_sm, C["amber"], (kp.centerx, kp.y + 330), "midtop")
        if logic.stab:
            self._txt("(stab: trim pasif)", self.f_sm, C["dim"],
                      (kp.centerx, kp.y + 360), "midtop")
        self._txt(f"AUX {aux}", self.f_sm, C["dim"], (kp.centerx, kp.bottom - 40), "midtop")

        # Mesaj
        if logic.message:
            img = self.f_lg.render(logic.message, True, C["amber"])
            r = img.get_rect(center=(640, 650))
            bg = r.inflate(40, 20)
            pg.draw.rect(cv, (0, 0, 0), bg, border_radius=14)
            pg.draw.rect(cv, C["amber"], bg, 3, border_radius=14)
            cv.blit(img, r)

        # Alt bilgi
        pg.draw.rect(cv, C["panel"], (0, 700, W, 100))
        lines = [
            "Sol Y: gaz (tut = degistir)   L2: ACIL KES   L1+R1: ARM   A: STAB   X/Y: RATE",
            "D-pad: trim   Select (1 sn): trim sifirla   Start (1 sn): cikis",
        ]
        for i, s in enumerate(lines):
            self._txt(s, self.f_sm, C["dim"], (W // 2, 722 + i * 34), "midtop")
        if logic.exit_hold > 0:
            w = int(W * min(1.0, logic.exit_hold / EXIT_HOLD_S))
            pg.draw.rect(cv, C["red"], (0, 794, w, 6))

        # Ekrana olcekle
        sw, sh = self.screen.get_size()
        if (sw, sh) == (W, H):
            self.screen.blit(cv, (0, 0))
        else:
            s = min(sw / W, sh / H)
            img = pg.transform.smoothscale(cv, (int(W * s), int(H * s)))
            self.screen.fill((0, 0, 0))
            self.screen.blit(img, img.get_rect(center=(sw // 2, sh // 2)))
        pg.display.flip()

    def events(self):
        quit_req = False
        for e in self.pg.event.get():
            if e.type == self.pg.QUIT:
                quit_req = True
            elif e.type == self.pg.KEYDOWN and e.key in (self.pg.K_ESCAPE, self.pg.K_q):
                quit_req = True
        return quit_req

    def close(self):
        self.pg.quit()


def tty_status(logic, link, ch, tel):
    thr, ail, ele, rud, aux = ch
    now = time.monotonic()
    if tel.ok(now):
        batt = f" PIL:{tel.batt_v:.1f}V" if tel.batt_sense else ""
        tinfo = f"ROLL:{tel.roll:+.0f} PITCH:{tel.pitch:+.0f} LINK:%{tel.link}{batt} "
    else:
        tinfo = 'TELEMETRI YOK '
    s = (f"\r\033[2K{'ARMED ' if logic.armed else 'KILITLI'} "
         f"GAZ %{int(logic.throttle_pct * 100):3d} A:{ail} E:{ele} R:{rud} "
         f"STAB:{'A' if logic.stab else 'P'} RATE:{logic.rate_name} "
         f"TRIM:{logic.trims['ail']:+d}/{logic.trims['ele']:+d} "
         f"SERI:{'OK' if link.connected else 'YOK'} {tinfo}{logic.message}")
    sys.stdout.write(s)
    sys.stdout.flush()


# ═══════════════════════════════════════════════════════════════
# Ana dongu
# ═══════════════════════════════════════════════════════════════

def main():
    ap = argparse.ArgumentParser(description="RC Plane yer istasyonu")
    ap.add_argument("--tty", action="store_true", help="arayuzsuz terminal modu")
    ap.add_argument("--windowed", action="store_true", help="tam ekran yerine pencere")
    ap.add_argument("--port", help="seri port (varsayilan: otomatik)")
    args = ap.parse_args()

    logic = FlightLogic(load_trims())
    link = SerialLink(args.port)
    parser = TelemetryParser()
    tel = Telemetry()

    pad = None
    pad_name = ""
    next_pad_try = 0.0

    ui = None
    if not args.tty:
        try:
            ui = Dashboard(windowed=args.windowed)
        except Exception as e:
            print(f"[UI] arayuz acilamadi ({e}), terminal moduna geciliyor")

    empty = GamepadState()
    last = time.monotonic()
    last_send = 0.0
    last_tty = 0.0
    period = 1.0 / LOOP_HZ

    try:
        while not logic.quit:
            now = time.monotonic()
            dt = min(0.1, now - last)
            last = now

            if pad is None and now >= next_pad_try:
                next_pad_try = now + 2.0
                try:
                    pad = EvdevGamepad()
                    pad_name = pad.name
                except Exception:
                    pad = None
                    pad_name = ""

            st = empty
            if pad is not None:
                try:
                    st = pad.poll()
                except OSError:
                    pad.close()
                    pad, pad_name = None, ""
                    logic.armed = False
                    logic.throttle_pct = 0.0
                    logic._say("KUMANDA KOPTU", now)

            logic.update(st, dt, now)
            if logic.trims_dirty:
                save_trims(logic.trims)
                logic.trims_dirty = False

            # Kumanda yoksa guvenli degerler gonder
            ch = logic.channels() if pad is not None else logic.safe_channels()

            link.tick(now)
            data = link.read()
            if data:
                tel.apply(parser.feed(data), now)
            if now - last_send >= SEND_PERIOD:
                link.send(build_frame(*ch), now)
                last_send = now

            if ui is not None:
                if ui.events():
                    break
                ui.draw(logic, st, link, pad_name, ch, tel)
            elif now - last_tty > 0.15:
                tty_status(logic, link, ch, tel)
                last_tty = now

            spent = time.monotonic() - now
            if spent < period:
                time.sleep(period - spent)
    except KeyboardInterrupt:
        pass
    finally:
        # Cikista birkac kez guvenli cerceve gonder
        for _ in range(5):
            link.send(build_frame(*logic.safe_channels()), time.monotonic() + 10)
            time.sleep(0.03)
        link.close()
        if pad is not None:
            pad.close()
        if ui is not None:
            ui.close()
        print("\nCikildi.")


if __name__ == "__main__":
    main()
