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
"""

import argparse
import glob
import json
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

    def _stick(self, cx, cy, ax, ay, title, xl, yl):
        pg = self.pg
        r = 120
        pg.draw.circle(self.canvas, C["bg"], (cx, cy), r)
        pg.draw.circle(self.canvas, C["line"], (cx, cy), r, 2)
        pg.draw.line(self.canvas, C["line"], (cx - r, cy), (cx + r, cy), 1)
        pg.draw.line(self.canvas, C["line"], (cx, cy - r), (cx, cy + r), 1)
        dx, dy = cx + int(clamp(ax, -1, 1) * (r - 14)), cy + int(clamp(ay, -1, 1) * (r - 14))
        pg.draw.circle(self.canvas, C["accent"], (dx, dy), 14)
        self._txt(title, self.f_md, C["text"], (cx, cy - r - 44), "midtop")
        self._txt(xl, self.f_sm, C["dim"], (cx, cy + r + 12), "midtop")
        self._txt(yl, self.f_sm, C["dim"], (cx - r - 14, cy), "midright")

    # -- cizim --
    def draw(self, logic, st, link, pad_name, ch):
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

        # Stick'ler
        self._stick(470, 440, st.lx, st.ly, "SOL STICK", "RUDDER", "GAZ")
        self._stick(810, 440, st.rx, st.ry, "SAG STICK", "AILERON", "ELEV")

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


def tty_status(logic, link, ch):
    thr, ail, ele, rud, aux = ch
    s = (f"\r\033[2K{'ARMED ' if logic.armed else 'KILITLI'} "
         f"GAZ %{int(logic.throttle_pct * 100):3d} A:{ail} E:{ele} R:{rud} "
         f"STAB:{'A' if logic.stab else 'P'} RATE:{logic.rate_name} "
         f"TRIM:{logic.trims['ail']:+d}/{logic.trims['ele']:+d} "
         f"SERI:{'OK' if link.connected else 'YOK'} {logic.message}")
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
            if now - last_send >= SEND_PERIOD:
                link.send(build_frame(*ch), now)
                last_send = now

            if ui is not None:
                if ui.events():
                    break
                ui.draw(logic, st, link, pad_name, ch)
            elif now - last_tty > 0.15:
                tty_status(logic, link, ch)
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
