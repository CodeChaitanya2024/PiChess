"""
Smart Chessboard - Matrix Scanning Architecture
================================================
Raspberry Pi Zero W

SENSOR MATRIX (8×8 = 64 Hall Effect sensors, A3144):
  Column driver : 1× 74HC595  (shift out one-hot column select, active LOW)
  Row reader    : 1× 74HC165  (parallel-load & shift in 8 row states)
  Scan cycle    : activate col 0 → read 8 rows → activate col 1 → ... × 8 cols
  A3144 output  : active LOW  (pulls to GND when magnet present)
  1N4148 diode  : in series on each sensor OUT line (anode→sensor, cathode→row bus)
                  prevents cross-talk between columns

LED MATRIX (8×8 = 64 LEDs):
  Column driver : 1× 74HC595  (separate from sensor column driver)
  Row driver    : 1× 74HC595  (sets which rows in the active column are ON)
  Multiplex     : cycle columns at ~400 Hz, update row pattern each column
  220Ω resistor : one per LED row line (current limiting)

TOTAL SHIFT REGISTERS: 3× 74HC595  +  1× 74HC165  ✓

GPIO PINOUT (BCM numbering):
  GPIO 10  MOSI  → 74HC595 sensor-col DS  (data in, first in chain)
  GPIO 11  SCLK  → shared clock for ALL shift registers (bit-banged, NOT hardware SPI)
  GPIO 9   MISO  → 74HC165 Q7 (serial data out)
  GPIO 7          → 74HC165 SH/LD  (pulse LOW to latch parallel inputs)
  GPIO 5          → 74HC595 sensor-col STCP  (latch pulse)
                    IMPORTANT: do NOT use GPIO 8 here. setup.sh enables the SPI
                    peripheral via raspi-config, which gives the OS hard ownership
                    of GPIO 8 (CE0), 9 (MISO), 10 (MOSI) and 11 (SCLK). GPIO 8
                    would be uncontrollable from userspace, silently breaking the
                    entire sensor matrix. GPIO 5 is free of any enabled peripheral.
  GPIO 25         → 74HC595 led-col  STCP  (latch pulse, separate)
  GPIO 24         → 74HC595 led-row  STCP  (latch pulse, separate)
  GPIO 2   SDA   → LCD1602 I2C SDA
  GPIO 3   SCL   → LCD1602 I2C SCL
  GPIO 18         → Piezo buzzer (active buzzer, plain on/off)
  GPIO 23         → Button MODE    (pull-up, active LOW)
  GPIO 22         → Button CONFIRM (pull-up, active LOW)
  GPIO 27         → Button UNDO    (pull-up, active LOW)

SENSOR COLUMN WIRING:
  74HC595 Q0 → col 0 (a-file), Q1 → col 1 (b-file), ... Q7 → col 7 (h-file)
  Each column line connects to VCC-side of all 8 A3144 sensors in that column
  (A3144 VCC=5V, GND=GND, OUT pulled up via 10KΩ to 5V,
   OUT → anode of 1N4148 diode → row bus line)
  When column is driven LOW by 595, sensors in that column can pull row LOW.

SENSOR ROW WIRING:
  Row bus 0 (rank 1) → 74HC165 pin A, ... Row bus 7 (rank 8) → pin H
  Each row bus has a 10KΩ pull-up to 5V.
  Row bus LOW = piece present in active column at that rank.

LED COLUMN WIRING (common-anode per column, BC327 PNP):
  74HC595 led-col Qn → 1KΩ → BC327 PNP base
  BC327 emitter → 5V, collector → column common-anode bus
  Qn LOW → PNP ON → column anode bus = ~5V  (column active)

LED ROW WIRING:
  74HC595 led-row Qn → 220Ω → LED anode on column bus
  LED cathode → GND (common)
  Qn HIGH + column active (anode = 5V) → LED ON
  Code sends ~row_mask to led-row 595 so a set bit in the frame buffer
  means LOW on the 595 output, which sinks the cathode to GND.

  CHAINED 595 WIRING (single MOSI line):
    Pi MOSI → Sensor-Col 595 DS
             Sensor-Col 595 Q7' → LED-Col 595 DS
                                  LED-Col 595 Q7' → LED-Row 595 DS
    Pi SCLK → all three 595 SHCP (shared, bit-banged)
    Pi GPIO5  → Sensor-Col 595 STCP  (individual latch)
    Pi GPIO25 → LED-Col    595 STCP
    Pi GPIO24 → LED-Row    595 STCP

  When we shift 3 bytes in, the 3rd byte ends up in LED-Row, 2nd in LED-Col,
  1st in Sensor-Col. We latch each separately.

FIXES APPLIED vs original:
  1. Removed unused `import spidev`.
  2. PIN_SENS_COL_LATCH moved from GPIO 8 to GPIO 5. setup.sh enables the SPI
     peripheral (raspi-config do_spi 0), which gives the OS hard ownership of
     GPIO 8 (CE0). Userspace cannot toggle it, so the sensor latch would never
     fire and no chess piece could ever be detected. GPIO 5 is free.
  2. _stockfish_turn / _lichess_opponent_turn: record() now called BEFORE
     board.push() so board.san(move) receives the pre-push board state.
  3. _guide_opponent_move return value now checked; if it times out the move
     is still applied (player forced accept) but a warning is logged.
  4. _detect_human_move: removed redundant double-check of lifted_sq in
     confirmed (dead inner `if` block replaced with direct action).
  5. main(): removed unreachable `while ctrl.running` loop after ctrl.run()
     (run() blocks internally; the loop would only execute after the game
     exits, by which point running is False anyway).
  6. _led_loop: removed the first (dead) col_period assignment that was
     immediately overwritten; only the correct formula remains.
  7. Docstring updated to clarify GPIO 8 / CE0 reuse is intentional and safe.
"""

import RPi.GPIO as GPIO
# spidev intentionally NOT imported — all shift-register I/O is bit-banged.
# Importing spidev would claim GPIO 8 (CE0) for the SPI hardware peripheral,
# conflicting with our use of GPIO 8 as the sensor-column 595 latch pin.
import smbus2
import time
import json
import threading
import chess
import chess.engine
import requests
import os
from datetime import datetime
import logging

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s] %(message)s',
    handlers=[
        logging.FileHandler('/home/pi/chessboard.log'),
        logging.StreamHandler()
    ]
)
log = logging.getLogger(__name__)

# ─────────────────────────────────────────────────────────────────────────────
# GPIO PINS (BCM)
# ─────────────────────────────────────────────────────────────────────────────
PIN_SCLK           = 11   # Shared bit-bang clock for all 4 shift registers
PIN_MOSI           = 10   # Data out from Pi → sensor-col 74HC595 DS
PIN_MISO           = 9    # Data in  to Pi   ← sensor-row 74HC165 Q7

PIN_SENS_COL_LATCH = 5    # 74HC595 sensor column STCP (latch) — GPIO 5, clear of SPI pins
PIN_SENS_ROW_LOAD  = 7    # 74HC165 sensor row   SH/LD (parallel load)

PIN_LED_COL_LATCH  = 25   # 74HC595 LED column STCP
PIN_LED_ROW_LATCH  = 24   # 74HC595 LED row    STCP

PIN_BUZZER         = 18
PIN_BTN_MODE       = 23
PIN_BTN_CONFIRM    = 22
PIN_BTN_UNDO       = 27

# ─────────────────────────────────────────────────────────────────────────────
# TIMING
# ─────────────────────────────────────────────────────────────────────────────
COL_SETTLE_US      = 50      # µs for column line to settle before reading rows
LED_REFRESH_HZ     = 400     # per-column LED multiplex rate → full frame = 50 Hz
SCAN_FULL_INTERVAL = 0.020   # Full 8-column sensor scan every 20 ms
DEBOUNCE_COUNT     = 3       # Consecutive identical scans to confirm change

STOCKFISH_PATH     = '/usr/games/stockfish'
LICHESS_TOKEN      = 'your_lichess_token_here'
LICHESS_BASE       = 'https://lichess.org/api'
GAME_LOG_DIR       = '/home/pi/game_logs/'


# ─────────────────────────────────────────────────────────────────────────────
# LCD 1602 I2C (PCF8574 backpack, default address 0x27)
# ─────────────────────────────────────────────────────────────────────────────
class LCD:
    ADDR = 0x27
    BL   = 0x08   # backlight
    EN   = 0x04
    RS   = 0x01

    def __init__(self):
        self.bus = smbus2.SMBus(1)
        self._init()

    def _wb(self, d):
        self.bus.write_byte(self.ADDR, d | self.BL)
        time.sleep(0.0001)

    def _pulse(self, d):
        self._wb(d | self.EN); time.sleep(0.0005)
        self._wb(d & ~self.EN); time.sleep(0.0001)

    def _w4(self, d): self._wb(d); self._pulse(d)

    def _cmd(self, b, rs=0):
        mode = rs * self.RS
        self._w4(mode | (b & 0xF0))
        self._w4(mode | ((b << 4) & 0xF0))

    def _init(self):
        time.sleep(0.05)
        for _ in range(3): self._w4(0x30); time.sleep(0.005)
        self._w4(0x20); time.sleep(0.001)
        self._cmd(0x28); self._cmd(0x0C); self._cmd(0x06)
        self.clear()

    def clear(self):
        self._cmd(0x01); time.sleep(0.002)

    def pos(self, col, row):
        self._cmd(0x80 | ([0x00, 0x40][row] + col))

    def write(self, txt):
        for c in txt: self._cmd(ord(c), rs=1)

    def show(self, line1, line2=''):
        self.clear()
        self.pos(0, 0); self.write(str(line1)[:16].ljust(16))
        self.pos(0, 1); self.write(str(line2)[:16].ljust(16))


# ─────────────────────────────────────────────────────────────────────────────
# MATRIX SHIFT REGISTER DRIVER
# ─────────────────────────────────────────────────────────────────────────────
class MatrixDriver:
    """
    Controls all four shift registers for the 8×8 sensor and LED matrices.
    All I/O is bit-banged; the hardware SPI peripheral is NOT used.
    """

    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)

        for pin, direction, init in [
            (PIN_SCLK,           GPIO.OUT, GPIO.LOW),
            (PIN_MOSI,           GPIO.OUT, GPIO.LOW),
            (PIN_SENS_COL_LATCH, GPIO.OUT, GPIO.LOW),
            (PIN_SENS_ROW_LOAD,  GPIO.OUT, GPIO.HIGH),
            (PIN_LED_COL_LATCH,  GPIO.OUT, GPIO.LOW),
            (PIN_LED_ROW_LATCH,  GPIO.OUT, GPIO.LOW),
        ]:
            GPIO.setup(pin, direction, initial=init)

        GPIO.setup(PIN_MISO, GPIO.IN)

        # LED state: 8 bytes, one per column, each byte = row bitmask
        self._led_frame = [0x00] * 8   # led_frame[col] = row bitmask

        # Sensor state: 8×8 bool grid
        self._sensor_grid = [[False] * 8 for _ in range(8)]

        # LED multiplexing thread
        self._led_thread  = None
        self._led_running = False

    # ── LOW-LEVEL BIT BANG ────────────────────────────────────────────────────

    def _clock_pulse(self):
        GPIO.output(PIN_SCLK, GPIO.HIGH)
        GPIO.output(PIN_SCLK, GPIO.LOW)

    def _shift_out_byte(self, byte):
        """Clock out 8 bits MSB first on MOSI, toggling SCLK."""
        for i in range(7, -1, -1):
            GPIO.output(PIN_MOSI, (byte >> i) & 1)
            self._clock_pulse()

    def _shift_in_byte(self):
        """Clock in 8 bits MSB first from MISO (74HC165 already loaded)."""
        val = 0
        for i in range(7, -1, -1):
            GPIO.output(PIN_SCLK, GPIO.HIGH)
            if GPIO.input(PIN_MISO):
                val |= (1 << i)
            GPIO.output(PIN_SCLK, GPIO.LOW)
        return val

    def _latch(self, pin):
        GPIO.output(pin, GPIO.HIGH)
        GPIO.output(pin, GPIO.LOW)

    def _load_165(self):
        """Pulse SH/LD LOW to latch parallel inputs into 74HC165."""
        GPIO.output(PIN_SENS_ROW_LOAD, GPIO.LOW)
        GPIO.output(PIN_SENS_ROW_LOAD, GPIO.HIGH)

    # ── SENSOR MATRIX SCAN ───────────────────────────────────────────────────

    def scan_sensors(self):
        """
        Full 8-column scan. Updates internal _sensor_grid and returns it.
        grid[row][col]: True = piece present (magnet detected).
        row 0 = rank 1, col 0 = a-file.

        For each column c (0..7):
          1. Shift out column-select byte to Sensor-Col 595 (active-low).
          2. Latch Sensor-Col 595.
          3. Wait COL_SETTLE_US for column line to settle.
          4. Pulse SH/LD on 74HC165 to latch the 8 row inputs.
          5. Clock in 8 bits from 74HC165.
          6. LOW bit = piece present (A3144 active-low + pull-up).
        """
        grid = [[False] * 8 for _ in range(8)]

        for col in range(8):
            col_byte = 0xFF ^ (1 << col)          # active-low: only bit col = 0

            self._shift_out_byte(col_byte)
            self._latch(PIN_SENS_COL_LATCH)

            time.sleep(COL_SETTLE_US / 1_000_000)

            self._load_165()
            row_byte = self._shift_in_byte()

            for row in range(8):
                grid[row][col] = not bool(row_byte & (1 << (7 - row)))

        # Deactivate all columns
        self._shift_out_byte(0xFF)
        self._latch(PIN_SENS_COL_LATCH)

        self._sensor_grid = grid
        return grid

    def get_occupied_set(self):
        """
        Returns set of chess square indices (0-63) that are occupied.
        Square index = row * 8 + col  (row=rank-1, col=file index a=0..h=7)
        """
        occupied = set()
        for row in range(8):
            for col in range(8):
                if self._sensor_grid[row][col]:
                    occupied.add(row * 8 + col)
        return occupied

    # ── LED MATRIX CONTROL ───────────────────────────────────────────────────

    def set_led_squares(self, squares):
        """
        squares: set/list of square indices (0-63) to illuminate.
        Updates internal frame buffer; multiplex thread picks it up.
        """
        frame = [0x00] * 8
        for sq in squares:
            if 0 <= sq < 64:
                row = sq // 8
                col = sq % 8
                frame[col] |= (1 << row)
        self._led_frame = frame

    def clear_leds(self):
        self._led_frame = [0x00] * 8
        self._write_led_col_row(0x00, 0x00)

    def _write_led_col_row(self, col_byte, row_byte):
        """
        Shift out LED column and row bytes to their respective 74HC595s.
        Chain order (MOSI → Sensor-Col → LED-Col → LED-Row):
          Shift 3 bytes: [0xFF (sensor-col dummy), led_col, led_row]
          Latch LED-Col and LED-Row selectively; sensor-col NOT latched.
        """
        self._shift_out_byte(0xFF)       # sensor-col (not latched, so harmless)
        self._shift_out_byte(col_byte)   # led-col
        self._shift_out_byte(row_byte)   # led-row

        self._latch(PIN_LED_COL_LATCH)
        self._latch(PIN_LED_ROW_LATCH)

    def start_led_multiplex(self):
        """Start background thread that cycles through LED columns."""
        self._led_running = True
        self._led_thread = threading.Thread(
            target=self._led_loop, daemon=True)
        self._led_thread.start()

    def stop_led_multiplex(self):
        self._led_running = False
        if self._led_thread:
            self._led_thread.join(timeout=0.5)
        self.clear_leds()

    def _led_loop(self):
        """
        Multiplexes LEDs at LED_REFRESH_HZ (per-column).
        Full frame rate = LED_REFRESH_HZ / 8.

        Hardware polarity (BC327 PNP common-anode per column):
          col_byte: bit n LOW  → BC327 ON → column anode bus = 5V (column active)
          row_byte: bit n LOW  → LED cathode sunk to GND → LED ON
          Frame buffer uses 1=ON convention; we invert here for hardware.

        FIX: removed the dead first col_period assignment that was immediately
        overwritten. Only the correct formula is kept.
        """
        col_period = 1.0 / (LED_REFRESH_HZ * 8)   # ~312 µs per column slot

        while self._led_running:
            frame = self._led_frame   # snapshot (GIL protects assignment)
            for col in range(8):
                row_mask = frame[col]
                if row_mask:
                    col_byte = 0xFF ^ (1 << col)    # active-low column select
                    row_byte = (~row_mask) & 0xFF   # invert: 1=ON → LOW=ON for PNP
                    self._write_led_col_row(col_byte, row_byte)
                else:
                    self._write_led_col_row(0xFF, 0xFF)   # blank column
                time.sleep(col_period)

    # ── FLASH / ANIMATE ──────────────────────────────────────────────────────

    def flash(self, squares, times=3, on_ms=250, off_ms=200):
        """Flash a set of squares. Blocks (call from main thread)."""
        for _ in range(times):
            self.set_led_squares(squares)
            time.sleep(on_ms / 1000)
            self.clear_leds()
            time.sleep(off_ms / 1000)

    def animate_move(self, from_sq, to_sq, duration=0.6):
        """Show from→to, then just to, then clear."""
        self.set_led_squares({from_sq, to_sq})
        time.sleep(duration * 0.6)
        self.set_led_squares({to_sq})
        time.sleep(duration * 0.4)
        self.clear_leds()

    def legal_move_highlight(self, board, from_sq_idx):
        """Light up from_sq and all legal destinations."""
        sq    = chess.Square(from_sq_idx)
        dests = {int(m.to_square) for m in board.legal_moves
                 if m.from_square == sq}
        self.set_led_squares(dests | {from_sq_idx})
        return dests

    def close(self):
        self.stop_led_multiplex()
        GPIO.cleanup()


# ─────────────────────────────────────────────────────────────────────────────
# SENSOR DEBOUNCER
# ─────────────────────────────────────────────────────────────────────────────
class Debouncer:
    def __init__(self, threshold=DEBOUNCE_COUNT):
        self.threshold = threshold
        self.confirmed = set()   # confirmed occupied squares
        self.pending   = {}      # sq → consecutive count differing from confirmed

    def update(self, raw_occupied):
        """
        raw_occupied: set of square indices currently sensed.
        Returns (confirmed_frozenset, newly_added_list, newly_removed_list).
        """
        added   = []
        removed = []

        # Squares appearing
        for sq in raw_occupied:
            if sq not in self.confirmed:
                self.pending[sq] = self.pending.get(sq, 0) + 1
                if self.pending[sq] >= self.threshold:
                    self.confirmed.add(sq)
                    del self.pending[sq]
                    added.append(sq)
            else:
                self.pending.pop(sq, None)

        # Squares disappearing
        for sq in list(self.confirmed):
            if sq not in raw_occupied:
                key = ~sq   # negative key distinguishes disappearing from appearing
                self.pending[key] = self.pending.get(key, 0) + 1
                if self.pending[key] >= self.threshold:
                    self.confirmed.discard(sq)
                    del self.pending[key]
                    removed.append(sq)
            else:
                self.pending.pop(~sq, None)

        # Prune stale pending entries for already-confirmed squares
        for sq in list(self.pending):
            if sq >= 0 and sq in self.confirmed:
                del self.pending[sq]

        return frozenset(self.confirmed), added, removed


# ─────────────────────────────────────────────────────────────────────────────
# BUZZER
# ─────────────────────────────────────────────────────────────────────────────
class Buzzer:
    """
    Driver for a 5V active buzzer (e.g. the one in the cart).
    Active buzzers have a built-in oscillator — they emit a fixed tone
    when power is applied. PWM frequency control does nothing on them,
    so we use plain GPIO on/off pulses instead.
    Distinct patterns are created by varying on-time and number of beeps.
    """

    def __init__(self):
        GPIO.setup(PIN_BUZZER, GPIO.OUT, initial=GPIO.LOW)

    def _beep(self, ms):
        GPIO.output(PIN_BUZZER, GPIO.HIGH)
        time.sleep(ms / 1000)
        GPIO.output(PIN_BUZZER, GPIO.LOW)

    def move(self):    self._beep(80)
    def capture(self): self._beep(200)
    def illegal(self): self._beep(500)
    def undo(self):    self._beep(100)

    def check(self):
        self._beep(80)
        time.sleep(0.06)
        self._beep(80)

    def startup(self):
        for _ in range(3):
            self._beep(60)
            time.sleep(0.05)

    def victory(self):
        for _ in range(4):
            self._beep(100)
            time.sleep(0.04)


# ─────────────────────────────────────────────────────────────────────────────
# MOVE RECORDER
# ─────────────────────────────────────────────────────────────────────────────
class MoveRecorder:
    def __init__(self):
        os.makedirs(GAME_LOG_DIR, exist_ok=True)
        self.moves = []
        self.t0    = time.time()
        ts         = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.path  = GAME_LOG_DIR + ts + '.json'

    def record(self, move, board):
        """
        Record a move.  MUST be called BEFORE board.push(move) so that
        board.san(move) can access the correct pre-move board state.
        """
        self.moves.append({
            'move': move.uci(),
            'san' : board.san(move),
            'fen' : board.fen(),
            'sec' : round(time.time() - self.t0, 2)
        })
        self._save()

    def _pgn(self):
        b      = chess.Board()
        tokens = []
        for i, e in enumerate(self.moves):
            try:
                m = chess.Move.from_uci(e['move'])
                if i % 2 == 0: tokens.append(f"{i // 2 + 1}.")
                tokens.append(b.san(m))
                b.push(m)
            except Exception:
                pass
        return ' '.join(tokens)

    def _save(self):
        with open(self.path, 'w') as f:
            json.dump({'moves': self.moves, 'pgn': self._pgn()}, f, indent=2)

    def get_pgn(self): return self._pgn()


# ─────────────────────────────────────────────────────────────────────────────
# STOCKFISH WRAPPER
# ─────────────────────────────────────────────────────────────────────────────
class Stockfish:
    def __init__(self, skill=5, time_limit=1.5):
        self.skill  = skill
        self.tlim   = time_limit
        self.engine = None

    def start(self):
        try:
            self.engine = chess.engine.SimpleEngine.popen_uci(STOCKFISH_PATH)
            self.engine.configure({'Skill Level': self.skill})
            log.info(f"Stockfish OK (skill {self.skill})")
            return True
        except Exception as e:
            log.error(f"Stockfish start failed: {e}")
            return False

    def best_move(self, board):
        if not self.engine: return None
        try:
            r = self.engine.play(board, chess.engine.Limit(time=self.tlim))
            return r.move
        except Exception as e:
            log.error(f"Stockfish error: {e}"); return None

    def stop(self):
        if self.engine:
            try: self.engine.quit()
            except: pass
            self.engine = None


# ─────────────────────────────────────────────────────────────────────────────
# LICHESS CLIENT
# ─────────────────────────────────────────────────────────────────────────────
class Lichess:
    def __init__(self):
        self.hdrs    = {'Authorization': f'Bearer {LICHESS_TOKEN}'}
        self.game_id = None

    def create_open_challenge(self, clock=300, inc=3):
        r = requests.post(
            f'{LICHESS_BASE}/challenge/open',
            headers=self.hdrs,
            data={'rated': 'false', 'clock.limit': clock,
                  'clock.increment': inc, 'variant': 'standard'},
            timeout=10)
        if r.ok:
            j = r.json()
            self.game_id = j['challenge']['id']
            return self.game_id, j['challenge'].get('url', '')
        log.error(f"Lichess challenge failed: {r.text}")
        return None, None

    def send_move(self, uci):
        if not self.game_id: return False
        r = requests.post(
            f'{LICHESS_BASE}/board/game/{self.game_id}/move/{uci}',
            headers=self.hdrs, timeout=5)
        return r.ok

    def stream_moves(self):
        """Generator yielding UCI move-list arrays from the Lichess game stream."""
        if not self.game_id: return
        try:
            r = requests.get(
                f'{LICHESS_BASE}/board/game/stream/{self.game_id}',
                headers=self.hdrs, stream=True, timeout=60)
            for line in r.iter_lines():
                if line:
                    try:
                        d = json.loads(line)
                        if d.get('type') in ('gameState', 'gameFull'):
                            moves_str = (d.get('moves', '') or
                                         d.get('state', {}).get('moves', ''))
                            if moves_str:
                                yield moves_str.split()
                    except Exception:
                        pass
        except Exception as e:
            log.error(f"Lichess stream: {e}")


# ─────────────────────────────────────────────────────────────────────────────
# GAME MODES
# ─────────────────────────────────────────────────────────────────────────────
MODE_2P   = 0
MODE_SF   = 1
MODE_LC   = 2
MODE_NAME = ['2-Player', 'Stockfish', 'Lichess']


# ─────────────────────────────────────────────────────────────────────────────
# MAIN CONTROLLER
# ─────────────────────────────────────────────────────────────────────────────
class ChessboardController:

    def __init__(self):
        self.matrix  = MatrixDriver()
        self.lcd     = LCD()
        self.buzzer  = Buzzer()
        self.sf      = Stockfish()
        self.lichess = Lichess()

        for pin in [PIN_BTN_MODE, PIN_BTN_CONFIRM, PIN_BTN_UNDO]:
            GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        self.board      = chess.Board()
        self.mode       = MODE_2P
        self.recorder   = MoveRecorder()
        self.debouncer  = Debouncer()
        self.running    = True

        self.state         = 'IDLE'    # IDLE | PIECE_LIFTED
        self.lifted_sq     = None
        self.prev_occupied = frozenset()

        self.matrix.start_led_multiplex()
        log.info("Controller ready")

    # ── BUTTONS ──────────────────────────────────────────────────────────────

    def _pressed(self, pin):
        return GPIO.input(pin) == GPIO.LOW

    def _wait_release(self, pin, timeout=2.0):
        t = time.time()
        while GPIO.input(pin) == GPIO.LOW:
            if time.time() - t > timeout: break
            time.sleep(0.01)

    def _check_buttons(self):
        """Returns 'undo', 'new_game', or None."""
        if self._pressed(PIN_BTN_UNDO):
            self._wait_release(PIN_BTN_UNDO)
            return 'undo'
        if self._pressed(PIN_BTN_CONFIRM):
            t = time.time()
            while self._pressed(PIN_BTN_CONFIRM):
                if time.time() - t > 2.0:
                    self._wait_release(PIN_BTN_CONFIRM)
                    return 'new_game'
                time.sleep(0.05)
            return None
        return None

    # ── STARTUP & MODE SELECT ─────────────────────────────────────────────────

    def run(self):
        self.buzzer.startup()
        self.lcd.show('Smart Chess', 'v1.0')
        time.sleep(1)
        self._mode_select()

    def _mode_select(self):
        self.lcd.show('Mode: ' + MODE_NAME[self.mode], 'MODE/OK to start')
        while True:
            if self._pressed(PIN_BTN_MODE):
                self.mode = (self.mode + 1) % 3
                self.lcd.show('Mode: ' + MODE_NAME[self.mode], 'MODE/OK to start')
                self._wait_release(PIN_BTN_MODE)
                time.sleep(0.1)
            if self._pressed(PIN_BTN_CONFIRM):
                self._wait_release(PIN_BTN_CONFIRM)
                self._start_game()
                return
            time.sleep(0.05)

    def _start_game(self):
        self.board     = chess.Board()
        self.recorder  = MoveRecorder()
        self.debouncer = Debouncer()
        self.state     = 'IDLE'
        self.lifted_sq = None

        self.matrix.scan_sensors()
        raw_occ = self.matrix.get_occupied_set()
        self.prev_occupied, _, _ = self.debouncer.update(raw_occ)

        if self.mode == MODE_SF:
            if not self.sf.start():
                self.lcd.show('Stockfish ERR', 'Check /usr/games')
                time.sleep(3); self._mode_select(); return
            self.lcd.show('Stockfish', 'You = White')

        elif self.mode == MODE_LC:
            self.lcd.show('Lichess', 'Creating game..')
            gid, url = self.lichess.create_open_challenge()
            if not gid:
                self.lcd.show('Lichess ERR', 'Token/WiFi?')
                time.sleep(3); self._mode_select(); return
            display_url = url.replace('https://lichess.org/', 'lichess.org/')
            self.lcd.show('Join:', display_url[-16:])
            log.info(f"Lichess game: {url}")
            time.sleep(4)

        else:
            self.lcd.show('2-Player', 'White to move')

        time.sleep(1)
        self._game_loop()

    # ── GAME LOOP ─────────────────────────────────────────────────────────────

    def _game_loop(self):
        while self.running:
            btn = self._check_buttons()
            if btn == 'undo':
                self._do_undo(); continue
            if btn == 'new_game':
                self.matrix.clear_leds()
                if self.mode == MODE_SF: self.sf.stop()
                self._mode_select(); return

            if self.board.is_game_over():
                self._game_over(); return

            self.matrix.scan_sensors()
            raw_occ = self.matrix.get_occupied_set()
            confirmed, added, removed = self.debouncer.update(raw_occ)

            turn = self.board.turn

            if self.mode == MODE_SF and turn == chess.BLACK:
                self._stockfish_turn()
                continue

            if self.mode == MODE_LC and turn == chess.BLACK:
                self._lichess_opponent_turn()
                continue

            self._detect_human_move(confirmed, removed, added)
            time.sleep(SCAN_FULL_INTERVAL)

    # ── HUMAN MOVE STATE MACHINE ──────────────────────────────────────────────

    def _detect_human_move(self, confirmed, removed, added):
        """
        Two-phase: IDLE → PIECE_LIFTED (piece removed) → move made (piece placed).

        FIX: removed the redundant double-check of `lifted_sq in confirmed`
        (the outer if-block already implies the condition; the dead inner if
        was replaced with a direct assignment + return).
        """
        if self.state == 'IDLE':
            if len(removed) == 1:
                sq    = removed[0]
                piece = self.board.piece_at(chess.Square(sq))
                if piece and piece.color == self.board.turn:
                    self.state     = 'PIECE_LIFTED'
                    self.lifted_sq = sq
                    dests = self.matrix.legal_move_highlight(self.board, sq)
                    log.info(f"Lifted {chess.square_name(sq)}, {len(dests)} legal moves")

        elif self.state == 'PIECE_LIFTED':
            # Piece returned to original square → cancel lift
            if self.lifted_sq in confirmed:
                self.state     = 'IDLE'
                self.lifted_sq = None
                self.matrix.clear_leds()
                self.prev_occupied = frozenset(confirmed)
                return

            # New square occupied → attempt move
            if len(added) >= 1:
                from_sq     = chess.Square(self.lifted_sq)
                legal_dests = {int(m.to_square) for m in self.board.legal_moves
                               if m.from_square == from_sq}
                for to_idx in added:
                    if to_idx in legal_dests:
                        self._apply_move(self.lifted_sq, to_idx)
                        return
                # Piece placed on an illegal square
                for to_idx in added:
                    if to_idx != self.lifted_sq:
                        self.buzzer.illegal()
                        self.matrix.flash({self.lifted_sq} | set(added),
                                          times=2, on_ms=150, off_ms=150)
                        self.matrix.legal_move_highlight(self.board, self.lifted_sq)
                        return

        self.prev_occupied = frozenset(confirmed)

    def _apply_move(self, from_idx, to_idx):
        from_sq = chess.Square(from_idx)
        to_sq   = chess.Square(to_idx)

        move  = chess.Move(from_sq, to_sq)
        piece = self.board.piece_at(from_sq)
        if piece and piece.piece_type == chess.PAWN:
            if chess.square_rank(to_sq) in (0, 7):
                move = chess.Move(from_sq, to_sq, promotion=chess.QUEEN)

        if move not in self.board.legal_moves:
            self.buzzer.illegal()
            return

        is_cap   = self.board.is_capture(move)
        is_check = self.board.gives_check(move)

        # FIX: record() BEFORE push() so board.san() sees the pre-move position
        self.recorder.record(move, self.board)
        self.board.push(move)

        self.matrix.animate_move(from_idx, to_idx)

        if is_check:   self.buzzer.check()
        elif is_cap:   self.buzzer.capture()
        else:          self.buzzer.move()

        self.state     = 'IDLE'
        self.lifted_sq = None

        turn_name = 'Black' if self.board.turn == chess.BLACK else 'White'
        self.lcd.show(f"{turn_name} to move", f"#{self.board.fullmove_number} {move.uci()}")

        if self.mode == MODE_LC:
            self.lichess.send_move(move.uci())

        log.info(f"Move: {move.uci()}")

    # ── STOCKFISH TURN ────────────────────────────────────────────────────────

    def _stockfish_turn(self):
        self.lcd.show('Stockfish', 'thinking...')
        move = self.sf.best_move(self.board)
        if not move:
            self.lcd.show('SF error', 'No move found')
            time.sleep(2)
            return

        from_idx = int(move.from_square)
        to_idx   = int(move.to_square)
        sq_name  = chess.square_name(move.from_square) + chess.square_name(move.to_square)
        self.lcd.show('SF plays:', sq_name)
        log.info(f"Stockfish: {move.uci()}")

        guided = self._guide_opponent_move(move)
        if not guided:
            log.warning("Stockfish move guide timed out; forcing move anyway")

        is_cap   = self.board.is_capture(move)
        is_check = self.board.gives_check(move)

        # FIX: record() BEFORE push() so board.san() sees the pre-move position
        self.recorder.record(move, self.board)
        self.board.push(move)

        self.matrix.animate_move(from_idx, to_idx)

        if is_check: self.buzzer.check()
        elif is_cap: self.buzzer.capture()
        else:        self.buzzer.move()

        self.lcd.show('Your turn', f"#{self.board.fullmove_number}")

    def _guide_opponent_move(self, move, timeout=90):
        """
        Flash from→to until player physically makes the move, or timeout.
        Returns True if the move was completed (or CONFIRM pressed),
        False if the timeout elapsed.

        FIX: return value is now meaningful and checked by callers.
        """
        from_idx = int(move.from_square)
        to_idx   = int(move.to_square)
        deadline = time.time() + timeout
        phase    = 0

        while time.time() < deadline and self.running:
            if phase % 6 < 3:
                self.matrix.set_led_squares({from_idx})
            else:
                self.matrix.set_led_squares({from_idx, to_idx})
            phase += 1

            self.matrix.scan_sensors()
            raw = self.matrix.get_occupied_set()

            if from_idx not in raw and to_idx in raw:
                self.matrix.clear_leds()
                return True

            if self._pressed(PIN_BTN_CONFIRM):
                self._wait_release(PIN_BTN_CONFIRM)
                self.matrix.clear_leds()
                return True

            time.sleep(0.15)

        self.matrix.clear_leds()
        return False   # timed out

    # ── LICHESS OPPONENT TURN ─────────────────────────────────────────────────

    def _lichess_opponent_turn(self):
        """Poll Lichess for opponent's move then guide player to make it."""
        self.lcd.show('Lichess', 'Waiting opp...')
        known_moves = len(self.board.move_stack)

        deadline = time.time() + 120
        while time.time() < deadline and self.running:
            for move_list in self.lichess.stream_moves():
                if len(move_list) > known_moves:
                    uci = move_list[known_moves]
                    try:
                        move = chess.Move.from_uci(uci)
                        if move in self.board.legal_moves:
                            from_idx = int(move.from_square)
                            to_idx   = int(move.to_square)
                            self.lcd.show('Opp moves:', uci)

                            guided = self._guide_opponent_move(move)
                            if not guided:
                                log.warning("Lichess opponent move guide timed out; forcing")

                            is_cap   = self.board.is_capture(move)
                            is_check = self.board.gives_check(move)

                            # FIX: record() BEFORE push()
                            self.recorder.record(move, self.board)
                            self.board.push(move)

                            self.matrix.animate_move(from_idx, to_idx)
                            if is_check: self.buzzer.check()
                            elif is_cap: self.buzzer.capture()
                            else:        self.buzzer.move()
                            self.lcd.show('Your turn', f"#{self.board.fullmove_number}")
                            return
                    except Exception as e:
                        log.error(f"Lichess move parse: {e}")
            time.sleep(2)

        self.lcd.show('Lichess timeout', 'No move received')
        time.sleep(2)

    # ── UNDO ─────────────────────────────────────────────────────────────────

    def _do_undo(self):
        if not self.board.move_stack:
            self.lcd.show('Nothing', 'to undo')
            self.buzzer.illegal()
            time.sleep(1)
            return
        self.board.pop()
        if self.mode == MODE_SF and self.board.move_stack:
            self.board.pop()   # undo Stockfish's reply too
        self.state     = 'IDLE'
        self.lifted_sq = None
        self.matrix.clear_leds()
        self.debouncer = Debouncer()
        self.buzzer.undo()
        turn = 'White' if self.board.turn == chess.WHITE else 'Black'
        self.lcd.show('Undone', f'{turn} to move')
        time.sleep(1)

    # ── GAME OVER ─────────────────────────────────────────────────────────────

    def _game_over(self):
        result = self.board.result()
        if self.board.is_checkmate():
            winner = 'Black' if self.board.turn == chess.WHITE else 'White'
            msg = f'{winner} wins!'
        elif self.board.is_stalemate():             msg = 'Stalemate'
        elif self.board.is_insufficient_material(): msg = 'Draw (material)'
        elif self.board.is_fifty_moves():           msg = 'Draw (50 moves)'
        else:                                       msg = f'Draw {result}'

        self.lcd.show(msg, self.recorder.get_pgn()[-16:])
        self.buzzer.victory()
        log.info(f"Game over: {msg} | PGN: {self.recorder.get_pgn()}")

        for _ in range(6):
            self.matrix.set_led_squares(set(range(64)))
            time.sleep(0.25)
            self.matrix.clear_leds()
            time.sleep(0.2)

        time.sleep(3)
        if self.mode == MODE_SF: self.sf.stop()
        self._mode_select()

    # ── SHUTDOWN ──────────────────────────────────────────────────────────────

    def shutdown(self):
        self.running = False
        self.matrix.close()
        self.lcd.show('Goodbye!', '')
        if self.mode == MODE_SF: self.sf.stop()
        log.info("Shutdown complete")


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────
def main():
    """
    FIX: removed unreachable `while ctrl.running` loop that appeared after
    ctrl.run(). ctrl.run() blocks inside _game_loop() for the lifetime of the
    program; the while-loop would only be reached after the game finishes, at
    which point ctrl.running is already False.
    """
    ctrl = ChessboardController()
    try:
        ctrl.run()
    except KeyboardInterrupt:
        log.info("Interrupted")
    finally:
        ctrl.shutdown()


if __name__ == '__main__':
    main()
