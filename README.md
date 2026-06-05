# Smart Chessboard

A Raspberry Pi Zero W–powered smart chessboard with an 8×8 Hall effect sensor matrix, LED matrix, LCD display, piezo buzzer, and support for Stockfish AI and Lichess online play.

---

## Hardware

| Component | Detail |
|---|---|
| MCU | Raspberry Pi Zero W |
| Sensors | 64× A3144 Hall effect sensors (8×8) |
| Shift registers | 3× 74HC595 (LED col/row + sensor col), 1× 74HC165 (sensor row read) |
| Display | LCD1602 via I2C (PCF8574 backpack, address `0x27`) |
| LEDs | 64× LEDs, common-anode per column (BC327 PNP drivers) |
| Buzzer | Active piezo on GPIO 18 |
| Buttons | MODE (GPIO 23), CONFIRM (GPIO 22), UNDO (GPIO 27) |

### GPIO Pinout (BCM)

| GPIO | Function |
|---|---|
| 10 (MOSI) | Sensor-col 74HC595 data in |
| 11 (SCLK) | Shared bit-bang clock (all shift registers) |
| 9 (MISO) | 74HC165 Q7 serial data out |
| 7 | 74HC165 SH/LD (parallel load) |
| 5 | Sensor-col 74HC595 STCP (latch) |
| 25 | LED-col 74HC595 STCP (latch) |
| 24 | LED-row 74HC595 STCP (latch) |
| 2 (SDA) | LCD1602 I2C SDA |
| 3 (SCL) | LCD1602 I2C SCL |
| 18 | Piezo buzzer |
| 23 | Button MODE |
| 22 | Button CONFIRM |
| 27 | Button UNDO |

> **Note:** SPI peripheral must be **disabled**. The code bit-bangs GPIO 9/10/11 directly; enabling the SPI peripheral gives the OS hard ownership of those pins and breaks the sensor matrix.

---

## Setup

### 1. Clone the repo

```bash
git clone https://github.com/your-username/smart-chessboard.git
cd smart-chessboard
```

### 2. Run the setup script

```bash
sudo bash setup.sh
```

This will:
- Enable I2C, disable SPI via `raspi-config`
- Install system packages (`stockfish`, `i2c-tools`, etc.)
- Install Python dependencies
- Create the `game_logs/` directory
- Install and enable a `systemd` service

### 3. Configure your Lichess token

Copy the example env file and add your token:

```bash
cp .env.example .env
nano .env
```

Set `LICHESS_TOKEN` to your token from [lichess.org/account/oauth/token](https://lichess.org/account/oauth/token).

The application reads it via:
```python
import os
LICHESS_TOKEN = os.environ.get('LICHESS_TOKEN', '')
```

To pass the variable to the systemd service, add this to `/etc/systemd/system/chessboard.service` under `[Service]`:
```ini
EnvironmentFile=/home/pi/.env
```

Then reload:
```bash
sudo systemctl daemon-reload
sudo systemctl restart chessboard
```

---

## Running

**Via systemd (auto-start on boot):**
```bash
sudo systemctl start chessboard
sudo systemctl status chessboard
```

**Manually:**
```bash
python3 chessboard.py
```

**View logs:**
```bash
tail -f ~/chessboard.log
```

---

## Game Modes

| Mode | Description |
|---|---|
| Local 2-player | Two players on the physical board |
| Stockfish | Play against the Stockfish engine |
| Lichess | Play against an online opponent via Lichess API |

Use the **MODE** button to select, **CONFIRM** to accept, **UNDO** to take back a move.

---

## Dependencies

See [`requirements.txt`](requirements.txt). Install manually with:

```bash
pip3 install --break-system-packages -r requirements.txt
```

---

## License

Apache 2.0
