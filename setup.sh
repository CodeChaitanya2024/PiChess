#!/bin/bash
# Smart Chessboard Setup Script
# Run as: sudo bash setup.sh
#
# FIXES vs original:
#   1. Added --break-system-packages to pip3 install (required on Pi OS Bookworm+)
#   2. Removed duplicate `python-chess` package — `chess` IS python-chess;
#      installing both causes version conflicts
#   3. Replaced hardcoded `pi` user with $INSTALL_USER (auto-detected from
#      $SUDO_USER, falling back to `pi` for older images)
#   4. Wrapped apt-get upgrade in a warning-only block so a network failure
#      doesn't abort the whole setup
#   5. Added --no-warn-script-location to silence pip noise on PATH

set -e
echo "=== Smart Chessboard Setup ==="

# ─── Detect target user ──────────────────────────────────────────────────────
# On newer Pi OS images the default user may not be `pi`.
# $SUDO_USER is set to the user who invoked sudo.
INSTALL_USER="${SUDO_USER:-pi}"
INSTALL_HOME="/home/${INSTALL_USER}"
echo "Installing for user: ${INSTALL_USER} (home: ${INSTALL_HOME})"

# ─── System update ───────────────────────────────────────────────────────────
apt-get update -y

# Upgrade is best-effort; don't abort setup if it fails (e.g. network issues)
apt-get upgrade -y || echo "WARNING: apt-get upgrade failed — continuing anyway"

# ─── Enable I2C; disable SPI ─────────────────────────────────────────────────
# I2C is required for the LCD1602 display.
# SPI must be DISABLED. Enabling SPI (do_spi 0) gives the OS hard ownership of
# GPIO 8 (CE0), 9, 10, and 11. Our code bit-bangs those pins directly for the
# shift-register chain; if the SPI peripheral owns them, GPIO 8 (sensor-column
# latch) cannot be toggled from userspace and the sensor matrix stops working.
raspi-config nonint do_spi 1    # 1 = disable SPI peripheral
raspi-config nonint do_i2c 0    # 0 = enable I2C
echo "I2C enabled, SPI disabled"

# ─── Install system packages ─────────────────────────────────────────────────
apt-get install -y \
    python3-pip \
    python3-dev \
    python3-smbus \
    i2c-tools \
    stockfish \
    git \
    libatlas-base-dev

# ─── Install Python packages ─────────────────────────────────────────────────
# Notes:
#   - `python-chess` IS the `chess` package (same package, different install name).
#     Installing both would cause a conflict. Only `python-chess` is listed here
#     because it is the canonical PyPI name.
#   - `spidev` is intentionally NOT installed — the chessboard uses bit-banged
#     GPIO only, and importing spidev would claim GPIO 8 (CE0) for the SPI
#     hardware peripheral, conflicting with our sensor-column latch pin.
#   - --break-system-packages is required on Raspberry Pi OS Bookworm (and later)
#     which uses externally-managed Python environments.
pip3 install --break-system-packages --no-warn-script-location \
    RPi.GPIO \
    smbus2 \
    python-chess \
    requests

# ─── Create game log directory ───────────────────────────────────────────────
mkdir -p "${INSTALL_HOME}/game_logs"
chown "${INSTALL_USER}:${INSTALL_USER}" "${INSTALL_HOME}/game_logs"

# ─── Copy chessboard code ────────────────────────────────────────────────────
cp chessboard.py "${INSTALL_HOME}/chessboard.py"
chown "${INSTALL_USER}:${INSTALL_USER}" "${INSTALL_HOME}/chessboard.py"

# ─── Create systemd service ──────────────────────────────────────────────────
cat > /etc/systemd/system/chessboard.service << EOF
[Unit]
Description=Smart Chessboard
After=network.target

[Service]
ExecStart=/usr/bin/python3 ${INSTALL_HOME}/chessboard.py
WorkingDirectory=${INSTALL_HOME}
User=${INSTALL_USER}
Restart=on-failure
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

systemctl daemon-reload
systemctl enable chessboard.service
echo "Systemd service installed (user: ${INSTALL_USER})"

# ─── Verify stockfish ────────────────────────────────────────────────────────
if [ -f /usr/games/stockfish ]; then
    echo "Stockfish found at /usr/games/stockfish"
else
    echo "WARNING: Stockfish not found at /usr/games/stockfish."
    echo "         Install manually or update STOCKFISH_PATH in chessboard.py"
fi

echo ""
echo "=== Setup complete ==="
echo "Edit ${INSTALL_HOME}/chessboard.py and set your LICHESS_TOKEN"
echo "Start with: sudo systemctl start chessboard"
echo "Or manually: python3 ${INSTALL_HOME}/chessboard.py"
