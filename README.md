# SX126xInterface for Reticulum

A custom Reticulum interface module that drives SX1262/SX1268 LoRa chips **directly over SPI** on a Raspberry Pi (or other Linux SBC). No separate RNode microcontroller needed — the Pi *is* the RNode.

Designed for the **MeshAdv Pi HAT v1.1** but works with any SPI-connected SX126x board with configurable pin mapping.

Updated to work with the Femtofox and any of the sx1262 boards.

## What This Does

Normally, Reticulum requires a separate RNode device (an ESP32 or nRF52 running RNode firmware) connected via USB serial. This interface eliminates that second device by talking directly to the SX1262 LoRa chip over the Pi's SPI bus.

It implements:
- **Direct SPI control** via a vendored SX126x driver on raw spidev + libgpiod 1.6
- **Two-stage board / SBC profile selection** — pick your SBC (`platform`) and your HAT (`radio_board`) independently, and the interface resolves which `(gpiochip, line_offset)` pair each physical header pin maps to on your specific SBC
- **CSMA/CA** with **CAD-based** carrier sense (replaces the old stale-RSSI check)
- **RNode-compatible split-packet framing** so the full 500-byte Reticulum MTU works over the 255-byte LoRa frame limit
- **Airtime tracking and limiting** with sliding-window accounting (no spike-after-reset artifact)
- **Radio lockup recovery** (escalating reinit, then offline)
- **Interoperability** with standard RNodes on the same frequency/parameters

## Pin / GPIO Resolution

Two-axis model so SBC and radio board can evolve independently:

- **`platform`** — describes the SBC (gpiochip device, default SPI bus, and a `header_pin_to_line` table that maps *physical 40-pin-header pin numbers* to `(gpiochip, line_offset)` for this specific SBC). Bundled: `raspberry-pi`, `luckfox-pico`.
- **`radio_board`** — describes the radio board's wiring in terms of physical 40-pin header pin numbers (portable across SBCs that share the same header layout), plus radio-electronics fields (TCXO voltage, RF switch, etc.). Bundled: `meshadv-pi-hat-v1.1`, `femtofox-integrated-v1`, `generic-sx1262-manual`.

Resolution: for each pin the board profile specifies, the platform's `header_pin_to_line` is consulted to get the `(gpiochip, line_offset)` tuple handed to the vendored driver. The startup NOTICE log prints every final resolved pin pair so silent misconfigurations are easy to spot.

**Escape hatch** — `radio_board = custom` bypasses the profile system: you provide `gpiochip` and `pin_*` values directly as gpiochip line offsets.

## Requirements

- Raspberry Pi (2/3/4/5, Zero 2W) with 40-pin GPIO header
- SX1262-based LoRa HAT (MeshAdv Pi HAT, or similar)
- Raspberry Pi OS (Bookworm or later recommended)
- Python 3.9+
- SPI enabled on the Pi

## Installation

### 1. Enable SPI on the Pi

```bash
sudo raspi-config nonint do_spi 0
```

Reboot if SPI wasn't already enabled.

### 2. Install dependencies

```bash
pip install rns
```

The driver depends only on stdlib Python plus `spidev` and `gpiod`
(provided by Debian/Ubuntu packages `python3-spidev` and `python3-libgpiod`).
Install those via your OS package manager:

```bash
sudo apt install python3-spidev python3-libgpiod
```

### 3. Install the interface module

Copy **both** `SX126xInterface.py` and `vendored_sx126x.py` to the
Reticulum custom interfaces directory:

```bash
mkdir -p ~/.reticulum/interfaces
cp SX126xInterface.py vendored_sx126x.py ~/.reticulum/interfaces/
```

### 4. Configure Reticulum

Edit `~/.reticulum/config` and add an interface block. If the file doesn't exist yet, run `rnsd` once to generate it, then edit.

#### Profile-based config (recommended) — Pi + MeshAdv Pi HAT

```ini
[interfaces]

  [[MeshAdv LoRa]]
    type = SX126xInterface
    interface_enabled = True
    frequency = 915000000
    bandwidth = 125000
    spreadingfactor = 8
    codingrate = 5
    txpower = 22
    # Profile selection (the two new keys)
    platform    = raspberry-pi
    radio_board = meshadv-pi-hat-v1.1
    # TCXO voltage for E22 module (profile default is 1.8; override if needed)
    dio3_tcxo_voltage = 1.8
```

#### Profile-based config — Femtofox's integrated SX1262 hat

```ini
[interfaces]

  [[Femtofox LoRa]]
    type = SX126xInterface
    interface_enabled = True
    frequency = 915000000
    bandwidth = 125000
    spreadingfactor = 8
    codingrate = 5
    txpower = 22
    platform    = luckfox-pico
    radio_board = femtofox-integrated-v1
```

#### Profile-based config — BQ/Uniteng Station G3 (Pi Zero 2W daughterboard)

**Hardware-verified** (2026-08-12): SPI talk-up works with bit-banged NSS.
Same Primary RF Slot also verified under ESP32-S3 RNode firmware.

**Critical SPI note:** hardware SPI0 CE0 does **not** chip-select the
SX126x on this board (GetStatus always `0x00`). NSS must be bit-banged on
physical pin 24 (BCM8). Use `dtoverlay=spi0-0cs` in
`/boot/firmware/config.txt` so GPIO8 is free for libgpiod (`spi0-1cs`
claims CE0 and blocks bit-bang).

Station G3's PA/LNA stage is gated by physical motherboard jumpers
(PA-PL1, PA-PL2, LNA-P) regardless of which MCU daughterboard is fitted —
these must be **OPEN/OPEN/OPEN** (PA Level 1) before power-on, matching
the verified ESP32-S3 path. On top of the jumper-selected level, this
daughterboard path additionally exposes software PA/LNA enable GPIOs
(`txen`/`rxen` below) that must also be driven for TX/RX to work.

```ini
[interfaces]

  [[Station G3 LoRa]]
    type = SX126xInterface
    interface_enabled = True
    frequency = 915000000
    bandwidth = 125000
    spreadingfactor = 7
    codingrate = 5
    # LOGICAL antenna dBm (post-PA). Profile maps via vendor L1@915 curve
    # to SX1262 chip dBm (e.g. 22 out → chip ~8). Default txpower_max=22.
    txpower = 22
    # Profile selection
    platform    = raspberry-pi
    radio_board = station-g3
    dio3_tcxo_voltage = 1.8
```

With `pa_curve` on this board, `txpower` / `txpower_max` are **logical
antenna dBm** (post external PA), not raw SX1262 chip dBm. The driver
picks the minimum chip setting whose vendor L1@915 PA out is ≥ the
target (e.g. 22→8, 27→14, 30→18, 32→21). Default `txpower_max = 22`
keeps PA drive modest (~chip 8). To test higher conducted power with
Level-1 jumpers OPEN, override both, e.g. `txpower = 32` and
`txpower_max = 32`. Chip drive is always clamped to −9…22 dBm.

#### Escape hatch — `radio_board = custom` (hand-wired boards)

When you don't have a named profile, set `radio_board = custom` and provide
`gpiochip` + `pin_*` directly. These values are **gpiochip line offsets**,
not physical pin numbers:

```ini
  [[My Hand-Wired Board]]
    type = SX126xInterface
    interface_enabled = True
    frequency = 915000000
    txpower = 17
    platform    = luckfox-pico
    radio_board = custom
    gpiochip    = gpiochip1
    pin_irq     = 23
    pin_busy    = 22
    pin_reset   = 25
    pin_txen    = -1
    pin_rxen    = 24
    spi_bus     = 0
    spi_cs      = 0
```

#### Per-key override (any profile mode)

If you need to override one pin without changing the whole profile, set
`pin_irq` etc. in the interface config block. In normal profile mode these
are interpreted as physical 40-pin-header pin numbers (re-resolved via the
platform's table); a WARNING is logged when an override differs from the
profile default. In `custom` mode, they are direct gpiochip line offsets.

#### Legacy mode (no `platform` / `radio_board`)

If neither key is set, the interface falls back to direct BCM gpiochip line
offsets for `pin_irq` etc. — the historical behaviour. A WARNING is logged
at startup recommending migration to the profile system.

#### With Transport enabled (router/repeater node)

Add this to the `[reticulum]` section of your config:

```ini
[reticulum]
  enable_transport = True
  share_instance = Yes
```

### 5. Start Reticulum

```bash
rnsd -v
```

You should see log output indicating the SX126x interface came online with the configured parameters. To run as a background service:

```bash
rnsd -s &
```

Or create a systemd service (see below).

## Pin Reference

### MeshAdv Pi HAT v1.1

| Function | BCM GPIO | Physical Pin |
|----------|----------|-------------|
| SPI MOSI | 10       | 19          |
| SPI MISO | 9        | 21          |
| SPI CLK  | 11       | 23          |
| CS (NSS) | 21       | 40          |
| RESET    | 18       | 12          |
| BUSY     | 20       | 38          |
| IRQ/DIO1 | 16       | 36          |
| TXEN     | 13       | 33          |
| RXEN     | 12       | 32          |
| GPS TX   | 14       | 8           |
| GPS RX   | 15       | 10          |
| GPS PPS  | 23       | 16          |

### BQ/Uniteng Station G3 (Pi Zero 2W path) — hardware-verified

Requires `dtoverlay=spi0-0cs` (not `spi0-1cs`) so NSS can be bit-banged.

| Function | BCM GPIO | Physical Pin | Notes |
|----------|----------|---------------|-------|
| SPI MOSI/MISO/CLK | native SPI0 | 19/21/23 | |
| CS (NSS) | 8        | 24            | **bit-banged** (`header_pin_cs = 24`); HW CE0 does not work |
| RESET    | 16       | 36            | |
| BUSY     | 24       | 18            | |
| IRQ/DIO1 | 22       | 15            | |
| TXEN (PA enable) | 17 | 11          | active-HIGH |
| RXEN (LNA enable)| 23 | 16          | active-LOW  |

### Waveshare SX1262 LoRa HAT (for reference)

The Waveshare HAT uses a UART-based E22 module, **not** direct SPI. It is **not compatible** with this interface. Use the SerialInterface instead, or get a direct-SPI board.

## Custom Platform / Board Profiles

If you have a board or SBC combination that isn't in the bundled list, you
can add your own profiles via two optional files in
`~/.reticulum/interfaces/`:

- `sx126x_platforms` — defines new platforms
- `sx126x_boards` — defines new boards

Both are ConfigObj/INI files. Each supports `based_on = <bundled-name>` to
inherit from a bundled profile and override individual fields.

Example `sx126x_boards`:

```ini
[boards]

  [[my-hand-wired-hat]]
  based_on = generic-sx1262-manual
  header_pin_irq = 11
  header_pin_reset = 7
  header_pin_txen = 13
  header_pin_rxen = 15
  profile_notes = My hand-wired SX1262 breakout
```

The resolver looks for these files in:
1. `~/.reticulum/interfaces/sx126x_platforms` (or `_boards`)
2. `<RETICULUM_HAT_MOD_DIR>/sx126x_platforms` (or `_boards`)
3. `<cwd>/sx126x_platforms` (or `_boards`)

Overlay load failures are logged as WARNINGs but do not prevent the
interface from starting with the bundled profiles.

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `frequency` | 915000000 | Frequency in Hz |
| `bandwidth` | 125000 | Bandwidth in Hz (7800-500000) |
| `spreadingfactor` | 8 | LoRa SF (5-12) |
| `codingrate` | 5 | LoRa CR denominator (5-8 = 4/5 to 4/8) |
| `txpower` | 22 | TX power in dBm (-9 to 22) |
| `platform` | _(none)_ | SBC profile name (e.g. `raspberry-pi`, `luckfox-pico`). See "Pin / GPIO Resolution" above. |
| `radio_board` | _(none)_ | HAT profile name (e.g. `meshadv-pi-hat-v1.1`, `femtofox-integrated-v1`, `custom`). |
| `gpiochip` | (from platform) | Override the gpiochip device name. |
| `spi_bus` | 0 | SPI bus number |
| `spi_cs` | 0 | SPI chip-select index (spidev CE0=0, CE1=1) |
| `pin_irq` | (from board) | IRQ pin. Physical pin in profile mode; gpiochip line offset in legacy/`custom` mode. |
| `pin_busy` | (from board) | BUSY pin. Physical pin in profile mode; gpiochip line offset in legacy/`custom` mode. |
| `pin_reset` | (from board) | RESET pin. Physical pin in profile mode; gpiochip line offset in legacy/`custom` mode. |
| `pin_txen` | (from board) | TXEN pin. Physical pin in profile mode; gpiochip line offset in legacy/`custom` mode. `-1` if unwired. |
| `pin_rxen` | (from board) | RXEN pin. Physical pin in profile mode; gpiochip line offset in legacy/`custom` mode. |
| `dio2_rf_switch` | (from board) | If true, route DIO2 as RF switch control (for E22-style modules). |
| `dio3_tcxo_voltage` | (from board) | TCXO voltage via DIO3 (0 or false to disable). |
| `tcxo_delay_ms` | (from board) | TCXO warm-up delay in ms. |
| `rx_boosted_gain` | (from board) | If true, use boosted RX gain register value. |
| `sync_word` | 0x12 | LoRa sync word (0x12=private, 0x34=public) |
| `csma_p` | 0.1 | CSMA transmit probability (0.0-1.0) |
| `csma_slot_ms` | 50 | CSMA slot time in milliseconds |
| `csma_max_backoff` | 5 | Max CSMA backoff exponent |
| `airtime_limit_short` | (none) | Short-term airtime limit (%) |
| `airtime_limit_long` | (none) | Long-term airtime limit (%) |

## Interoperability with RNodes

To communicate with standard RNodes, configure **matching** radio parameters on both sides:
- Same frequency
- Same bandwidth
- Same spreading factor
- Same coding rate
- Same sync word (0x12 is the RNode/Reticulum default)

The split-packet framing is RNode-compatible, so this interface can talk directly to any RNode on the same LoRa parameters.

## Running as a systemd Service

```bash
sudo tee /etc/systemd/system/rnsd.service << 'EOF'
[Unit]
Description=Reticulum Network Stack Daemon
After=network.target

[Service]
Type=simple
User=pi
ExecStart=/usr/local/bin/rnsd -s
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable rnsd
sudo systemctl start rnsd
```

Check status with:

```bash
sudo systemctl status rnsd
rnstatus
```

## Troubleshooting

**"SX126x radio not detected"**
- Check that SPI is enabled: `ls /dev/spidev*` should show at least `spidev0.0`
- Check wiring — make sure the HAT is fully seated
- Ensure the antenna is connected (the E22 module can fail to init without a load)
- Verify `python3-libgpiod` and `python3-spidev` are installed
- Confirm `vendored_sx126x.py` is in `~/.reticulum/interfaces/` alongside `SX126xInterface.py`

**No packets received**
- Verify frequency, bandwidth, SF, and CR match the other node exactly
- Check sync word matches (default 0x12 for RNode)
- Try increasing TX power on the remote side
- Check antenna connection on both ends

**Intermittent packet loss**
- Normal for LoRa at range. The CSMA parameters can be tuned:
  - Lower `csma_p` for busier networks (more polite, but slower)
  - Increase `csma_slot_ms` for longer-range links (accounts for propagation)

**GPIO permission errors**
- Run as root, or add your user to the `gpio` group: `sudo usermod -aG gpio $USER`
- On newer Pi OS, you may also need: `sudo usermod -aG spi $USER`

## How It Works

### Architecture

```
┌──────────────────────────────────────────────────┐
│  Raspberry Pi                                     │
│                                                   │
│  ┌───────────────┐     ┌──────────────────────┐  │
│  │  Sideband /   │     │   SX126xInterface    │  │
│  │  NomadNet /   │◄───►│   (this module)      │  │
│  │  MeshChat     │     │                      │  │
│  │               │     │   TX queue (Queue)   │  │
│  └───────────────┘     │         ▲            │  │
│         ▲              │         │ non-block  │  │
│         │              │         │ enqueue    │  │
│  ┌──────┴──────┐       │  ┌──────┴─────────┐  │  │
│  │ Reticulum   │       │  │  radio thread  │  │  │
│  │ (rnsd)      │◄─────►│  │  (SINGLE       │  │  │
│  │  Transport  │       │  │   radio owner) │  │  │
│  │  thread     │       │  │   CSMA+CAD      │  │  │
│  └─────────────┘       │  │   split framing │  │  │
│                        │  │   airtime acct. │  │  │
│                        │  └────────┬─────────┘  │  │
│                        └───────────┼────────────┘  │
│                                    │ SPI+GPIO    │
├────────────────────────────────────┼─────────────┤
│  MeshAdv Pi HAT                    ▼             │
│  ┌──────────────────────────────────────────┐    │
│  │  Ebyte E22-900M30S (SX1262, 1W)         │    │
│  │  + antenna                               │    │
│  └──────────────────────────────────────────┘    │
└──────────────────────────────────────────────────┘
```

Key concurrency invariant: **only one thread ever touches the radio**.
The `process_outgoing` method (called by Reticulum's Transport thread)
merely enqueues the frame onto a thread-safe `queue.Queue` and returns
immediately — no blocking, no CSMA wait, no TX. A dedicated **radio-owner
thread** drains the queue and is the sole caller of every SX126x command.
This eliminates the TX/RX race that plagued the older implementation.
The thread blocks on the SX126x IRQ line via libgpiod edge events
(vendored driver; no busy-spin), so idle RX costs effectively zero CPU.

### Split-Packet Framing

The SX1262 can only transmit 255 bytes per LoRa frame, but Reticulum's MTU is 500 bytes. This interface uses the same split-packet protocol as RNode firmware:

- Each frame gets a 1-byte header prepended
- Upper nibble: random 4-bit sequence ID
- Bit 0: FLAG_SPLIT (1 if this packet spans 2 frames)
- Packets ≤ 254 bytes → single frame
- Packets > 254 bytes → 2 frames with matching sequence IDs

This is wire-compatible with RNode firmware and the micropython-reticulum SX1262 interface.

## Contributing

This is a community project filling a gap in the Reticulum ecosystem. If you test it on hardware, please report your results. PRs welcome for:
- Additional board pin presets
- Improved CSMA tuning
- CAD (Channel Activity Detection) based carrier sense
- Support for SX1276/SX1278 boards
- Airtime accounting improvements

## Acknowledgements

- [Mark Qvist](https://github.com/markqvist) for Reticulum and the RNode platform
- [Chris Myers](https://github.com/chrismyers2000) for the MeshAdv Pi HAT
- [chandrawi](https://github.com/chandrawi/LoRaRF-Python) for the LoRaRF-Python library (whose command opcodes the vendored driver mirrors byte-for-byte)
- [varna9000](https://github.com/varna9000/micropython-reticulum) for the micropython-reticulum SX1262 interface which validated the split-packet framing approach
- The Reticulum Discussion #652 community for pushing for SPI HAT support

## License

MIT License
