# SX126xInterface for Reticulum

A custom Reticulum interface module that drives SX1262/SX1268 LoRa chips **directly over SPI** on a Raspberry Pi (or other Linux SBC). No separate RNode microcontroller needed — the Pi *is* the RNode.

Designed for the **MeshAdv Pi HAT v1.1** but works with any SPI-connected SX126x board with configurable pin mapping.

## What This Does

Normally, Reticulum requires a separate RNode device (an ESP32 or nRF52 running RNode firmware) connected via USB serial. This interface eliminates that second device by talking directly to the SX1262 LoRa chip over the Pi's SPI bus.

It implements:
- **Direct SPI control** via the [LoRaRF-Python](https://github.com/chandrawi/LoRaRF-Python) library
- **CSMA/CA** collision avoidance (p-persistent, like RNode firmware)
- **RNode-compatible split-packet framing** so the full 500-byte Reticulum MTU works over the 255-byte LoRa frame limit
- **Airtime tracking and limiting**
- **Interoperability** with standard RNodes on the same frequency/parameters

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
pip install rns LoRaRF
```

Or if your OS restricts pip:

```bash
pip install rns LoRaRF --break-system-packages
```

### 3. Install the interface module

Copy `SX126xInterface.py` to the Reticulum custom interfaces directory:

```bash
mkdir -p ~/.reticulum/interfaces
cp SX126xInterface.py ~/.reticulum/interfaces/
```

### 4. Configure Reticulum

Edit `~/.reticulum/config` and add an interface block. If the file doesn't exist yet, run `rnsd` once to generate it, then edit.

#### MeshAdv Pi HAT v1.1 (915 MHz, default config)

```ini
[[MeshAdv LoRa]]
  type = SX126xInterface
  interface_enabled = True
  frequency = 915000000
  bandwidth = 125000
  spreadingfactor = 8
  codingrate = 5
  txpower = 22
  # SPI
  spi_bus = 0
  spi_cs = 0
  # GPIO pins (BCM numbering) — MeshAdv Pi HAT defaults
  pin_cs = 21
  pin_irq = 16
  pin_busy = 20
  pin_reset = 18
  pin_txen = 13
  pin_rxen = 12
  # TCXO voltage for E22 module
  dio3_tcxo_voltage = 1.8
```

#### MeshAdv Pi HAT v1.1 (868 MHz variant)

```ini
[[MeshAdv LoRa 868]]
  type = SX126xInterface
  interface_enabled = True
  frequency = 868000000
  bandwidth = 125000
  spreadingfactor = 8
  codingrate = 5
  txpower = 22
  spi_bus = 0
  spi_cs = 0
  pin_cs = 21
  pin_irq = 16
  pin_busy = 20
  pin_reset = 18
  pin_txen = 13
  pin_rxen = 12
  dio3_tcxo_voltage = 1.8
```

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

### Waveshare SX1262 LoRa HAT (for reference)

The Waveshare HAT uses a UART-based E22 module, **not** direct SPI. It is **not compatible** with this interface. Use the SerialInterface instead, or get a direct-SPI board.

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `frequency` | 915000000 | Frequency in Hz |
| `bandwidth` | 125000 | Bandwidth in Hz (7800-500000) |
| `spreadingfactor` | 8 | LoRa SF (5-12) |
| `codingrate` | 5 | LoRa CR denominator (5-8 = 4/5 to 4/8) |
| `txpower` | 22 | TX power in dBm (-9 to 22) |
| `spi_bus` | 0 | SPI bus number |
| `spi_cs` | 0 | SPI chip select |
| `pin_cs` | 21 | BCM GPIO for chip select |
| `pin_irq` | 16 | BCM GPIO for IRQ (DIO1) |
| `pin_busy` | 20 | BCM GPIO for BUSY |
| `pin_reset` | 18 | BCM GPIO for RESET |
| `pin_txen` | 13 | BCM GPIO for TX enable |
| `pin_rxen` | 12 | BCM GPIO for RX enable |
| `dio3_tcxo_voltage` | 1.8 | TCXO voltage via DIO3 (0 to disable) |
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

**"LoRaRF library not found"**
- Run `pip install LoRaRF`

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
│  │               │     │  ┌─────────────────┐ │  │
│  └───────────────┘     │  │ CSMA/CA engine  │ │  │
│         ▲              │  │ Split-pkt frame │ │  │
│         │              │  │ Airtime tracker │ │  │
│  ┌──────┴──────┐       │  └────────┬────────┘ │  │
│  │ Reticulum   │       │           │ SPI      │  │
│  │ (rnsd)      │◄─────►│           ▼          │  │
│  └─────────────┘       │  ┌─────────────────┐ │  │
│                        │  │  LoRaRF-Python  │ │  │
│                        │  │  (SX126x drv)   │ │  │
│                        │  └────────┬────────┘ │  │
│                        └───────────┼──────────┘  │
│                                    │ SPI+GPIO    │
├────────────────────────────────────┼─────────────┤
│  MeshAdv Pi HAT                    ▼             │
│  ┌──────────────────────────────────────────┐    │
│  │  Ebyte E22-900M30S (SX1262, 1W)         │    │
│  │  + antenna                               │    │
│  └──────────────────────────────────────────┘    │
└──────────────────────────────────────────────────┘
```

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
- [chandrawi](https://github.com/chandrawi/LoRaRF-Python) for the LoRaRF-Python library
- [varna9000](https://github.com/varna9000/micropython-reticulum) for the micropython-reticulum SX1262 interface which validated the split-packet framing approach
- The Reticulum Discussion #652 community for pushing for SPI HAT support

## License

MIT License
