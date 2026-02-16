# TAC1100

**ModBus RTU client for TAIYE TAC1100 series smart mini power meter**

![License](https://img.shields.io/badge/license-GPL--2.0-blue.svg)
![Version](https://img.shields.io/badge/version-1.0-green.svg)

## Overview

`tac1100` is a command-line utility to read and configure TAIYE TAC1100 series smart mini power meters via ModBus RTU protocol. It allows you to:

- **Read** electrical parameters (voltage, current, power, energy, etc.)
- **Configure** meter settings (address, baud rate, password, etc.)
- **Manage** historical data and display settings
- **Control** KPPA (Key Parameter Programming Authorization) for secure operations

### Architecture

The program implements a **two-level cooperative locking system** designed for RS485 bus sharing:

1. **Shared Lock (LOCK_SH)** - Allows multiple ModBus clients to coexist
2. **Exclusive Lock (LOCK_EX)** - Ensures only one client communicates at a time

This architecture enables collision-free operation when running multiple instances of `tac1100`, `sdm120c`, `aurora`, or other compatible ModBus clients on the same serial port.

## Features

- Read single or multiple electrical parameters
- Support for all TAC1100 measurement registers
- Write operations with KPPA authorization
- Automatic retry and error handling
- **Cooperative bus sharing** with other ModBus clients (sdm120c, aurora)
- **Two-level locking system**: shared lock for coexistence, exclusive lock for communication
- Serial port locking mechanism preventing bus collisions
- Multiple output formats (normal, compact, IEC 62056)
- Debug and trace modes

## Requirements

### Dependencies

- **libmodbus** (>= 3.0.0) - [http://libmodbus.org](http://libmodbus.org)
- GCC compiler
- Linux operating system

### Installing libmodbus

**Debian/Ubuntu:**
```bash
sudo apt-get install libmodbus-dev
```

**From source:**
```bash
git clone https://github.com/stephane/libmodbus.git
cd libmodbus
./autogen.sh
./configure
make
sudo make install
```

## Installation

### Compile from source

```bash
# Clone the repository
git clone https://github.com/flanesi/TAC1100.git
cd TAC1100

# Clean and compile
make clean && make

# Install (requires sudo)
sudo make install
```

### Uninstall

```bash
sudo make uninstall
```

## Usage
<PRE>
TAC1100c 1.0: ModBus RTU client to read TAC1100 series smart mini power meter registers
Copyright (C) 2026 Flavio Anesi
Complied with libmodbus 3.1.6

Usage: tac1100 [-a address] [-d n] [-x] [-p] [-v] [-c] [-e] [-i] [-t] [-f] [-g] [-T] [[-m]|[-q]] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -s new_address device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -r baud_rate device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -N parity device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -Q current_password -K new_password device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -L demand_period device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -U slide_time device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -R scroll_time device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -G backlit_time device
       tac1100 [-a address] [-d n] [-x] [-b baud_rate] [-P parity] [-S bit] [-z num_retries] [-j seconds] [-w seconds] -Q current_password -H reset_type device

Required:
        device          Serial device (i.e. /dev/ttyUSB0)
Connection parameters:
        -a address      Meter number (1-247). Default: 1
        -b baud_rate    Use baud_rate serial port speed (1200, 2400, 4800, 9600, 19200)
                        Default: 9600
        -P parity       Use parity (E, N, O). Default: N
        -S bit          Use stop bits (1, 2). Default: 1
Reading parameters (no parameter = retrieves all values):
        -p              Get power (W)
        -v              Get voltage (V)
        -c              Get current (A)
        -l              Get apparent power (VA)
        -n              Get reactive power (VAR)
        -f              Get frequency (Hz)
        -o              Get phase angle (Degree)
        -g              Get power factor
        -i              Get imported energy (Wh)
        -e              Get exported energy (Wh)
        -t              Get total energy (Wh)
        -A              Get imported reactive energy (VARh)
        -B              Get exported reactive energy (VARh)
        -C              Get total reactive energy (VARh)
        -T              Get Time for automatic scroll display (0=no rotation)
        -m              Output values in IEC 62056 format ID(VALUE*UNIT)
        -q              Output values in compact mode
Writing new settings parameters:
        -s new_address  Set new meter number (1-247)
        -r baud_rate    Set baud_rate meter speed (1200, 2400, 4800, 9600, 19200)
        -N parity       Set parity and stop bits (0-3) [REQUIRES RESTART]
                        0: N1, 1: E1, 2: O1, 3: N2
        -Q password     Current password for KPPA authorization (default: 0000)
        -K password     Set new password (0-9999) [REQUIRES KPPA: use -Q]
        -L demand_period Set demand period (0-60 minutes, 0=update every second, default 60)
        -U slide_time   Set slide time (1 to Demand_Period-1, default 1)
        -R scroll_time  Set automatic scroll display time (0-60 seconds, 0=no rotation, default 0)
        -G backlit_time Set backlit time (0-120 or 255 minutes, 0=always on, 255=off, default 60)
        -H reset_type   Reset historical data [REQUIRES KPPA: use -Q]
                        0=max demand, 8=monthly energy, 9=daily energy

KPPA = Key Parameter Programming Authorization
Operations marked [REQUIRES KPPA] need -Q option with current password.
Operations marked [REQUIRES RESTART] need meter restart after modification.

Examples:
  Read all values:        tac1100 /dev/ttyUSB0
  Change password:        tac1100 -Q 0000 -K 1234 /dev/ttyUSB0
  Reset max demand:       tac1100 -Q 1234 -H 0 /dev/ttyUSB0
  Change meter address:   tac1100 -s 5 /dev/ttyUSB0

Fine tuning & debug parameters:
        -z num_retries  Try to read max num_retries times on bus before exiting
                        with error. Default: 1 (no retry)
        -j 1/10 secs    Response timeout. Default: 2=0.2s
        -D 1/1000 secs  Delay before sending commands. Default: 0ms
        -w seconds      Time to wait to lock serial port (1-30s). Default: 0s
        -W 1/1000 secs  Time to wait for 485 line to settle. Default: 0ms
        -y 1/1000 secs  Set timeout between every bytes (1-500). Default: disabled
        -d debug_level  Debug (0=disable, 1=debug, 2=errors to syslog, 3=both)
                        Default: 0
        -x              Trace (libmodbus debug on)</PRE>

### Basic Syntax

```bash
tac1100 [OPTIONS] <device>
```

Where `<device>` is the serial port (e.g., `/dev/ttyUSB0`)

### Connection Parameters

| Option | Description | Default |
|--------|-------------|---------|
| `-a` | Meter address (1-247) | 1 |
| `-b` | Baud rate (1200, 2400, 4800, 9600, 19200) | 9600 |
| `-P` | Parity (E, N, O) | N |
| `-S` | Stop bits (1, 2) | 1 |

### Reading Parameters

Without any read option, the program retrieves all values:

```bash
tac1100 /dev/ttyUSB0
```

**Read specific parameters:**

| Option | Parameter |
|--------|-----------|
| `-v` | Voltage (V) |
| `-c` | Current (A) |
| `-p` | Active power (W) |
| `-l` | Apparent power (VA) |
| `-n` | Reactive power (VAR) |
| `-f` | Frequency (Hz) |
| `-o` | Phase angle (Degrees) |
| `-g` | Power factor |
| `-i` | Import active energy (Wh) |
| `-e` | Export active energy (Wh) |
| `-t` | Total active energy (Wh) |
| `-A` | Import reactive energy (VARh) |
| `-B` | Export reactive energy (VARh) |
| `-C` | Total reactive energy (VARh) |
| `-T` | Auto scroll display time |

**Examples:**

```bash
# Read voltage and current
tac1100 -v -c /dev/ttyUSB0

# Read all energy counters
tac1100 -i -e -t /dev/ttyUSB0

# Read power parameters
tac1100 -p -l -n -g /dev/ttyUSB0
```

### Writing Parameters

**⚠️ Important Notes:**
- Operations marked `[REQUIRES KPPA]` need the `-Q` option with current password
- Operations marked `[REQUIRES RESTART]` need meter restart after modification
- Default password is `0000`

#### Change Meter Address

```bash
tac1100 -s 5 /dev/ttyUSB0
```

#### Change Baud Rate

```bash
tac1100 -r 19200 /dev/ttyUSB0
```

#### Change Parity/Stop Bits `[REQUIRES RESTART]`

```bash
# Set to Even parity, 1 stop bit (E1)
tac1100 -N 1 /dev/ttyUSB0
```

Parity options:
- `0` = N1 (None, 1 stop bit)
- `1` = E1 (Even, 1 stop bit)
- `2` = O1 (Odd, 1 stop bit)
- `3` = N2 (None, 2 stop bits)

#### Change Password `[REQUIRES KPPA]`

```bash
# Change password from 0000 (default) to 1234
tac1100 -Q 0000 -K 1234 /dev/ttyUSB0

# Change password from 1234 to 5678
tac1100 -Q 1234 -K 5678 /dev/ttyUSB0
```

#### Reset Historical Data `[REQUIRES KPPA]`

```bash
# Reset maximum demand (password: 0000)
tac1100 -Q 0000 -H 0 /dev/ttyUSB0

# Reset monthly energy consumption
tac1100 -Q 0000 -H 8 /dev/ttyUSB0

# Reset daily energy consumption
tac1100 -Q 0000 -H 9 /dev/ttyUSB0
```

#### Configure Display Settings

```bash
# Set demand period to 15 minutes
tac1100 -L 15 /dev/ttyUSB0

# Set slide time to 5
tac1100 -U 5 /dev/ttyUSB0

# Set auto scroll time to 10 seconds
tac1100 -R 10 /dev/ttyUSB0

# Set backlight always on
tac1100 -G 0 /dev/ttyUSB0

# Set backlight timeout to 30 minutes
tac1100 -G 30 /dev/ttyUSB0
```

### Output Formats

```bash
# Normal output (default)
tac1100 /dev/ttyUSB0

# Compact output
tac1100 -q /dev/ttyUSB0

# IEC 62056 format
tac1100 -m /dev/ttyUSB0
```

### Debug and Advanced Options

| Option | Description |
|--------|-------------|
| `-d` | Debug level (0=disable, 1=debug, 2=syslog, 3=both) |
| `-x` | Enable libmodbus trace |
| `-z` | Number of retries before error (default: 1) |
| `-j` | Response timeout in 1/10 seconds (default: 2 = 0.2s) |
| `-w` | Wait time to lock serial port (1-30s, default: 0) |
| `-D` | Delay before sending commands in ms |
| `-W` | Wait time for RS485 line to settle in ms |
| `-y` | Byte timeout in ms (1-500) |

**Example with debug:**

```bash
tac1100 -d 1 -x -v /dev/ttyUSB0
```

## KPPA (Key Parameter Programming Authorization)

Some write operations require KPPA authorization for security. To perform these operations:

1. Know your current meter password (default: `0000`)
2. Use the `-Q` option to provide the current password
3. Execute the desired operation

**Operations requiring KPPA:**
- Change password (`-K`)
- Reset historical data (`-H`)

**Example workflow:**

```bash
# First time setup (password is default 0000)
tac1100 -Q 0000 -K 1234 /dev/ttyUSB0

# After password change, use new password
tac1100 -Q 1234 -H 0 /dev/ttyUSB0
```

## Cooperative Bus Sharing

`tac1100` implements a **two-level locking system** that allows multiple ModBus clients to share the same RS485 bus without collisions:

### Supported Compatible Clients

- **sdm120c** - SDM120 power meter client by gianfrdp
- **tac1100** - TAC1100 power meter client (multiple instances)
- **aurora** - Aurora inverter client by Curtronis
- Any program with "modbus" in its name

### How It Works

**Level 1 - Shared Lock (LOCK_SH):**
- Multiple processes can coexist and wait for bus access
- Clients recognize each other as compatible
- Low overhead, allows parallel initialization

**Level 2 - Exclusive Lock (LOCK_EX):**
- Only one process communicates on the bus at a time
- Automatically acquired before ModBus communication
- Prevents bus collisions and CRC errors
- Released immediately after communication

### Example Usage

**Multiple meters on same bus:**
```bash
# Terminal 1 - Read TAC1100 at address 1
tac1100 -a 1 /dev/ttyUSB0

# Terminal 2 - Read TAC1100 at address 2 (simultaneously)
tac1100 -a 2 /dev/ttyUSB0

# Terminal 3 - Read SDM120 at address 3
sdm120c -a 3 /dev/ttyUSB0
```

**Continuous monitoring with multiple clients:**
```bash
# Monitor TAC1100 meters
while true; do tac1100 -a 1 /dev/ttyUSB0; sleep 5; done &
while true; do tac1100 -a 2 /dev/ttyUSB0; sleep 5; done &

# Monitor SDM120 meter simultaneously
while true; do sdm120c -a 3 /dev/ttyUSB0; sleep 5; done &
```

**Expected behavior:**
- ✅ All processes run without errors
- ✅ No bus collisions or timeouts
- ✅ Clients wait their turn automatically
- ✅ Order of execution doesn't matter

### Important Notes

**For sdm120c users:**
To achieve full collision-free operation when reading the **same meter address** simultaneously, `sdm120c` must also implement the exclusive lock mechanism. Without this modification:
- ✅ Different addresses work fine
- ✅ Infrequent polling (10+ seconds) usually works
- ⚠️ Same address + frequent polling may cause occasional timeouts

**Workarounds without modifying sdm120c:**
1. Use different meter addresses
2. Stagger polling times (offset by 5+ seconds)
3. Increase retry count: `-z 5`

### Debug Bus Sharing

Enable debug mode to see lock operations:
```bash
tac1100 -a 2 -d 1 /dev/ttyUSB0
```

Output shows:
```
Compatible ModBus client detected: sdm120c (PID 12345). Sharing bus.
Upgrading to exclusive lock for ModBus communication...
Exclusive lock acquired. Ready for ModBus communication.
[readings...]
Releasing exclusive ModBus lock...
```

## TAC1100 Register Map

### Read Registers (Input Registers, Function 04H)

| Address | Parameter | Unit |
|---------|-----------|------|
| 0x0000 | Voltage | V |
| 0x0006 | Current | A |
| 0x000C | Active Power | W |
| 0x0012 | Reactive Power | VAR |
| 0x0018 | Apparent Power | VA |
| 0x001E | Power Factor | - |
| 0x0024 | Phase Angle | Degrees |
| 0x0030 | Frequency | Hz |
| 0x0500 | Total Import Active Energy | kWh |
| 0x0502 | Total Export Active Energy | kWh |
| 0x0504 | Total Active Energy | kWh |
| 0x0508 | Total Import Reactive Energy | kVARh |
| 0x050A | Total Export Reactive Energy | kVARh |
| 0x050C | Total Reactive Energy | kVARh |

### Write Registers (Holding Registers, Function 03H/10H)

| Address | Parameter | Range | KPPA Required | Restart Required |
|---------|-----------|-------|---------------|------------------|
| 0x5000 | KPPA | - | No | No |
| 0x5002 | Demand Period | 0-60 min | No | No |
| 0x5003 | Slide Time | 1 to DP-1 | No | No |
| 0x5005 | Device Address | 1-247 | No | No |
| 0x5006 | Baud Rate | 0-4 | No | No |
| 0x5007 | Parity/Stop | 0-3 | No | Yes |
| 0x5008 | Password | 0-9999 | Yes | No |
| 0x5018 | Auto Scroll Time | 0-60 sec | No | No |
| 0x5019 | Backlit Time | 0-120, 255 | No | No |
| 0x5600 | Reset Historical | 0, 8, 9 | Yes | No |

## Troubleshooting

### Communication Errors

**Problem:** `Connection failed` or `Read error`

**Solutions:**
1. Check serial port permissions: `sudo chmod 666 /dev/ttyUSB0`
2. Verify correct baud rate and parity settings
3. Check physical RS485 connections
4. Increase timeout: `-j 5` (0.5 seconds)
5. Add retries: `-z 3`

### Bus Collision Issues

**Problem:** `ERROR (11) Resource temporarily unavailable` or frequent timeouts when multiple clients are active

**Cause:** Multiple processes trying to communicate simultaneously on the RS485 bus

**Solutions:**

1. **Update tac1100** to latest version with exclusive lock support (already included)

2. **If using sdm120c simultaneously:**
   - Update sdm120c with exclusive lock support (recommended)
   - OR use different meter addresses
   - OR stagger polling times:
     ```bash
     # tac1100 reads immediately
     while true; do tac1100 -a 1 /dev/ttyUSB0; sleep 10; done &
     
     # sdm120c reads 5 seconds later
     sleep 5; while true; do sdm120c -a 2 /dev/ttyUSB0; sleep 10; done
     ```
   - OR increase retries: `tac1100 -a 1 -z 5 /dev/ttyUSB0`

3. **Enable debug to diagnose:**
   ```bash
   tac1100 -a 2 -d 1 /dev/ttyUSB0
   ```
   Look for:
   - `Compatible ModBus client detected` - Good, clients recognize each other
   - `Exclusive lock acquired` - Good, communication is protected
   - `Resource temporarily unavailable` - Bus collision detected

### Lock File Issues

**Problem:** `Exceeded maximum lock attempts` or `Lock file may be corrupted`

**Solutions:**
1. Remove stale lock file:
   ```bash
   sudo rm -f /var/lock/LCK..ttyUSB0*
   ```
2. Check no ghost processes:
   ```bash
   ps aux | grep -E "tac1100|sdm120c|aurora"
   ```
3. Increase lock wait time: `-w 15` (15 seconds)

### KPPA Authorization Errors

**Problem:** `Failed to enable KPPA` when using `-K` or `-H`

**Solutions:**
1. Verify you're using the correct current password with `-Q`
2. Default password is `0000`
3. Ensure meter is powered and responding
4. Check ModBus communication is working (try reading values first)

### Write Operation Failures

**Problem:** `Write operation failed`

**Solutions:**
1. For operations requiring KPPA, ensure you use `-Q` with correct password
2. For parity changes (`-N`), remember to restart the meter afterward
3. Add command delay: `-D 100` (100ms delay)
4. Check meter is not in read-only mode

## Technical Specifications

- **Protocol:** ModBus RTU
- **Supported Baud Rates:** 1200, 2400, 4800, 9600, 19200
- **Supported Parity:** None, Even, Odd
- **Default Settings:** 9600 baud, No parity, 1 stop bit
- **Register Format:** Float (IEEE 754) for measurements, UINT16 for configuration
- **Byte Order:** Big-endian (Modbus standard)

## License

This program is free software; you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation; either version 2 of the License, or (at your option) any later version.

See the [LICENSE](LICENSE) file for details.

## Credits

- **Author:** Flavio Anesi - [www.flanesi.it](http://www.flanesi.it)
- **Based on:** sdm120c by Gianfranco Di Prinzio
- **Locking code:** Partially from aurora by Curtronis

## Support

For bugs, feature requests, or questions:
- Open an issue on [GitHub](https://github.com/flanesi/TAC1100/issues)
- Visit [www.flanesi.it](http://www.flanesi.it)

## FAQ

### Can I run multiple instances of tac1100 simultaneously?

Yes! The program uses a two-level locking system that allows multiple instances to share the same RS485 bus. Each instance will automatically wait for exclusive access before communicating.

### Can tac1100 work together with sdm120c or aurora?

Yes! `tac1100` automatically recognizes compatible ModBus clients (sdm120c, aurora, and others) and shares the bus cooperatively. For best results with the same meter address, ensure all clients implement the exclusive lock mechanism.

### What happens if two programs try to read at the exact same time?

The exclusive lock system ensures only one program communicates on the bus at any given moment. Other programs automatically wait their turn. There will be no bus collisions or communication errors.

### How do I know if bus sharing is working correctly?

Enable debug mode: `tac1100 -a 2 -d 1 /dev/ttyUSB0`

You should see:
```
Compatible ModBus client detected: sdm120c (PID xxxxx). Sharing bus.
Upgrading to exclusive lock for ModBus communication...
Exclusive lock acquired. Ready for ModBus communication.
```

If you see timeouts or "Resource temporarily unavailable" errors, increase retries (`-z 5`) or check your setup.

### What's the difference between LOCK_SH and LOCK_EX?

- **LOCK_SH (Shared Lock)**: Multiple processes can hold this simultaneously. Used during initialization and waiting.
- **LOCK_EX (Exclusive Lock)**: Only one process can hold this at a time. Used during actual ModBus communication to prevent bus collisions.

### Can I disable the exclusive lock if I only use tac1100?

The exclusive lock is always enabled and adds negligible overhead (< 1ms). It's designed to be transparent and safe even with a single instance.

## Changelog

### Version 1.0 (2026)
- Initial release
- Full support for TAC1100 series meters
- KPPA authorization implementation
- Read all measurement parameters
- Write configuration with password protection
- Historical data reset functionality
- **Cooperative bus sharing with two-level locking system**
- **Compatible with sdm120c, aurora, and other ModBus clients**
- **Exclusive lock during ModBus communication prevents bus collisions**
- Automatic recognition of compatible ModBus clients
- Support for multiple simultaneous instances
- Enhanced error handling and retry mechanisms
