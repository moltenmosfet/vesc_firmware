# VESC firmware — Molten MOSFET dyno fork

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

This is the **Molten MOSFET** fork of [vedderb/bldc](https://github.com/vedderb/bldc),
the VESC motor-controller firmware. It runs the absorber drive of a custom EUC
dynamometer: a salvaged Nissan Leaf EM57 traction motor, belt-coupled to the roller
and driven as a four-quadrant load by a VESC-derivative SiC inverter.

## Why a fork

The dyno needs capabilities stock VESC firmware doesn't have:

**d-axis dissipation braking.** During absorption tests the rig recovers up to tens of
kW from the device under test. Instead of dumping that energy into a resistor bank
through a brake chopper, it is burned inside the absorber motor's own windings:
controlled d-axis current injection produces near-zero torque but real I²R heat,
which the motor's water jacket carries away. The firmware side of this is a
dissipation mode — a commanded d-axis current wrapped in ramping, a refresh-or-decay
watchdog, and current/thermal clamps. Torque (q-axis) control keeps priority at all
times; dissipation only ever takes the leftover current budget.

**The d-axis bus clamp — braking without a battery.** When the DC bus has no sink
(a bench PSU that can't sink current; a full pack that must not be overcharged),
regen would run the bus into the overvoltage fault. A firmware-local fast-loop
controller burns regen in the windings as it is produced: a bus-current floor
(input current never goes below a setpoint — zero means never backfeed) plus a
voltage clamp as reactive backstop. Armed-only, RAM-only, deliberately not
watchdogged. Design, wire protocol, margins and the bench ladder:
[documentation/mm_d_axis_dissipation.md](documentation/mm_d_axis_dissipation.md).

Dyno build (drops unused LispBM for flash headroom): `make 60 USE_LISPBM=0`.

## Fork design rules

- **Runtime control is native VESC CAN.** New features get a CAN frame first; COMM
  packets are added only for bench/serial use.
- **No `mcconf`/`appconf` layout changes.** The fork stays connectable and
  configurable with stock VESC Tool; feature limits travel in the commands themselves.
- **Every new actuator command auto-expires.** Refresh-or-decay semantics, cleared on
  fault, motor release, and comms timeout — same philosophy as the stock off-delay
  current commands.

## Branches & status

| Branch | Purpose | Status |
|---|---|---|
| `master` | Tracks upstream `vedderb/bldc` master | clean mirror |
| `feature/d-axis-dissipation` | d-axis dissipation braking mode | in development |

The host-side driver lives in the companion fork
[moltenmosfet/PyVESC](https://github.com/moltenmosfet/PyVESC), which speaks both the
native CAN protocol and COMM, including the commands added here.

---

*Everything below is the upstream VESC firmware README (build instructions apply
unchanged to this fork).*

# VESC firmware

An open source motor controller firmware.

This is the source code for the VESC DC/BLDC/FOC controller. Read more at
[https://vesc-project.com/](https://vesc-project.com/)

## Supported boards

All of them!

Check the supported boards by typing `make`

```
[Firmware]
     fw   - Build firmware for default target
                            supported boards are: 100_250 100_250_no_limits 100_500...
```

There are also many other options that can be changed in [conf_general.h](conf_general.h).

## Prerequisites

### On Ubuntu (Linux)/macOS
- Tools: `git`, `wget`, and `make`
- Additional Linux requirements: `libgl-dev` and `libxcb-xinerama0`
- Helpful Ubuntu commands:
```bash
sudo apt install git build-essential libgl-dev libxcb-xinerama0 wget git-gui
```
- Helpful macOS tools: 

```bash
brew install stlink
brew install openocd
```

### On Windows
- Chocolately: https://chocolatey.org/install
- Git: https://git-scm.com/download/win. Make sure to click any boxes to add Git to your Environment (aka PATH)

## Install Dev environment and build

### On Ubuntu (Linux)/MacOS
Open up a terminal
1.  `git clone http://github.com/vedderb/bldc.git`
2.  `cd bldc`
3.  Continue with [On all platforms](#on-all-platforms)

### On Windows

1.  Open up a Windows Powershell terminal (Resist the urge to run Powershell as administrator, that will break things)
2.  Type `choco install make`
3.  `git clone http://github.com/vedderb/bldc`
4.  `cd bldc`
5.  Continue with [On all platforms](#on-all-platforms)

### On all platforms

1.  `git checkout origin/master`
2.  `make arm_sdk_install`
3.  `make` <-- Pick out the name of your target device from the supported boards list. For instance, I have a Trampa **VESC 100/250**, so my target is `100_250`
4.   `make 100_250` <-- This will build the **VESC 100/250** firmware and place it into the `bldc/builds/100_250/` directory

## Other tools

**Linux Optional - Add udev rules to use the stlink v2 programmer without being root**
```bash
wget vedder.se/Temp/49-stlinkv2.rules
sudo mv 49-stlinkv2.rules /etc/udev/rules.d/
sudo udevadm trigger
```

## IDE
### Prerequisites
#### On macOS/Linux

- `python3`, and `pip`

#### On Windows
- Python 3: https://www.python.org/downloads/. Make sure to click the box to add Python3 to your Environment.

### All platforms

1.  `pip install aqtinstall`
2.  `make qt_install`
3.  Open Qt Creator IDE installed in `tools/Qt/Tools/QtCreator/bin/qtcreator`
4.  With Qt Creator, open the vesc firmware Qt Creator project, named vesc.pro. You will find it in `Project/Qt Creator/vesc.pro`
5.  The IDE is configured by default to build 100_250 firmware, this can be changed in the bottom of the left panel, there you will find all hardware variants supported by VESC

## Upload to VESC
### Method 1 - Flash it using an STLink SWD debugger

1.  Build and flash the [bootloader](https://github.com/vedderb/bldc-bootloader) first
2.  Then `_flash` to the target of your choice. So for instance, for the VESC 100/250: 
```bash
make 100_250_flash
```

### Method 2 - Upload Firmware via VESC tool through USB

1.  Clone and build the firmware in **.bin** format as in the above Build instructions

In VESC tool

2.  Connect to the VESC
3.  Navigate to the Firmware tab on the left side menu 
4.  Click on Custom file tab
5.  Click on the folder icon to select the built firmware in .bin format (e.g. `build/100_250/100_250.bin`)

##### [ Reminder : It is normal to see VESC disconnects during the firmware upload process ]  
#####  **[ Warning : DO NOT DISCONNECT POWER/USB to VESC during the upload process, or you will risk bricking your VESC ]**  
#####  **[ Warning : ONLY DISCONNECT your VESC 10s after the upload loading bar completed and "FW Upload DONE" ]**

6.  Press the upload firmware button (downward arrow) on the bottom right to start upload the selected firmware.
7.  Wait for **10s** after the loading bar completed (Warning: unplug sooner will risk bricking your VESC)
8.  The VESC will disconnect itself after new firmware is uploaded.

## In case you bricked your VESC
you will need to upload a new working firmware to the VESC.  
However, to upload a firmware to a bricked VESC, you have to use a SWD Debugger.


## Contribute

Head to the [forums](https://vesc-project.com/forum) to get involved and improve this project.
Join the [Discord](https://discord.gg/JgvV5NwYts) for real-time support and chat

## Tags

Every firmware release has a tag. They are created as follows:

```bash
git tag -a [version] [commit] -m "VESC Firmware Version [version]"
git push --tags
```

## License

The software is released under the GNU General Public License version 3.0
