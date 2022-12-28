# GTM-LAB
An unofficial goTenna Mesh protocol playground

## Introduction
Hello and welcome to the GTM Lab. This is a fun project to study and 
re-implement the goTenna Mesh protocols on inexpensive hardware based on the 
ESP32 MCU and Semtech SX1276 long range radio modules (like HopeRF RFM95W), 
using only unrestricted, publicly available information. All project code is 
open source under the [MIT license](/LICENSE).

While the long-term goal of the project is a fully GTM interoperable open-source 
protocol stack running on open-source hardware, the short term goal is really just 
to have fun and share the lessons learned.

So, if protocol reverse-engineering and exploring the sub-GHz ISM band are
your idea of fun too, grab a copy of the code and give it a spin!

## Development status
Executive summary: **can send, receive and relay messages** to/from the 
goTenna mesh network.

### RX capability:
- full GTM waveform reception including error correction
- control packet (SYNC/ACK) reception
- data packet reception and large payload reassembly

### TX capability:
- assemble radio packets with header and error correction codes
- generate ACK, SYNC, DATA, TIME packets with correct formats
- packetize and send large payloads - up to 255 bytes
- listen-before-talk on all channels

### Relay/mesh capability:
The relay function is operational at net-positive performance; 
it works on its own, or in conjunction with existing nodes, to extend 
the mesh network coverage and performance, while avoiding any harmful 
interference with existing network traffic.

## Interop testing
The software is tested for interoperation with:
- itself
- goTenna Mesh, firmware 1.1.8

## Code quality
As should be painfully obvious, this is experimental proof-of-concept code, 
not intended and not suitable for any serious application.

## Supported hardware
- **Easy:** out of the box, the code contains pre-defined configurations for 
three widely available, **open-source hardware** boards, selectable via the 
`BOARD_TYPE` definition in the configuration file:
  - [LILYGO T-Beam 1.0](https://github.com/Xinyuan-LilyGO/LilyGO-T-Beam) and newer
  - [Sparkfun WRL-15006](https://github.com/sparkfun/ESP32_LoRa_1Ch_Gateway) (not recommended)
  - [Sparkfun MicroMod](https://github.com/sparkfun/SparkFun_MicroMod_Main_Board_Single) with [ESP32 CPU](https://github.com/sparkfun/MicroMod_ESP32_Processor) and [1W LoRa module](https://github.com/sparkfun/MicroMod_Function_LoRa_1W)

- **Medium:** any ESP32 board with an SPI-connected RFM95W or compatible 
SX1276-based radio should work, as long as the correct pin connections are 
configured in the source (using the existing definitions as an example)

- **Advanced:** RFM69-type (SX1231H based) radios should also work, but would 
require a rewrite of all register-specific code as the register numbers and 
layouts are different.

**NOTE:** LoRa functionality is NOT REQUIRED and NOT USED. 
The choice of a LoRa capable radio was purely circumstantial, 
due to its availability and front-end capabilities. However, the LoRa modem 
is disabled at startup and never used - the radio operates in "legacy" FSK 
mode.

## Installation

### Hardware requirements
You will need a supported ESP32+SX1276 board (see "Easy" under "Supported hardware" above)

If purchasing new hardware for this project, we recommend the T-Beam;
its 18650 Li-Ion battery holder with on-board management and built-in 
u-blox NEO GPS will come in handy in future experiments.
Make sure to select the 915MHz "USA" version, which covers all goTenna Mesh 
bands worldwide.

The Sparkfun WRL-15006 has the worst RX performance of all supported models 
and is therefore the least recommended. If you already have one, use it while
you're waiting for your T-Beam order to arrive. :)

The 1W Sparkfun MicroMod shows promise but needs more testing.

### Build environment requirements
[Arduino IDE](https://www.arduino.cc/en/software/) 1.8+ with [ESP32 support](https://github.com/espressif/arduino-esp32)

If setting up Arduino/ESP32 for the first time, refer to the 
[recommended installation instructions](https://github.com/espressif/arduino-esp32/blob/master/docs/arduino-ide/boards_manager.md).

### Dependencies
The only external dependency is the **Time** library by Michael Margolis. 
Install it using the *Tools > Manage Libraries* menu option in the 
Arduino IDE.

All other required libraries are included (see **Credits** section for details)

### Optional libraries
These libraries are used for optional extra functionality:

- **SparkFun u-blox GNSS** library by SparkFun Electronics - for GPS functions

### Build and install

- Download the code (all files) from https://github.com/sybip/gtm-lab
- Open `gtm-lab` in the Arduino IDE
- **do not skip this step** Edit the `gtmConfig.h` file and change the 
`BOARD_TYPE` and `ISM_REGION` definitions to suit your environment
- Select your ESP32 board from the Arduino *Tools > Board* menu (e.g. *T-Beam*)
- Compile and upload to your board.

All done - you're ready to play!

## Operation
### Basic operation - packet logger and simple shouter
Connect to the board's USB port with a serial terminal at 115200bps.
Using your goTenna Mesh kit, generate some goTenna message traffic (ping, 
location tethering etc).
The messages should appear in the serial terminal:
```
I (6475353) GTMLAB: RX CCh=01 SYNC(1): chIDX=1, frags=2, iniTTL=3, curTTL=2
I (6475404) GTMLAB: RX DCh=42 DATA(2): len=90, fragIDX=0
I (6475436) GTMLAB: RX DCh=41 DATA(2): len=45, fragIDX=1
I (6475436) GTMLAB: complete: len=135, hash=0xcafe, time=88ms
RX_MSG:030212|003fff55555555555500ff0000fb100100......
```

Type a short text in the serial console and press Enter. The text will be 
transmitted as a SHOUT class message, and (assuming that all your settings 
are correct) received by the goTenna Mesh clients in range.

**NOTE:** By default, gtm-lab will transmit on the **testnet** AppID 0x7357, 
therefore shouted messages will **not** be received by the goTenna app.
This is intentional to minimize unwanted interference with goTenna app traffic.
To enable sending to goTenna app, enter command `!sa3fff` in the gtm-lab
console (set AppID to 0x3fff), and messages will become visible.

### Radio protocol exploration
Change the logging verbosity to "Debug" (console command `!ld`) or even 
"Verbose" (`!lv`) to view details of received packets, correction protocols, 
channel hopping etc.
Be warned that Verbose mode will break protocol timings due to serial port 
congestion. On boards that support high-speed serial, like the T-Beam, this 
may be mitigated by increasing the serial port speed to 1Mbps via console 
command `!ssh`.

### Message formats exploration
The output lines prefixed with `RX_ACK` and `RX_MSG` contain full hex dumps of
received radio traffic (see the source code for details on what some of those
bytes mean). All message classes are received, including P2P encrypted messages.

These objects can be further processed on the computer using a Python or PERL 
script to study the format of the packets and extract useful information.

Example: [metalogger](https://gist.github.com/sybip/4b754010cf5a667c161f3ae3d8d106ef).

The pyGT module [`gtairobj.py`](https://github.com/sybip/pyGT/blob/master/gtairobj.py)
contains functions for parsing and creating GTM radio protocol objects compatible with
the gtm-lab test console.

### Test console
Implemented in the `playHarder.cpp` file and documented in 
[playHarder.md](https://github.com/sybip/gtm-lab/blob/main/playHarder.md), 
and subject to continuous breaking changes.

## Documentation and further info
All technical information relevant to this project is (or will be) published 
under a copyleft license in the [pyGT project wiki](https://github.com/sybip/pyGT/wiki/), 
and of course everyone's invited to share their findings if they want to.

No Github account? - no problem! Just shoot a bitmessage to the author at 
BM-2cW2P5qmucxH4jWP2JEyqKA12VhTJibUXL

## Credits

### Software libraries
- [Arduino Time library](https://github.com/PaulStoffregen/Time) Copyright Michael Margolis, LGPLv2.1 license
- [Arduino LoRa library](https://github.com/sandeepmistry/arduino-LoRa) Copyright Sandeep Mistry, MIT license 
(modified to operate in non-LoRa mode and renamed to **LoRaX** to avoid a name conflict)
- [Reed-Solomon FEC library](https://github.com/simonyipeter/Arduino-FEC) 
Copyright Mike Lubinets, Simoniy Peter, MIT license (modded to support variable length messages and 
alternative FCRs, renamed to **RS-mod-FCR**)
- [SparkFun u-blox GNSS Arduino Library](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library)
Copyright SparkFun Electronics, MIT license

### Knowledge and information
- [Tim](https://nitter.net/bjt2n3904) and [Woody](https://nitter.net/tb69rr) made 
**[it](https://github.com/tkuester/gr-gotenna)** look easy: 
https://www.youtube.com/watch?v=pKP74WGa_s0
- [Clayton Smith](https://github.com/argilo) brought it all the way home with 
https://github.com/argilo/gr-tenna
- [Alec Murphy's ESP32GTM](https://gitlab.com/almurphy/ESP32GTM) for Arduino
inspiration and some code snippets
- and of course [every single person mentioned on this page](https://github.com/sybip/pyGT/wiki/Resources)

Thank you all.

## Now go have some fun!

---
This project is not affiliated with goTenna inc.
