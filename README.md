# GTM-LAB
An unofficial goTenna Mesh protocol playground for curious minds of all ages

## Introduction
Hello and welcome to the GTM Lab. This is a fun project to study and 
re-implement the goTenna Mesh protocols on inexpensive hardware based on the 
ESP32 MCU and Semtech SX1276 long range radio modules (like HopeRF RFM95W or 
EByte E19-915M series), using only unrestricted, publicly available information.
All project code is open source under the [MIT license](/LICENSE).

While the long-term goal of the project is a fully GTM interoperable open-source 
protocol stack running on open-source hardware, the short term goal is really just 
to have fun and share the lessons learned.

So, if protocol reverse-engineering and exploring the sub-GHz ISM band are
your idea of fun too, grab a copy of the code and give it a spin!

## Development status
Executive summary: **can seamlessly send, receive and relay messages
and ACKs** to/from the goTenna ad-hoc mesh network.

### Radio capability:
- using inexpensive radio modules based on Semtech SX1276 chip
- mutually interoperable with goTenna Mesh at waveform level
- frequency corrected and temperature compensated
- per-packet AFC on RX
- listen-before-talk collision avoidance
- send and receive all currently known GTM frame types
- send and receive data packets up to 255 bytes
- TX power control from 0-20 dBm

### Endpoint capability
- full access (send and receive) endpoint for the goTenna mesh network
- message objects (up to 237 bytes free form data) and delivery ACKs
- all message classes (P2P, Shout, Group and Emergency), any TTL 1-15
- compatible with all tested GTM applications, plugins and SDKs
- simple connection via USB serial, or GTM-compatible BLE API (WIP)

### Relay/mesh capability:
The relay function is operational at net-positive performance;
it works on its own, or in conjunction with existing nodes, to extend
the mesh network coverage and performance, while avoiding any harmful
interference with existing network traffic.

### Hackability (you know you want to)
- everything in plain view, no proprietary/binary/NDA-walled black boxes
- all radio registers accessible for read/write in most operating modes via console commands
- most protocol variables (timing etc) can also be changed on the fly
- copious amounts of debugging information to illuminate even the darkest corners
- modular open source code - keep what you need and leave the rest
- easy to expand via native compatibility with the vast ESP32/Arduino ecosystem
- inexpensive open source hardware provides immunity to supply chain shocks or vendor lock-in
- in austere conditions, nodes can even be handmade from scavenged, easily
identifiable standard parts using stripboard, wire and a soldering iron
- GPS and WiFi onboard; LCD, sensors etc can also be easily added
- GTM/MOAN veterans might also appreciate:
  - external antenna connection
  - auto restart after power loss recovery
  - software **and hardware** reboot via USB/serial connection

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

- **Advanced:** radio modules based on the SX1231H chip (like RFM69) should
also work, but would require a rewrite of all register-specific code as the
register numbers and layouts are different.

**NOTE:** LoRa functionality is NOT REQUIRED and NOT USED. 
The choice of a LoRa capable radio was purely circumstantial, 
due to its availability and front-end capabilities. However, the LoRa modem 
is disabled at startup and never used - the radio operates in "legacy" FSK 
mode.

**HOWEVER ALSO NOTE:** Only ONE SPECIFIC LoRa transceiver model (SX1276) is
supported by gtm-lab currently. Double-check the model number before ordering,
as other models will not work.

## Installation

### Hardware requirements
You will need a supported ESP32+SX1276 board (see "Easy" under "Supported hardware" above)

If purchasing new hardware for this project, we recommend the T-Beam;
its 18650 Li-Ion battery holder with on-board management and built-in 
u-blox NEO GPS will come in handy in future experiments.
Make sure to select the 915MHz "USA" version, which covers all goTenna Mesh 
bands worldwide.

In the author's personal experience, the Sparkfun WRL-15006 had the worst RX
performance of all supported models and is therefore the least recommended.
If you already have one, use it while you're waiting for your T-Beam order
to arrive. :)

The 1W Sparkfun MicroMod shows promise but needs more testing.

### Build environment requirements
[Arduino IDE](https://www.arduino.cc/en/software/) 1.8+ with [ESP32 support](https://github.com/espressif/arduino-esp32)

### Dependencies
The only mandatory external dependency is the **Time** library
by Michael Margolis. Install it using the *Tools > Manage Libraries*
menu option in the Arduino IDE.

All other required libraries are included (see **Credits** section for details)

### Optional libraries
These libraries are used for optional extra functionality (install from Arduino IDE):

- **SparkFun u-blox GNSS** library by SparkFun Electronics - for GPS functions
- **AXP202X** library by Lewis He - for battery management on the T-Beam
- **ESP8266 and ESP32 OLED driver for SSD1306 displays** by ThingPulse - for screen functions on T-Beam

### Build and install

- Download the code (all files) from https://github.com/sybip/gtm-lab
- Open `gtm-lab` in the Arduino IDE
- **do not skip this step** Edit the `gtmConfig.h` file and change the 
`BOARD_TYPE` and `ISM_REGION` definitions to suit your environment
- avoid reusing `gtmConfig.h` files from older versions as option names may
have changed
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

## Core components
The original building blocks of the gtm-lab software

### gtmRadio
Implements the goTenna Mesh FHSS waveform using a Semtech SX1276 radio.
You need this module to communicate with other GTM nodes on the network.
Uses `LoRaX` and `RS-mod-FCR` libraries internally.

### gtmNode
Implements the data packet handling logic: duplicate filtering, relaying,
queuing, input and output.
Uses `gtmRadio` for network access, and provides hardware-independent,
waveform-agnostic functions for sending and receiving messages in `gtmAPI`.

## Optional / auxiliary code
This is early stage work in progress, and the module descriptions below
should be viewed as a statement of goals rather than current capabilities.

### gtmBLE
Using the ESP32 built-in Bluetooth module, this module implements a BLE
service that emulates the goTenna Mesh device's BLE front-end, targeting
100% compatibility with existing goTenna apps and SDKs.
A no-op stub command handler is also included, to enable testing even on
generic ESP32 boards without a long-range radio.

### gtmAPI
This module sits between `gtmBLE` and `gtmNode`, and translates BLE API
requests into actual gtmRadio operations, and back.
In other words, if `gtmBLE` emulates the user side of the GTM and `gtmNode`
the network side, `gtmAPI` connects the two of them together to form a
system that is drop-in compatible with the original goTenna Mesh
in all functional aspects.

Example: [02-user-device.ino](https://gist.github.com/sybip/71ef23ce380ee6ad53170d6b9bf44aef)

## Issues, challenges and limitations

### Frequency accuracy and stability
In general, the observed frequency performance of all tested radio modules has
been consistently inferior to the original GTM in terms of both accuracy and
stability. This has been addressed partially by employing frequency offset
correction and temperature compensation functionality in the code, however,
it remains an area of ongoing study and further improvement.

### RF power output
Most common radio modules (with the exception of Micromod/EByte) have a
maximum RF power output of only 20dBm (100mW) at the antenna port,
significantly lower than the GTM's 1W.

### Power consumption
The code is not just completely unoptimized for power usage, but in some
places the power efficiency is currently deliberately sacrificed for the
sake of increased operating flexibility and transparency.

Some opportunities for future power optimization include:

- using the built-in automation features of the SX radios, such as AFC
- interrupt-driven I/O instead of polling
- non-blocking BLE to allow all code to run on a single core
- ... and a lot more

## Documentation and further info
All technical information relevant to this project is (or will be) published 
under a copyleft license in the [pyGT project wiki](https://github.com/sybip/pyGT/wiki/), 
and of course everyone's invited to share their findings if they want to.

No Github account? - no problem! Just shoot a bitmessage to the author at 
BM-2cW2P5qmucxH4jWP2JEyqKA12VhTJibUXL, or send a note (non confidential only)
to npub18e2gkqjducs9avdn09gakr9h5v3tdsst8mamjgu4wfv3l6drhjaqmu5t7d on Nostr.

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
- [ESP8266 and ESP32 OLED driver for SSD1306 displays](https://github.com/ThingPulse/esp8266-oled-ssd1306)
 Copyright Thingpulse, Daniel Eichhorn, Fabrice Weinberg, MIT license
- [AXP202X Library](https://github.com/lewisxhe/AXP202X_Library)
 Copyright Lewis He, MIT license

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
