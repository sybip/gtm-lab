# GTM-LAB
An unofficial goTenna Mesh protocol playground

## Introduction
Hello and welcome to the GTM Lab. This is a fun project to study and 
re-implement the goTenna Mesh protocols on inexpensive hardware based on the 
ESP32 MCU and RFM95W radio modules, using only unrestricted, publicly available 
information. All project code is open source under the [MIT license](/LICENSE).

The inspiration for the project was a message posted by user **dillon** 
on the goTenna community forum:
> put a goTenna assembly line in every home and let’s see them fight this door-to-door

While the long-term goal of the project is a fully GTM interoperable open-source 
protocol stack running on open-source hardware, the short term goal is really just 
to have fun and share the lessons learned.

So, if protocol reverse-engineering and exploring the sub-GHz ISM band are 
your idea of fun too, grab a copy of the code and give it a spin!

## Development status
Executive summary: **can send and receive messages** to/from the goTenna 
mesh network.

### RX capability:
- channel hopping
- preamble and syncword detection
- control channel scanning with adjustable dwell time
- error correction
- control packet (SYNC/ACK) reception
- data packet reception and reassembly

### TX capability:
- assemble radio packets with header and error correction codes
- generate ACK, SYNC, DATA payloads with correct formats
- packetize and send large messages up to 255 bytes

### TODO - not yet implemented:
- listen-before-transmit control channel sensing
- RSSI reading
- better documentation
- relay/mesh functionality
- better buffer overflow protection
- clean up the RX flow (refactor as FSM?)
- use interrupts instead of polling
- optimize power usage

## Interop testing
The software is tested for interoperation with:
- itself
- goTenna Mesh, firmware 1.1.8

## Code quality
Could be indulgently described as *barely functional early-proof-of-concept*

## Supported hardware
- **Easy:** out of the box, the code contains pre-defined configurations for two 
consumer boards, selectable via the `BOARD_TYPE` line in the source file:
  - [Sparkfun WRL-15006](https://github.com/sparkfun/ESP32_LoRa_1Ch_Gateway)
  - [LILYGO t-beam 1.0](https://github.com/Xinyuan-LilyGO/LilyGO-T-Beam) and newer

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
- Download the code (all files) from https://github.com/sybip/gtm-lab
- **do not skip this step** Edit the `gtmConfig.h` file and change the 
`BOARD_TYPE` and `ISM_REGION` definitions to suit your environment; 
- Compile and upload to your board using Arduino 1.8+ with ESP32 1.0+ add-ons

You're ready to play!

### Dependencies
All required libraries are included, slightly modified from the original ones 
to support the (somewhat left-field) requirements of this project:
- [Arduino LoRa](https://github.com/sandeepmistry/arduino-LoRa) library 
modified to operate in non-LoRa mode, renamed to **LoRaX** to avoid a name conflict
- [Arduino RS-FEC](https://github.com/simonyipeter/Arduino-FEC) (Reed-Solomon) 
library was modded to support variable length messages and alternative FCRs, 
and renamed to **RS-mod-FCR**

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
RX_MSG:0302|003fff55555555555500ff0000fb100100......
```

Type a short text in the serial console and press Enter. The text will be 
transmitted as a SHOUT class message, and (assuming that all your settings 
are correct) received by the goTenna Mesh clients in range.

### Radio protocol exploration
Change the `VERBOSITY` line to `ESP_LOG_DEBUG` or even `ESP_LOG_VERBOSE` 
to view details of received packets, correction protocols, channel hopping etc.

### Message formats exploration
The output lines prefixed with `RX_ACK` and `RX_MSG` contain full hex dumps of 
received radio traffic (see the source code for details on what some of those 
bytes mean). All message classes are received, including encrypted messages.

These objects can be further processed on the computer using a Python or PERL 
script to study the format of the packets and extract useful information.

### Test console
Documented inside the playHard.h file and subject to continuous change.

## Documentation and further info
All technical information relevant to this project is (or will be) published 
in the [pyGT project wiki](https://github.com/sybip/pyGT/wiki/), and of course 
everyone's welcome to share their findings if they want to.

No Github account? - no problem! Just shoot a bitmessage to the author at 
BM-2cW2P5qmucxH4jWP2JEyqKA12VhTJibUXL .

## Credits

- [Tim](https://nitter.net/bjt2n3904) and [Woody](https://nitter.net/tb69rr) made 
**[it](https://github.com/tkuester/gr-gotenna)** look easy: 
https://www.youtube.com/watch?v=pKP74WGa_s0
- [Clayton Smith](https://github.com/argilo) brought it all the way home with 
https://github.com/argilo/gr-tenna
- [Sandeep Mistry](https://github.com/sandeepmistry/arduino-LoRa), 
[Mike Lubinets](https://github.com/mersinvald/Reed-Solomon), 
[Simoniy Peter](https://github.com/simonyipeter/Arduino-FEC), 
[Alec Murphy](https://gitlab.com/almurphy/ESP32GTM), 
many thanks and humble apologies for butchering your code :)
- and [everyone mentioned on this page](https://github.com/sybip/pyGT/wiki/Resources)

Thank you. 

## Now go have some fun!

---
This project is not affiliated with goTenna inc.
