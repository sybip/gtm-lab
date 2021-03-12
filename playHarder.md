# playHarder - the gtm-lab test console

The gtm-lab suite contains a serial console for running various tests
intended to help our understanding of the protocol. Some of the commands are
for modifying various operating parameters, others allow direct interaction
with the low-level radio registers, and others execute simple tasks, like
sending test messages. All commands are prefixed with a bang (!) character.

Due to the informal and exploratory nature of the project, the test console
is subject to frequent breaking changes. For this reason, it should 
**absolutely not be treated as an API** for any serious development.

Keep in mind: it's only a *playground*, not a *firmware*. So **let's play**.

## Supported commands

| cmd | args          | example | description
|-----|---------------|---------|------------
| !l  | **v/d/i/w/e** | !ld     | set console logging level <br/> v=VERBOSE d=DEBUG i=INFO w=WARN e=ERROR
| !m  | **0/1/r/t**   | !m0     | set radio mode (will also pause chan scanning) <br/>0=SLEEP 1=STANDBY r=RX t=TX
| !sg | **X**         | !sg8    | set geopolitical region to X (1,2,4,8)
| !sp | **DD**        | !sp10   | set TX power dBm (DD in decimal 00 - 20)
| !sr | **0/1**       | !sr1    | set mesh relay function 0=OFF, 1=ON
| !sa | **XXXX**      | !sa3fff | set App ID to XXXX (in HEX)
| !st | **XY**        | !st32   | set TTL for test msgs: init=X, curr=Y
| !da |               |         | dump ALL radio registers
| !di |               |         | dump radio IRQ registers
| !df |               |         | dump entire FIFO content<br/>(use !dr00 to read one FIFO byte)
| !ds |               |         | dump state variables (incomplete)
| !dr | **XX**        | !dr3e   | dump radio register XX (in HEX)
| !w  | **XXYY**      | !w3520  | write to radio register XX value YY (XX and YY in HEX)
| !c  | **DD**        | !c01    | change channel (DD in decimal)
| !h0 |               |         | *soft hold* - pause channel scanning until next protocol event
| !h1 |               |         | resume normal channel scanning behaviour
| !h2 |               |         | *hard hold* - stay on current chan even when protocol says to move
| !td |               |         | transmit a random ACK packet directly
| !tt |               |         | transmit a TIME packet directly
| !ta | XXXX          | !tac4f3 | transmit ACK packet (using queue) for hash ID XXXX (in HEX)<br/>if no hash ID given, a random one will be used)
| !tm | XX            | !tmb4   | transmit a SHOUT message with XX bytes of text (in HEX)<br/>if no size given, a random one will be used
| !tx | **XXXX...XX** |         | transmit a data object - supplied in HEX
| !tk |               |         | transmit a TAK PLI beacon