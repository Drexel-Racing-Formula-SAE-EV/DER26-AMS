# Atomic AMS replacement-image protocol

The ESP32 and AMS share the canonical definitions in
`common/ams_hil_image_protocol.h`. Multi-byte values are big-endian.

For the current 75s6p/120-temperature topology, one 100 ms publication is:

| Order | CAN ID | Count | Purpose |
|---:|---:|---:|---|
| 1 | `0x200`–`0x202` | 3 | Measurement, truth, and summary |
| 2 | `0x212` | 1 | START generation |
| 3 | `0x210` | 25 | Cell-voltage triplets |
| 4 | `0x211` | 40 | Temperature triplets |
| 5 | `0x212` | 1 | COMMIT generation and CRC32 |

The atomic image is 67 frames; the complete plant burst is 70 frames.

## Frame contract

START (`0x212`):

```text
byte 0  protocol version
byte 1  START opcode
byte 2  generation
byte 3  segment count
byte 4  total cell/group count
byte 5  total temperature count
byte 6  expected cell-frame count
byte 7  expected temperature-frame count
```

Cell/temperature data (`0x210`/`0x211`):

```text
byte 0    generation
byte 1    segment in bits 7:5, first local index in bits 4:0
bytes 2:7 three signed/unsigned 16-bit samples
```

COMMIT (`0x212`):

```text
byte 0    protocol version
byte 1    COMMIT opcode
byte 2    generation
byte 3    reserved, zero
bytes 4:7 CRC32
```

The CRC is IEEE CRC32 over every quantized `uint16` cell millivolt value in
segment/index order, followed by every quantized `int16` deci-degree-C value
interpreted as its two-byte unsigned representation in segment/index order.

## Receiver behavior

The AMS keeps the incoming image private until COMMIT. It:

- validates the declared topology and expected frame counts;
- requires the generation on every data and COMMIT frame to match START;
- records independent received bitmaps for all 75 cells and 120 temperatures;
- accepts arbitrary frame order;
- accepts an identical duplicate but invalidates a conflicting duplicate;
- treats a repeated identical START for the active generation as idempotent,
  preserving already staged data and the original deadline;
- abandons an incomplete generation when a newer START arrives;
- rejects incomplete images, CRC failures, and assemblies older than 250 ms;
- rejects replayed or backward uint8 generations while the last image is
  fresh;
- permits generation resynchronization only after the last committed image has
  exceeded the 500 ms freshness timeout, supporting an independent ESP32
  power-cycle without allowing it to overwrite a fresh image;
- copies the complete image and all freshness timestamps under the existing
  ADBMS lock in one commit operation.

A failed generation never changes the published `smb_ics` image. Normal AMS
freshness and stale-sensor logic therefore operates on the last complete
generation and fails closed when no new generation commits.

## Software evidence

The focused AMS host test injects complete, reordered, dropped, duplicated,
conflicting, CRC-corrupted, replayed, delayed, timed-out, reset, and
tick-wrapped sequences:

```bash
make -C AMS/host_tests clean hil-adbms-test
```

The shared protocol test also checks address packing, big-endian conversion,
and standard CRC vectors:

```bash
make -C HiL/esp32_plant/tests clean test
```

These tests establish software semantics. They do not qualify MCP2515 timing,
physical bus loading, termination, bus-off recovery, or actual task deadlines.
The bench-facing AMS diagnostic command is `bringup hil-image`.
