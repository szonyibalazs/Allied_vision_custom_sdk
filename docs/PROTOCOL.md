# Protocol notes and camera quirks

How `u3v.c` talks to an Alvium USB3 camera, and the behaviours that were not obvious from the
specification. Everything here was observed on two cameras (1280x960 and 1936x1216, USB ID
`1ab2:0001`); addresses for other models should be taken from the camera's own GenICam XML
(`./probe` + `genicam_regs.py`).

All multi-byte values are **little-endian**.

## USB layout

| Interface | Endpoint | Use |
|---|---|---|
| 0 | `0x01` OUT / `0x81` IN (bulk) | GenCP control channel |
| 1 | `0x85` IN (bulk) | Image stream |

There is no Linux kernel driver for this; `libusb` claims both interfaces. On USB 3 the bulk
max packet size is 1024 and `max_cmd` / `max_ack` read back as 1024 (512 on USB 2).

## GenCP control channel

Command packet (12-byte header + body):

| Offset | Size | Field |
|---|---|---|
| 0 | u32 | magic `0x43563355` (bytes `U3VC`) |
| 4 | u16 | flags; `1 << 14` = request acknowledge |
| 6 | u16 | command: `0x0800` READMEM, `0x0802` WRITEMEM |
| 8 | u16 | length of the body in bytes |
| 10 | u16 | request id (increment each packet) |
| 12 | ... | body |

READMEM body: `u64 address, u16 reserved(0), u16 count`. WRITEMEM body: `u64 address`, then data.

Acknowledge: same magic; the `u16` at offset 4 is the **status** (0 = success), command is
`0x0801` / `0x0803`, and read data starts at offset 12. A read ack plus header must fit in
`max_ack`, so read in chunks (the driver uses well under that; the XML download uses 480 bytes).

Status codes seen: `0x8002` INVALID_PARAMETER, `0x8003` INVALID_ADDRESS, `0x8004` WRITE_PROTECT,
`0x8007` BUSY, `0xA003` SI_PAYLOAD_SIZE_NOT_ALIGNED, `0xA004` SI_REGISTERS_INCONSISTENT.
Check the status of **every** write.

## Bootstrap registers

| Register | Address |
|---|---|
| Manufacturer name (string) | `0x0004` |
| Device capability | `0x01c4` |
| Manifest table address (u64) | `0x01d0` |
| SBRM address (u64) | `0x01d8` (reads `0x10000`) |
| SBRM: max command transfer / max ack transfer / SIRM address | SBRM `+0x14` / `+0x18` / `+0x20` (SIRM reads `0x20000`) |

Manifest table: `u64` entry count at `+0`, first entry at `+8`: `u32 schema @+4`, `u64 address @+8`,
`u64 size @+16`. The address/size point to the camera's GenICam XML, stored as a **zip** archive in
device memory (about 50 KB to 850 KB depending on model).

## Stream setup (SIRM)

The host must describe the transfer layout before enabling the stream. Registers at the SIRM base:

| Offset | Meaning |
|---|---|
| `+0x00` | info; transfer alignment = `1 << (info >> 24)` (1024 here) |
| `+0x04` | control; bit 0 = stream enable |
| `+0x08 .. +0x14` | device-reported required payload / leader / trailer sizes (**read-only**; writing gives `0x8004`) |
| `+0x18` | max leader size (host writes) |
| `+0x1c` | payload chunk size |
| `+0x20` | payload chunk count |
| `+0x24` | transfer 1 size (remainder chunk) |
| `+0x28` | transfer 2 size |
| `+0x2c` | max trailer size |

The plan the driver writes: read the required payload size from `+0x08`; chunk size =
`min(required, 1 MiB rounded down to the alignment)`; count = `required / chunk`; transfer 1 =
remainder rounded **up** to the alignment; leader = trailer = alignment; transfer 2 = 0. Write all
of it, then set enable. Enabling with an incomplete or stale plan fails with `0xA004`. Recompute the
plan after changing pixel format or ROI.

## Stream blocks

Per frame the camera sends three bulk transfers (the host reads them in order):

- **Leader**, 52 bytes (the camera sends it unpadded even though the host declared 1024): `"U3VL"` @0,
  leader size `u16 @6`, block id `u64 @8`, payload type `u16 @18`, timestamp `u64 @20` (ns),
  pixel format `u32 @28`, width `u32 @32`, height `u32 @36`.
- **Payload**, `width * height * bytes_per_pixel` bytes (Mono8 = 1, RGB8 = 3, packed R,G,B).
- **Trailer**, 32 bytes: `"U3VT"` @0, trailer size `u16 @6`, block id `u64 @8`, status `u16 @16`,
  valid payload size `u64 @20`. A healthy frame has status `0x0000`; `0xA100` means the camera discarded
  data (the valid payload size then tells how much of the image is real).

## Feature registers (from the cameras' GenICam XML)

| Feature | Address | Notes |
|---|---|---|
| AcquisitionStart / Stop | `0x10220000` / `0x10220010` | write 1 |
| PixelFormat | `0x10100000` | Mono8 `0x01080001`, RGB8 `0x02180014` |
| Width / Height | `0x10120000` / `0x10130000` | |
| FrameRateEnable | `0x102200e0` | write 1 to honour AcquisitionFrameRate |
| AcquisitionFrameRate | `0x10220100` | float32 |
| TestPattern | `0x101c1000` | 1 = grey diagonal ramp |
| DeviceLinkSpeed | `0x10050000` | u64, read-only |
| DeviceLinkThroughputLimitMode / Limit | `0x10040000` / `0x10040010` | **do not write**, see below |
| DeviceReset | `0x100A0000` | write 1 |
| ExposureTime | `0x10240000` | float32 (1280x960 model) or double (1936x1216 model) |
| ExposureAuto / GainAuto / BalanceWhiteAuto | `0x1b1c1000` / `0x1b1c4000` / `0x1b1ca000` | 0 Off, 1 Once, 2 Continuous |
| ExposureAutoMax | `0x1b1c3004` (float32) or `0x1b1c3008` (double) | microseconds |
| AcquisitionStatus | `0x10220300 + selector * 4` | selectors: 1 AcquisitionActive, 4 FrameActive, 5 ExposureActive, 6 ReadoutActive; bit 0 |

Float registers: check the `Length` in the XML (4 = float32, 8 = double); `u3v.c` tries the 8-byte
form first and falls back to the 4-byte one.

## Camera quirks

1. **The control state machine can wedge.** After a failed or aborted transaction the camera may answer
   everything with `0x8002`, or not at all. A `libusb_reset_device()` plus draining the control IN endpoint
   before the first command clears the usual cases (`u3v_open` does this when asked). If even that fails
   (`./u3v unstick` also tries), unplug and replug the camera.
2. **A silent USB 3 stream.** Sometimes every register is correct, `AcquisitionStart` and the stream enable
   succeed, `AcquisitionStatus` shows frames being exposed and read out - yet endpoint `0x85` returns nothing
   (and the xHCI controller raises no interrupts). The camera's own `DeviceReset` (write 1 to `0x100A0000`)
   fixes it. The device re-enumerates; wait about 15 s. `./u3v devreset` does this.
3. **Truncated frames** (trailer status `0xA100`, partial image, no error otherwise) have three separate causes:
   - USB 2 link: payload x fps exceeds the link, e.g. 1280x960 Mono8 only fits at ~1 fps.
   - Writing `0` to `DeviceLinkThroughputLimitMode`. Leave the register alone; the power-on value works.
   - A slow stdout consumer. The camera has a small buffer; if the host stops reading for a moment the rest
     of the frame is dropped. Keep reading on a dedicated thread (the driver's ring buffer and
     `cam_view.py`'s reader thread exist for this).
4. **Parameters are locked while acquiring** (`0x8004` on PixelFormat, ROI, ...). Write `AcquisitionStop` first.
5. **The camera remembers state across runs**, including auto exposure. A leftover 109 ms exposure caps the
   frame rate at 1/0.109 = 9.2 fps and the camera clamps `AcquisitionFrameRate` accordingly (reads back lower
   than you set). `live` therefore resets auto features and exposure at start, then sets `ExposureAutoMax` to
   50 % of the frame period (exposure plus readout must fit in one period).
6. **Do not `SIGKILL` the driver.** It leaves the camera streaming. Use `SIGTERM` (it stops acquisition and the
   stream) or run `./u3v stop`.
7. **Frame rate ceiling.** On the 1280x960 model the camera clamps `AcquisitionFrameRate` to about 41 fps.
   This is a camera limit, not USB bandwidth (40 fps is about 150 MB/s).
8. **Interface claim fails** (`claim ctrl failed`): another process (an old `u3v`, Vimba, or a viewer) still holds
   the camera.

## Reading the camera's XML

```bash
./probe cam.zip                 # downloads the zip from device memory (manifest table)
mkdir xml && unzip cam.zip -d xml
python3 genicam_regs.py xml/GenICam_Default.xml ExposureTime PixelFormat
```

`genicam_regs.py` follows `pValue` / `Offset` chains. Selector-indexed or SwissKnife-computed registers need
a manual look at the XML.
