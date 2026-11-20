# Allied Vision custom SDK

A small, dependency-light **USB3 Vision driver for Allied Vision Alvium cameras on Linux** that
talks to the camera directly over `libusb` - no Vimba X, no GenICam runtime, no vendor binaries.

Alvium USB cameras are *not* UVC webcams: there is no `/dev/video*`, and the vendor SDK is
heavy. This project implements the USB3 Vision protocol (GenCP control channel + bulk image
stream) from scratch, so a camera can be brought up and streamed with a few hundred lines of C
and almost no CPU.

> Unofficial. Not affiliated with or endorsed by Allied Vision. Written against the public
> USB3 Vision / GenCP specifications; the open-source [Aravis](https://github.com/AravisProject/aravis)
> project was used as a protocol reference only (no code copied).

## Status

| | |
|---|---|
| Platform | Linux (developed and measured on x86-64) |
| Cameras tested | two Alvium USB3 cameras, USB ID `1ab2:0001`: a 1280x960 model and a 1936x1216 model |
| Pixel formats | Mono8, RGB8 (auto-detected in the stream) |
| Verified | full frames at 15 fps (1936x1216 RGB8) and up to ~41 fps (1280x960, the camera's own limit) |
| Not done yet | Raspberry Pi testing, asynchronous multi-URB engine, v4l2loopback output, multi-camera scheduling |
| License | not chosen yet - see "License" below |

## What is in here

| File | Purpose |
|---|---|
| `u3v.c` | The driver. GenCP read/write, stream setup, and a `live` mode that writes the raw U3V block stream to stdout. Also many low-level bring-up/debug modes. |
| `cam_view.py` | Live viewer: starts `u3v live`, parses frames on a reader thread, shows them with OpenCV. Reference for consuming the stream. |
| `probe.c` | Reads bootstrap registers and downloads the camera's GenICam XML (zip). |
| `genicam_regs.py` | Resolves a GenICam feature name (e.g. `ExposureTime`) to a register address from that XML. |
| `99-alvium-u3v.rules` | udev rule for non-root access. |
| `docs/PROTOCOL.md` | Register map, wire formats, and the camera quirks that cost the most time to find. |

## Requirements

- Linux, a C compiler, `libusb-1.0` development headers (`sudo apt install build-essential libusb-1.0-0-dev`)
- For the viewer: Python 3, `numpy`, `opencv-python`
- The camera on a **USB 3 port**. On USB 2 the link is too slow for full frames and the camera
  silently truncates them (see PROTOCOL.md).

## Quick start

```bash
make                                   # builds ./u3v and ./probe
sudo cp 99-alvium-u3v.rules /etc/udev/rules.d/ && sudo udevadm control --reload && sudo udevadm trigger
# replug the camera, then:

./u3v info                             # identify the camera, show geometry / pixel format
python3 cam_view.py 15                 # live view at 15 fps (q / Esc to quit)
```

`cam_view.py` looks for the driver next to itself; set `U3V_BIN=/path/to/u3v` to override.

If `./u3v live` produces no frames (the stream stays silent after a replug or a failed
start), reset the camera and try again:

```bash
./u3v devreset && sleep 15 && python3 cam_view.py 15
```

## Using the stream from your own code

`./u3v live [fps] [mono|rgb]` writes the raw USB3 Vision block stream to **stdout** and all
diagnostics to **stderr**. Each frame is:

```
"U3VL" leader (52 B) | payload (width*height*bytes_per_pixel) | "U3VT" trailer (32 B)
```

Leader fields used by `cam_view.py` (little-endian): pixel format `u32 @28`
(`0x01080001` = Mono8, `0x02180014` = RGB8), width `u32 @32`, height `u32 @36`,
timestamp `u64 @20` (ns). Check the trailer magic after the payload; a missing trailer
means a truncated frame, so drop it and resync on the next `U3VL`.
`cam_view.py` (`FrameReader`) is a compact reference parser. Any language that can read a pipe works.

Shutdown: send `SIGTERM`. The driver stops acquisition and the stream before exiting. Do not
`SIGKILL` it - the camera keeps streaming and may need `./u3v stop` or `./u3v devreset`.

## Driver modes

Run `./u3v` modes with the camera plugged in; most print to stderr.

| Mode | What it does |
|---|---|
| `live [fps] [mono\|rgb]` | Continuous raw stream on stdout (see above). |
| `info` | Camera identity, geometry, pixel format, frame rate. |
| `grab out.raw [W H [fps]] [testpat]` | Capture a few frames to a file (also `out.raw.stream`). |
| `stop` | Stop acquisition and stream cleanly. |
| `devreset` | Camera `DeviceReset` - fixes a silent USB3 stream. The device re-enumerates; wait ~15 s. |
| `unstick` | USB-level recovery when GenCP replies `INVALID_PARAMETER` to everything. |
| `snap`, `frame`, `streamon`, `rawdump`, `epdebug`, `sirmtest`, `sirmplan`, `linkinfo`, `drainstream` | Low-level bring-up and debug helpers. |

Environment for `live` (RGB8 cameras): `U3V_NOAUTO=1` disables auto exposure / gain / white
balance; `U3V_AUTO=egw` selects a subset (`e`xposure, `g`ain, `w`hite balance). With auto on, the
auto-exposure ceiling is set to 50 % of the frame period so the requested frame rate is kept.

## Finding registers for your camera model

Register addresses come from the camera's own GenICam XML:

```bash
./probe my_camera.zip && unzip my_camera.zip -d xml/        # unzip into a fresh directory
python3 genicam_regs.py xml/GenICam_Default.xml ExposureTime Gain PixelFormat
python3 genicam_regs.py xml/GenICam_Default.xml --grep Balance
```

Addresses differ between models and firmware (for example `ExposureTime` is a 32-bit float on
one tested camera and a 64-bit double on the other). The XML is vendor material, so it is
git-ignored here - extract it from your own camera.

## Performance

Measured on an x86-64 desktop (40 logical cores), 1936x1216 RGB8 at 15 fps (about 106 MB/s):

| Component | CPU (percent of **one** core) | Memory |
|---|---|---|
| `u3v live` (everything except the Python viewer) | about 2-3 % | about 47 MB RSS |
| Python reader thread (frame cut + copy, no display) | about 7 % | - |

The driver reads USB on its own thread into a 48 x 1 MiB ring buffer and writes stdout on a
second thread, so a slow consumer never stalls the camera. Displaying a 1936x1216 frame with
`cv2.imshow` costs about 33 ms, so downscale for display if you need more than ~30 fps on screen.

Frame rate on the 1280x960 camera: 15 -> 15.0, 30 -> about 28-30, anything higher is clamped
by the camera to about 41 fps (reads back as `41.00`).

## Known limitations

- Only tested on the two cameras above. Other Alvium models/firmware may need different register addresses.
- One camera per process; multi-camera use and bandwidth scheduling are not implemented.
- Synchronous 1 MiB USB reads (one in flight). An asynchronous multi-URB engine is planned.
- No `/dev/video` device yet (the camera is not UVC).
- Exposure, gain and similar features are set with fixed register writes in `live`; there is no general feature API.

## Roadmap

- Asynchronous libusb engine (queued URBs, zero-copy) for the lowest CPU use
- `v4l2loopback` output so the camera looks like a normal webcam to OpenCV / ffmpeg / ROS
- Raspberry Pi validation and 3-camera operation
- Configurable ROI, exposure and gain from the command line

## Troubleshooting

See the "Camera quirks" section of [docs/PROTOCOL.md](docs/PROTOCOL.md). The short version:
truncated or torn frames mean a link/consumer problem, a silent stream means `./u3v devreset`,
and `claim ctrl failed` means another process (an old `u3v`, or Vimba) still holds the camera.

## License

No license has been chosen yet, which means all rights are reserved by default. Add a
`LICENSE` file before others reuse the code.
