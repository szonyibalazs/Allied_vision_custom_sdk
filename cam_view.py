#!/usr/bin/env python3
"""
Alvium (Allied Vision) U3V kamera élőkép-nézegető — SDK nélkül.

Használat:  python3 cam_view.py [fps]        (alap: 15)
Kilépés:    q vagy ESC az ablakban, ill. Ctrl+C.

Működés:
  - A ./u3v live bekapcsolja a kamera streamjét, és a nyers U3V bájtfolyamot
    (leader "U3VL" + payload + trailer "U3VT") a stdout-ra írja.
  - Egy OLVASÓ SZÁL folyamatosan üríti a csövet és kivágja a képkockákat.
    Ez kritikus: ha az olvasás akár rövid időre is megáll (pl. lassú imshow),
    a kamera belső puffere megtelik és a frame végét eldobja (trailer státusz
    0xA100) -> csonka, "szétesett" kép, színek elcsúszása.
  - A fő szál csak a legfrissebb képet jeleníti meg (cv2.imshow). Ha a
    megjelenítés lassabb a kameránál, a köztes képkockák kimaradnak, de a
    stream sosem akad meg.
  - Mono8 és RGB8 kamerát is automatikusan felismeri a leaderből.
  - Ez NEM cv2.VideoCapture: a kamera nem UVC, nincs /dev/video.
"""
import fcntl
import os
import subprocess
import sys
import threading
import time
import struct

import cv2
import numpy as np

BOOST_DARK = True          # nagyon sötét kép megjelenítési kiemelése
MONO8_PF = 0x01080001
RGB8_PF = 0x02180014
LEADER = 52
TRAILER = 32
F_SETPIPE_SZ = 1031


class FrameReader(threading.Thread):
    """Csőből U3V frame-eket szed; mindig a legfrissebbet tartja meg."""

    def __init__(self, fd):
        super().__init__(daemon=True)
        self.fd = fd
        self.lock = threading.Lock()
        self.latest = None        # (seq, ndarray)
        self.seq = 0
        self.truncated = 0        # csonka (eldobott) frame-ek száma
        self.eof = False

    def run(self):
        buf = bytearray()
        pos = 0
        while True:
            chunk = os.read(self.fd, 1 << 20)
            if not chunk:
                break
            buf += chunk
            while True:
                i = buf.find(b"U3VL", pos)
                if i < 0:
                    pos = max(pos, len(buf) - 3)   # a magic blokkhatáron átnyúlhat
                    break
                if len(buf) < i + LEADER:
                    pos = i
                    break
                pf, w, h = struct.unpack_from("<III", buf, i + 28)
                if pf not in (MONO8_PF, RGB8_PF) or not (16 <= w <= 8192 and 16 <= h <= 8192):
                    pos = i + 4                      # hamis leader a képadatban
                    continue
                bpp = 1 if pf == MONO8_PF else 3
                end = i + LEADER + w * h * bpp
                if len(buf) < end + TRAILER:
                    pos = i
                    break                            # a frame vége még úton van
                if buf[end:end + 4] != b"U3VT":
                    self.truncated += 1              # csonka frame: resync
                    pos = i + 4
                    continue
                img = np.frombuffer(buf, dtype=np.uint8, count=w * h * bpp,
                                    offset=i + LEADER).copy()
                img = img.reshape(h, w) if bpp == 1 else img.reshape(h, w, 3)
                with self.lock:
                    self.seq += 1
                    self.latest = (self.seq, img)
                pos = end + TRAILER
            if pos > (48 << 20):                     # a már feldolgozott rész eldobása
                del buf[:pos]
                pos = 0
        self.eof = True

    def get(self, last_seq):
        with self.lock:
            if self.latest is None or self.latest[0] == last_seq:
                return None
            return self.latest


def main():
    fps_arg = sys.argv[1] if len(sys.argv) > 1 else "15"
    # the driver binary: $U3V_BIN, else ./u3v next to this script
    driver = os.environ.get("U3V_BIN") or os.path.join(os.path.dirname(os.path.abspath(__file__)), "u3v")
    proc = subprocess.Popen([driver, "live", fps_arg],
                            stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                            bufsize=0)
    try:
        fcntl.fcntl(proc.stdout.fileno(), F_SETPIPE_SZ, 1 << 20)
    except OSError:
        pass

    reader = FrameReader(proc.stdout.fileno())
    reader.start()

    win = "Alvium U3V live"
    last_seq, shown, t0 = 0, 0, time.time()
    print("Élőkép indult. Kilépés: q / ESC az ablakban.")
    try:
        while not reader.eof or reader.latest is not None and reader.latest[0] != last_seq:
            item = reader.get(last_seq)
            if item is None:
                if cv2.waitKey(5) & 0xFF in (ord("q"), 27):
                    break
                if reader.eof:
                    break
                continue
            last_seq, img = item
            if img.ndim == 3:
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)   # U3V RGB -> OpenCV BGR
            if BOOST_DARK and img.max() < 40:
                img = cv2.convertScaleAbs(img, alpha=255.0 / max(1, int(img.max())))
            cv2.imshow(win, img)
            shown += 1
            if time.time() - t0 > 1.0:
                print(f"megjelenitve: {shown / (time.time() - t0):.1f} fps, beolvasott: {reader.seq}, csonka: {reader.truncated}", flush=True)
                cv2.setWindowTitle(win, f"Alvium U3V live — {shown / (time.time() - t0):.1f} fps"
                                        f" — csonka: {reader.truncated} — q = kilépés")
                shown, t0 = 0, time.time()
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
    except KeyboardInterrupt:
        pass
    finally:
        if proc.poll() is None:
            proc.terminate()          # a ./u3v SIGTERM-re is leáll; a kamerát lásd: ./u3v stop
        try:
            proc.wait(timeout=3)
        except Exception:
            proc.kill()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
