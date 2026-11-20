/*
 * u3v.c - SDK-free USB3 Vision driver for Allied Vision Alvium cameras (libusb only)
 *
 * Protocol layers implemented from scratch (no Vimba X, no GenICam runtime):
 *   - GenCP control: READMEM/WRITEMEM over the control bulk endpoints (EP1 OUT / EP1 IN)
 *   - USB3 Vision bootstrap: ABRM / SBRM / SIRM register discovery
 *   - Image stream: bulk EP5 IN carrying Leader / Payload / Trailer blocks
 *
 * Usage:  ./u3v <mode> [args]       (diagnostics go to stderr, image data to stdout in `live`)
 *   live [fps] [mono|rgb]   continuous raw U3V block stream on stdout (this is what cam_view.py reads)
 *   info                    identify the camera, print geometry / pixel format / frame rate
 *   grab out.raw [W H [fps]] [testpat]   grab a few frames into out.raw (+ out.raw.stream)
 *   stop                    stop acquisition + stream cleanly (resets the USB device first)
 *   devreset                send the camera's own DeviceReset (un-wedges a silent USB3 stream)
 *   unstick                 raw USB-level recovery when GenCP answers INVALID_PARAMETER
 *   snap / frame / streamon / rawdump / epdebug / sirmtest / sirmplan / linkinfo / drainstream
 *                           low level bring-up and debug helpers, see docs/PROTOCOL.md
 *
 * Environment (live mode):  U3V_NOAUTO=1  disable auto exposure/gain/white balance (RGB8 only)
 *                           U3V_AUTO=egw  subset of auto features: e=exposure g=gain w=white balance
 */
#include <stdio.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#define VID 0x1ab2
#define PID 0x0001

#define EP_CTRL_OUT 0x01
#define EP_CTRL_IN  0x81
#define EP_STREAM   0x85

#define CMD_READMEM  0x0800
#define CMD_WRITEMEM 0x0802
#define FLAG_ACK     (1 << 14)

/* known feature register addresses (from the camera's own GenICam XML) */
#define REG_ACQ_START      0x10220000
#define REG_ACQ_STOP       0x10220010
#define REG_PIXEL_FORMAT   0x10100000
#define REG_WIDTH          0x10120000
#define REG_HEIGHT         0x10130000
#define REG_WIDTH_MAX      0x10110000
#define REG_HEIGHT_MAX     0x10110010
#define REG_FRAMERATE_EN   0x102200e0
#define REG_FRAMERATE      0x10220100

/* ABRM offsets */
#define ABRM_SBRM_ADDRESS  0x01d8
#define SBRM_MAX_CMD       0x0014
#define SBRM_MAX_ACK       0x0018
#define SBRM_SIRM_ADDRESS  0x0020
/* SIRM offsets */
#define SIRM_CONTROL       0x0004
#define SIRM_REQ_PAYLOAD   0x0008
#define SIRM_REQ_LEADER    0x0010
#define SIRM_REQ_TRAILER   0x0014
#define SIRM_PAYLOAD_SIZE  0x001c
#define SIRM_MAX_LEADER    0x0018
#define SIRM_MAX_TRAILER   0x002c

static uint16_t pkt_id = 1;

static void put16le(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static void put32le(uint8_t *p, uint32_t v) { put16le(p, v & 0xffff); put16le(p + 2, v >> 16); }
static void put64le(uint8_t *p, uint64_t v) { put32le(p, v & 0xffffffff); put32le(p + 4, v >> 32); }
static uint16_t get16le(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint32_t get32le(const uint8_t *p) { return get16le(p) | ((uint32_t)get16le(p + 2) << 16); }
static uint64_t get64le(const uint8_t *p)
{
    return get32le(p) | ((uint64_t)get32le(p + 4) << 32);
}

typedef struct {
    libusb_device_handle *h;
    uint64_t sbrm, sirm;
    uint32_t max_cmd, max_ack;
} u3v_t;

static const char *statname(uint16_t s)
{
    switch (s) {
    case 0x0000: return "OK";
    case 0x8002: return "INVALID_PARAMETER";
    case 0x8003: return "INVALID_ADDRESS";
    case 0x8005: return "BAD_ALIGNMENT";
    case 0x8006: return "ACCESS_DENIED";
    case 0x8007: return "BUSY";
    case 0x8FFF: return "GENERIC_ERROR";
    default: return "?";
    }
}

/* ---------- control channel ---------- */

static int ctrl_cmd(u3v_t *u, const uint8_t *cmd, int clen,
                    uint8_t *rsp, int rmax, int *rrx)
{
    int sent, r, rx = 0;
    *rrx = 0;
    r = libusb_bulk_transfer(u->h, EP_CTRL_OUT, (uint8_t *)cmd, clen, &sent, 2000);
    if (r) return r;
    r = libusb_bulk_transfer(u->h, EP_CTRL_IN, rsp, rmax, &rx, 2000);
    *rrx = rx;
    return r;
}

/* one transaction; returns data length or negative on error; retries once on failure */
static int transact(u3v_t *u, const uint8_t *cmd, int clen,
                    uint8_t *rsp, int rmax)
{
    int rrx = 0;
    for (int attempt = 0; attempt < 3; attempt++) {
        int r = ctrl_cmd(u, cmd, clen, rsp, rmax, &rrx);
        if (r == LIBUSB_ERROR_TIMEOUT || r == LIBUSB_ERROR_OVERFLOW) {
            /* drain stale data and retry */
            uint8_t tmp[65536]; int rx, d = 0;
            for (int i = 0; i < 16; i++) {
                r = libusb_bulk_transfer(u->h, EP_CTRL_IN, tmp, sizeof(tmp), &rx, 100);
                if (r == LIBUSB_ERROR_TIMEOUT) break;
                d += rx;
            }
            if (d) fprintf(stderr, "drained %d stale bytes\n", d);
            continue;
        }
        if (r) return r;
        if (rrx < 12 || memcmp(rsp, "U3VC", 4) != 0) return -1;
        uint16_t status = get16le(rsp + 4);
        if (status != 0) {
            fprintf(stderr, "ctrl status 0x%04x (%s)\n", status, statname(status));
            return -2;
        }
        return rrx;
    }
    return -3;
}

int u3v_readmem(u3v_t *u, uint64_t addr, uint16_t len, uint8_t *out)
{
    uint8_t cmd[24], rsp[65536];
    uint32_t chunk_max = (u->max_ack ? u->max_ack - 12 - 2 : 498);
    if (chunk_max > 4000) chunk_max = 4000;

    int done = 0;
    while (done < len) {
        uint32_t chunk = len - done;
        if (chunk > chunk_max) chunk = chunk_max;
        put32le(cmd + 0, 0x43563355);
        put16le(cmd + 4, FLAG_ACK);
        put16le(cmd + 6, CMD_READMEM);
        put16le(cmd + 8, 12);
        put16le(cmd + 10, pkt_id++);
        put64le(cmd + 12, addr + done);
        put16le(cmd + 20, 0);
        put16le(cmd + 22, chunk);
        int r = transact(u, cmd, sizeof(cmd), rsp, sizeof(rsp));
        if (r < 12) return r < 0 ? r : -4;
        int n = r - 12;
        if ((uint32_t)n > chunk) n = chunk;
        memcpy(out + done, rsp + 12, n);
        done += n;
    }
    return len;
}

int u3v_writemem(u3v_t *u, uint64_t addr, uint16_t len, const uint8_t *data)
{
    uint8_t cmd[65536], rsp[65536];
    uint32_t chunk_max = (u->max_cmd ? u->max_cmd - 12 - 8 : 492);
    if (chunk_max > 60000) chunk_max = 60000;

    int done = 0;
    while (done < len) {
        uint32_t chunk = len - done;
        if (chunk > chunk_max) chunk = chunk_max;
        put32le(cmd + 0, 0x43563355);
        put16le(cmd + 4, FLAG_ACK);
        put16le(cmd + 6, CMD_WRITEMEM);
        put16le(cmd + 8, 8 + chunk);
        put16le(cmd + 10, pkt_id++);
        put64le(cmd + 12, addr + done);
        memcpy(cmd + 20, data + done, chunk);
        int r = transact(u, cmd, 20 + chunk, rsp, sizeof(rsp));
        if (r < 12) return r < 0 ? r : -4;
        done += chunk;
    }
    return len;
}

int u3v_read32(u3v_t *u, uint64_t addr, uint32_t *val)
{
    return u3v_readmem(u, addr, 4, (uint8_t *)val);
}

int u3v_write32(u3v_t *u, uint64_t addr, uint32_t val)
{
    uint8_t b[4];
    put32le(b, val);
    return u3v_writemem(u, addr, 4, b);
}

/* ---------- bootstrap ---------- */

static int drain_ctrl(libusb_device_handle *h)
{
    uint8_t tmp[65536]; int rx, drained = 0;
    for (int i = 0; i < 32; i++) {
        int r = libusb_bulk_transfer(h, EP_CTRL_IN, tmp, sizeof(tmp), &rx, 100);
        if (r == LIBUSB_ERROR_TIMEOUT) break;
        drained += rx;
    }
    return drained;
}

int u3v_open(u3v_t *u, int do_reset)
{
    memset(u, 0, sizeof(*u));
    int r = libusb_init(NULL);
    if (r) return r;

    u->h = libusb_open_device_with_vid_pid(NULL, VID, PID);
    if (!u->h) { fprintf(stderr, "camera not found\n"); return -1; }

    if (do_reset) {
        r = libusb_reset_device(u->h);
        usleep(2000000);
    }
    libusb_detach_kernel_driver(u->h, 0);
    if (libusb_claim_interface(u->h, 0)) { fprintf(stderr, "claim ctrl failed\n"); return -1; }

    int drained = drain_ctrl(u->h);
    if (drained) fprintf(stderr, "drained %d stale bytes\n", drained);

    uint8_t b8[8];
    /* bootstrap: SBRM address + limits */
    if (u3v_readmem(u, 0x01d8, 8, b8) < 0) return -1;
    u->sbrm = get64le(b8);
    u3v_readmem(u, u->sbrm + SBRM_MAX_CMD, 4, b8);
    u->max_cmd = get32le(b8);
    u3v_readmem(u, u->sbrm + SBRM_MAX_ACK, 4, b8);
    u->max_ack = get32le(b8);
    u3v_readmem(u, u->sbrm + SBRM_SIRM_ADDRESS, 8, b8);
    u->sirm = get64le(b8);
    fprintf(stderr, "SBRM=0x%llx SIRM=0x%llx max_cmd=%u max_ack=%u\n",
           (unsigned long long)u->sbrm, (unsigned long long)u->sirm,
           u->max_cmd, u->max_ack);
    return 0;
}

void u3v_close(u3v_t *u)
{
    if (u->h) libusb_close(u->h);
    libusb_exit(NULL);
}

/* ---------- main ---------- */

static uint32_t pfnc_name(uint32_t pf, char *out)
{
    /* PFNC values are ASCII four-cc in the low 32 bits, big-endian-ish */
    char s[5] = {0};
    s[0] = (pf >> 24) & 0xff;
    s[1] = (pf >> 16) & 0xff;
    s[2] = (pf >> 8) & 0xff;
    s[3] = pf & 0xff;
    strcpy(out, s);
    return pf;
}


/* float-regiszter iras: elobb 64 bites double (1936x1216 modell), ha nem megy, 32 bites float (1280x960 modell) */
static int write_float_reg(u3v_t *u, uint64_t addr64, uint64_t addr32, double v)
{
    uint8_t b8[8]; memcpy(b8, &v, 8);
    int r = u3v_writemem(u, addr64, 8, b8);
    if (r >= 0) return r;
    float f = (float)v; uint8_t b4[4]; memcpy(b4, &f, 4);
    return u3v_writemem(u, addr32, 4, b4);
}
static double read_float_reg(u3v_t *u, uint64_t addr64, uint64_t addr32, int *wide)
{
    uint8_t b[8]; double d = 0; float f = 0;
    if (u3v_readmem(u, addr64, 8, b) >= 0) { memcpy(&d, b, 8); if (d > 0 && d < 1e9) { *wide = 1; return d; } }
    if (u3v_readmem(u, addr32, 4, b) >= 0) { memcpy(&f, b, 4); *wide = 0; return f; }
    return 0;
}

static volatile sig_atomic_t g_stop = 0;
#define RING_N 48
#define RING_BLK (1 << 20)
static struct {
    uint8_t *blk[RING_N]; int len[RING_N];
    int head, tail, cnt, done, dead;
    pthread_mutex_t m; pthread_cond_t c;
} g_ring = { .m = PTHREAD_MUTEX_INITIALIZER, .c = PTHREAD_COND_INITIALIZER };

static void *ring_writer(void *arg)
{
    (void)arg;
    for (;;) {
        pthread_mutex_lock(&g_ring.m);
        while (g_ring.cnt == 0 && !g_ring.done) pthread_cond_wait(&g_ring.c, &g_ring.m);
        if (g_ring.cnt == 0) { pthread_mutex_unlock(&g_ring.m); break; }
        int t = g_ring.tail, n = g_ring.len[t];
        pthread_mutex_unlock(&g_ring.m);
        const uint8_t *p = g_ring.blk[t];
        while (n > 0) {
            /* poll: ha a fogyaszto nem olvas (teli pipe), leallitaskor se akadjunk be */
            struct pollfd pfd = { .fd = STDOUT_FILENO, .events = POLLOUT };
            int pr = poll(&pfd, 1, 200);
            if (g_stop) { g_ring.dead = 1; return NULL; }
            if (pr == 0) continue;
            if (pr < 0) { if (errno == EINTR) continue; g_ring.dead = 1; return NULL; }
            ssize_t w = write(STDOUT_FILENO, p, n);
            if (w < 0) { if (errno == EINTR || errno == EAGAIN) continue; g_ring.dead = 1; return NULL; }
            p += w; n -= (int)w;
        }
        pthread_mutex_lock(&g_ring.m);
        g_ring.tail = (g_ring.tail + 1) % RING_N;
        g_ring.cnt--;
        pthread_mutex_unlock(&g_ring.m);
    }
    return NULL;
}

static void on_sig(int sg) { (void)sg; g_stop = 1; }

int main(int argc, char **argv)
{
    u3v_t u;
    setvbuf(stdout, NULL, _IONBF, 0);
    int reset = 0;
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "reset")) reset = 1;

    int unstick_mode = argc > 1 && !strcmp(argv[1], "unstick");
    if (!unstick_mode && u3v_open(&u, reset) < 0) return 1;

    uint32_t v;
    if (!strcmp(argv[1], "sirmtest")) {
        /* SIRM write diagnostics: which accesses are accepted? */
        uint8_t s[0x30];
        fprintf(stderr, "== ABRM access privilege (0x0204):\n");
        u3v_read32(&u, 0x0204, &v);
        fprintf(stderr, "  0x%08x\n", v);
        fprintf(stderr, "== ABRM device configuration (0x01e0):\n");
        u3v_readmem(&u, 0x01e0, 8, s);
        fprintf(stderr, "  0x%016llx\n", (unsigned long long)get64le(s));

        fprintf(stderr, "== SIRM single reads:\n");
        for (int i = 0; i < 0x30; i += 4) {
            int r2 = u3v_read32(&u, u.sirm + i, &v);
            fprintf(stderr, "  +0x%02x: 0x%08x (r=%d)\n", i, v, r2);
        }

        fprintf(stderr, "== claim interface 1 first, then try writes\n");
        int rc = libusb_claim_interface(u.h, 1);
        fprintf(stderr, "claim if1: %s\n", libusb_error_name(rc));

        fprintf(stderr, "== try write32 REQ_PAYLOAD_LO (sirm+0x08) = 0x12c000\n");
        u3v_write32(&u, u.sirm + 0x08, 0x12c000);
        fprintf(stderr, "== try write32 REQ_PAYLOAD_HI (sirm+0x0c) = 0\n");
        u3v_write32(&u, u.sirm + 0x0c, 0);
        fprintf(stderr, "== try write32 REQ_LEADER (sirm+0x10) = 32\n");
        u3v_write32(&u, u.sirm + 0x10, 32);
        fprintf(stderr, "== try write32 REQ_TRAILER (sirm+0x14) = 24\n");
        u3v_write32(&u, u.sirm + 0x14, 24);

        fprintf(stderr, "== SIRM single reads after:\n");
        for (int i = 0; i < 0x30; i += 4) {
            u3v_read32(&u, u.sirm + i, &v);
            fprintf(stderr, "  +0x%02x: 0x%08x\n", i, v);
        }

        fprintf(stderr, "== try stream enable (sirm+0x04) = 1\n");
        u3v_write32(&u, u.sirm + 0x04, 1);
        u3v_read32(&u, u.sirm + 0x04, &v);
        fprintf(stderr, "  control = 0x%08x\n", v);
        return 0;
    }

    if (!strcmp(argv[1], "grab") && argc > 2) {
        int testpat = (argc > 3 && !strcmp(argv[3], "testpat"));
        fprintf(stderr, "[grab] claiming if1...\n");
        libusb_claim_interface(u.h, 1);
        fprintf(stderr, "[grab] acq stop...\n");
        u3v_write32(&u, REG_ACQ_STOP, 1);
        fprintf(stderr, "[grab] stream off...\n");
        { uint32_t c0; u3v_read32(&u, u.sirm + 0x04, &c0);
          if (c0 & 1) u3v_write32(&u, u.sirm + 0x04, 0); }
        { uint8_t tmp[65536]; int rx;
          for (int i = 0; i < 32; i++) {
              int r2 = libusb_bulk_transfer(u.h, EP_STREAM, tmp, sizeof(tmp), &rx, 100);
              if (r2 == LIBUSB_ERROR_TIMEOUT) break;
          } }
        /* optional fps override: grab out.raw W H fps */
        if (argc > 5 && argv[5][0] >= '0' && argv[5][0] <= '9') {
            float fps = atof(argv[5]);
            uint32_t fv;
            u3v_write32(&u, 0x102200e0, 1);          /* frame rate enable */
            *(float *)&fv = fps;
            u3v_write32(&u, REG_FRAMERATE, fv);
            uint32_t rv; u3v_read32(&u, REG_FRAMERATE, &rv);
            fprintf(stderr, "fps set to %.2f (readback %.2f)\n", fps, *(float *)&rv);
        }
        /* optional ROI override: grab out.raw WxH */
        if (argc > 3 && argv[3][0] != 't') {
            uint32_t nw = (uint32_t)strtoul(argv[3], NULL, 0);
            uint32_t nh = (uint32_t)strtoul(argv[4], NULL, 0);
            u3v_write32(&u, REG_WIDTH, nw);
            u3v_write32(&u, REG_HEIGHT, nh);
            uint32_t cw, ch;
            u3v_read32(&u, REG_WIDTH, &cw);
            u3v_read32(&u, REG_HEIGHT, &ch);
            fprintf(stderr, "ROI set: %ux%u (readback %ux%u)\n", nw, nh, cw, ch);
        }
        if (testpat) {
            u3v_write32(&u, 0x101c1000, 1);  /* TestPattern = GreyDiagonalRamp */
            uint32_t tp; u3v_read32(&u, 0x101c1000, &tp);
            fprintf(stderr, "testpattern = %u\n", tp);
        }
        /* configure SIRM chunk plan (host declares transfer layout) */
        {
            uint32_t si_info = 0;
            u3v_read32(&u, u.sirm + 0x00, &si_info);
            uint32_t alignment = 1u << ((si_info & 0xff000000) >> 24);
            uint32_t max_transfer = 1u << 20;
            uint32_t aligned_max = max_transfer / alignment * alignment;
            uint32_t w2, h2;
            u3v_read32(&u, REG_WIDTH, &w2);
            u3v_read32(&u, REG_HEIGHT, &h2);
            uint32_t pf2; u3v_read32(&u, REG_PIXEL_FORMAT, &pf2);
            uint32_t bpp = (pf2 == 0x01080001) ? 1 : 3;   /* Mono8 or RGB8 */
            uint32_t req_payload = w2 * h2 * bpp;
            uint32_t req_leader = 0x34, req_trailer = 0x20;
            uint32_t leader_sz = (req_leader + alignment - 1) / alignment * alignment;
            uint32_t trailer_sz = (req_trailer + alignment - 1) / alignment * alignment;
            uint32_t payload_sz = req_payload < aligned_max ? req_payload : aligned_max;
            uint32_t payload_cnt = req_payload / payload_sz;
            uint32_t transfer1 = ((req_payload % payload_sz) + alignment - 1) / alignment * alignment;
            fprintf(stderr, "SIRM plan: %ux%u bpp=%u align=%u payload=%u cnt=%u t1=%u\n",
                   w2, h2, bpp, alignment, payload_sz, payload_cnt, transfer1);
            u3v_write32(&u, u.sirm + 0x18, leader_sz);
            u3v_write32(&u, u.sirm + 0x2c, trailer_sz);
            u3v_write32(&u, u.sirm + 0x1c, payload_sz);
            u3v_write32(&u, u.sirm + 0x20, payload_cnt);
            u3v_write32(&u, u.sirm + 0x24, transfer1);
            u3v_write32(&u, u.sirm + 0x28, 0);
        }
        { int we = u3v_write32(&u, u.sirm + 0x04, 1);   /* stream enable */
          fprintf(stderr, "stream enable: %d\n", we); }
        { int wa = u3v_write32(&u, REG_ACQ_START, 1);
          fprintf(stderr, "acq start: %d\n", wa); }

        /* stream everything into a big buffer with per-read logging */
        int ntimeout = 0;
        uint8_t *buf = malloc(16 * 1024 * 1024);
        size_t total = 0;
        for (int i = 0; i < 400 && total < 4 * 1024 * 1024; i++) {
            int rx = 0;
            int r2 = libusb_bulk_transfer(u.h, EP_STREAM, buf + total, 1 << 20, &rx, 1000);
            if (r2 == LIBUSB_ERROR_TIMEOUT) { ntimeout++; if (ntimeout <= 3 || ntimeout % 20 == 0) fprintf(stderr, "read %d: timeout #%d\n", i, ntimeout); continue; }
            if (r2) { fprintf(stderr, "read %d: err %s\n", i, libusb_error_name(r2)); break; }
            fprintf(stderr, "read %d: %d bytes (total %zu)\n", i, rx, total + rx);
            total += rx;
        }

        /* locate leader/trailer */
        size_t lpos = 0, tpos = 0;
        for (size_t i = 0; i + 4 <= total; i++)
            if (!memcmp(buf + i, "U3VL", 4)) { lpos = i; break; }
        for (size_t i = lpos + 52; i + 4 <= total; i++)
            if (!memcmp(buf + i, "U3VT", 4)) { tpos = i; break; }
        fprintf(stderr, "leader at %zu, trailer at %zu, payload = %zd bytes\n",
               lpos, tpos, (ssize_t)(tpos - lpos - 52));
        fprintf(stderr, "leader:");
        for (int i = 0; i < 52; i++) fprintf(stderr, " %02x", buf[lpos + i]);
        fprintf(stderr, "\n");

        FILE *f = fopen(argv[2], "wb");
        fwrite(buf + lpos + 52, 1, tpos - lpos - 52, f);
        fclose(f);
        fprintf(stderr, "payload -> %s\n", argv[2]);
        { char nm[512]; snprintf(nm, sizeof(nm), "%s.stream", argv[2]);
          f = fopen(nm, "wb"); fwrite(buf, 1, total, f); fclose(f);
          fprintf(stderr, "full stream -> %s (%zu bytes)\n", nm, total); }

        u3v_write32(&u, REG_ACQ_STOP, 1);
        u3v_write32(&u, u.sirm + 0x04, 0);
        if (testpat) u3v_write32(&u, 0x101c1000, 0);  /* pattern off */
        free(buf);
        return 0;
    }

    if (!strcmp(argv[1], "frame") && argc > 2) {
        /* clean start */
        libusb_claim_interface(u.h, 1);
        u3v_write32(&u, REG_ACQ_STOP, 1);
        { uint32_t c0; u3v_read32(&u, u.sirm + 0x04, &c0);
          if (c0 & 1) u3v_write32(&u, u.sirm + 0x04, 0); }
        { uint8_t tmp[65536]; int rx;
          for (int i = 0; i < 32; i++) {
              int r2 = libusb_bulk_transfer(u.h, EP_STREAM, tmp, sizeof(tmp), &rx, 100);
              if (r2 == LIBUSB_ERROR_TIMEOUT) break;
          } }
        u3v_write32(&u, u.sirm + 0x04, 1);
        u3v_write32(&u, REG_ACQ_START, 1);

        /* leader: read 512, expect short packet at 52 */
        uint8_t leader[512];
        int rx = 0, r2 = libusb_bulk_transfer(u.h, EP_STREAM, leader, 512, &rx, 3000);
        fprintf(stderr, "leader: r=%s rx=%d\n", libusb_error_name(r2), rx);
        fprintf(stderr, "leader full:");
        for (int i = 0; i < rx; i++) fprintf(stderr, " %02x", leader[i]);
        fprintf(stderr, "\n");
        uint32_t l_w = get32le(leader + 32), l_h = get32le(leader + 36);
        fprintf(stderr, "leader w=%u h=%u (guess)\n", l_w, l_h);

        /* payload: read until we hit 'U3VT' magic */
        uint8_t *frame = malloc(8 * 1024 * 1024);
        size_t got = 0;
        int nchunks = 0;
        while (got < 8 * 1024 * 1024) {
            r2 = libusb_bulk_transfer(u.h, EP_STREAM, frame + got, 1 << 20, &rx, 3000);
            if (r2) { fprintf(stderr, "payload r=%s at %zu\n", libusb_error_name(r2), got); break; }
            nchunks++;
            if (rx >= 4 && !memcmp(frame + got, "U3VT", 4)) { fprintf(stderr, "trailer after %zu bytes (%d chunks)\n", got, nchunks); break; }
            got += rx;
        }
        fprintf(stderr, "payload total: %zu bytes\n", got - 32 >= 0 ? got : got);
        /* last chunk contained trailer; find payload end = got (trailer at end) */
        FILE *f = fopen(argv[2], "wb");
        fwrite(frame, 1, got, f);
        fclose(f);
        fprintf(stderr, "raw frame written to %s\n", argv[2]);

        u3v_write32(&u, REG_ACQ_STOP, 1);
        u3v_write32(&u, u.sirm + 0x04, 0);
        free(frame);
        return 0;
    }

    if (!strcmp(argv[1], "live")) {
        /* continuous raw U3V stream to STDOUT (diagnostics on stderr).
         * usage: u3v live [fps]  — frames = "U3VL"+payload+"U3VT" blocks, parsed by the reader */
        int fpswant = 15;
        for (int i = 2; i < argc; i++)
            if (argv[i][0] >= '0' && argv[i][0] <= '9') fpswant = atoi(argv[i]);
        libusb_claim_interface(u.h, 1);
        u3v_write32(&u, REG_ACQ_STOP, 1);
        { uint32_t c0; u3v_read32(&u, u.sirm + 0x04, &c0);
          if (c0 & 1) u3v_write32(&u, u.sirm + 0x04, 0); }
        { uint8_t tmp[65536]; int rx;
          for (int i = 0; i < 32; i++) {
              int r2 = libusb_bulk_transfer(u.h, EP_STREAM, tmp, sizeof(tmp), &rx, 100);
              if (r2 == LIBUSB_ERROR_TIMEOUT) break;
          } }
        { int want_mono = 0, want_rgb = 0;
          for (int i = 2; i < argc; i++) {
              if (!strcmp(argv[i], "mono")) want_mono = 1;
              if (!strcmp(argv[i], "rgb")) want_rgb = 1;
          }
          uint32_t pf; u3v_read32(&u, REG_PIXEL_FORMAT, &pf);
          if (want_mono && pf != 0x01080001) {
              fprintf(stderr, "[live] Mono8-ba valtok\n");
              u3v_write32(&u, REG_PIXEL_FORMAT, 0x01080001);
          } else if (want_rgb && pf != 0x02180014) {
              fprintf(stderr, "[live] RGB8-ba valtok\n");
              u3v_write32(&u, REG_PIXEL_FORMAT, 0x02180014);
          }
          u3v_read32(&u, REG_PIXEL_FORMAT, &pf);
          fprintf(stderr, "[live] pixelformat=0x%08x\n", pf);
          /* szines (RGB8) kamera: auto expozicio / auto gain / auto feheregyensuly folyamatosan.
           * Mono kamerat NEM erint (ott nincs BalanceWhiteAuto, es a mono ut valtozatlan marad). */
          if (pf == 0x02180014) {
              /* determinisztikus indulas: a kamera megjegyzi az elozo futas auto/expozicio allapotat
               * (pl. 109 ms expozicio -> max 9 fps), ezert eloszor minden auto OFF + 5 ms expozicio */
              const char *am = getenv("U3V_NOAUTO") ? "" : (getenv("U3V_AUTO") ? getenv("U3V_AUTO") : "egw");
              int r0 = u3v_write32(&u, 0x1b1c1000, 0);   /* ExposureAuto = Off */
              int r1 = u3v_write32(&u, 0x1b1c4000, 0);   /* GainAuto = Off */
              int r2 = u3v_write32(&u, 0x1b1ca000, 0);   /* BalanceWhiteAuto = Off */
              int r3 = write_float_reg(&u, 0x10240000, 0x10240000, 5000.0);   /* ExposureTime = 5000 us (double vagy float) */
              fprintf(stderr, "[live] auto OFF: %d %d %d, ExposureTime=5000us write=%d\n", r0, r1, r2, r3);
              if (strchr(am, 'e')) {
                  /* az auto expozicio plafonja a frame-periodus 50%-a (kulonben leesik az fps) */
                  double mx = 0.5e6 / (fpswant > 0 ? fpswant : 15);
                  int e4 = write_float_reg(&u, 0x1B1C3008, 0x1B1C3004, mx);
                  int wide = 0; double rb = read_float_reg(&u, 0x1B1C3008, 0x1B1C3004, &wide);
                  fprintf(stderr, "[live] ExposureAutoMax = %.0f us (write=%d, %s)\n", rb, e4, wide ? "double" : "float32");
                  fprintf(stderr, "[live] ExposureAuto=Continuous: %d\n", u3v_write32(&u, 0x1b1c1000, 2));
              }
              if (strchr(am, 'g')) fprintf(stderr, "[live] GainAuto=Continuous: %d\n", u3v_write32(&u, 0x1b1c4000, 2));
              if (strchr(am, 'w')) fprintf(stderr, "[live] BalanceWhiteAuto=Continuous: %d\n", u3v_write32(&u, 0x1b1ca000, 2));
          }
          /* NEM irjuk a throughput-limit regisztereket: 0-ra irva csonkolt frame-eket ad (0xA100) */
          { uint32_t lim0; u3v_read32(&u, 0x10040000, &lim0);
            fprintf(stderr, "[live] limit-mode=%u (nem modositjuk)\n", lim0); } }
        { uint32_t fv; float f = (float)fpswant; memcpy(&fv, &f, 4);
          u3v_write32(&u, 0x102200e0, 1);
          u3v_write32(&u, REG_FRAMERATE, fv);
          uint32_t rv; u3v_read32(&u, REG_FRAMERATE, &rv);
          fprintf(stderr, "[live] fps=%.2f\n", *(float *)&rv); }
        { uint32_t si_info; u3v_read32(&u, u.sirm + 0x00, &si_info);
          uint32_t alignment = 1u << ((si_info & 0xff000000) >> 24);
          uint8_t b8[8];
          u3v_readmem(&u, u.sirm + 0x08, 8, b8);
          uint32_t req_payload = (uint32_t)get64le(b8);
          uint32_t aligned_max = (1u << 20) / alignment * alignment;
          uint32_t payload_sz = req_payload < aligned_max ? req_payload : aligned_max;
          uint32_t payload_cnt = req_payload / payload_sz;
          uint32_t transfer1 = ((req_payload % payload_sz) + alignment - 1) / alignment * alignment;
          fprintf(stderr, "[live] plan: payload=%u cnt=%u t1=%u (req=%u align=%u)\n",
                  payload_sz, payload_cnt, transfer1, req_payload, alignment);
          u3v_write32(&u, u.sirm + 0x18, alignment);
          u3v_write32(&u, u.sirm + 0x2c, alignment);
          u3v_write32(&u, u.sirm + 0x1c, payload_sz);
          u3v_write32(&u, u.sirm + 0x20, payload_cnt);
          u3v_write32(&u, u.sirm + 0x24, transfer1);
          u3v_write32(&u, u.sirm + 0x28, 0);
        }
        { int we = u3v_write32(&u, u.sirm + 0x04, 1);
          fprintf(stderr, "[live] stream enable=%d\n", we); }
        { int wa = u3v_write32(&u, REG_ACQ_START, 1);
          fprintf(stderr, "[live] ACQ_START=%d\n", wa); }

        /* USB-olvasas (ez a szal) es stdout-iras (writer szal) szetvalasztva, gyurupufferen at:
         * az USB sosem var a fogyasztora, kulonben a kamera puffere megtelik es eldobja a frame-et */
        signal(SIGPIPE, SIG_IGN);
        { struct sigaction sa; memset(&sa, 0, sizeof(sa)); sa.sa_handler = on_sig;
          sigaction(SIGTERM, &sa, NULL); sigaction(SIGINT, &sa, NULL); }
        for (int k = 0; k < RING_N; k++) {
            g_ring.blk[k] = malloc(RING_BLK);
            if (!g_ring.blk[k]) { fprintf(stderr, "[live] nincs memoria\n"); return 1; }
        }
        uint8_t *scratch = malloc(RING_BLK);
        /* non-blocking stdout: teli pipe mellett a write ne ragadjon be leallitaskor */
        { int fl = fcntl(STDOUT_FILENO, F_GETFL); if (fl >= 0) fcntl(STDOUT_FILENO, F_SETFL, fl | O_NONBLOCK); }
        pthread_t wt;
        pthread_create(&wt, NULL, ring_writer, NULL);
        size_t total = 0, dropped = 0;
        int ntimeout = 0;
        while (!g_stop && !g_ring.dead) {
            pthread_mutex_lock(&g_ring.m);
            int full = (g_ring.cnt == RING_N);
            pthread_mutex_unlock(&g_ring.m);
            uint8_t *dst = full ? scratch : g_ring.blk[g_ring.head];
            int rx = 0;
            int r2 = libusb_bulk_transfer(u.h, EP_STREAM, dst, RING_BLK, &rx, 500);
            if (r2 == LIBUSB_ERROR_TIMEOUT) {
                if (++ntimeout > 20) { fprintf(stderr, "[live] stream timeout, kilepek\n"); break; }
                continue;
            }
            ntimeout = 0;
            if (r2) { fprintf(stderr, "[live] %s\n", libusb_error_name(r2)); break; }
            total += rx;
            if (full) { dropped += rx; continue; }   /* a fogyaszto lemaradt: blokkot dobunk, az USB nem all meg */
            pthread_mutex_lock(&g_ring.m);
            g_ring.len[g_ring.head] = rx;
            g_ring.head = (g_ring.head + 1) % RING_N;
            g_ring.cnt++;
            pthread_cond_signal(&g_ring.c);
            pthread_mutex_unlock(&g_ring.m);
        }
        pthread_mutex_lock(&g_ring.m);
        g_ring.done = 1;
        pthread_cond_signal(&g_ring.c);
        pthread_mutex_unlock(&g_ring.m);
        pthread_join(wt, NULL);
        u3v_write32(&u, REG_ACQ_STOP, 1);
        u3v_write32(&u, u.sirm + 0x04, 0);
        fprintf(stderr, "[live] stopped, %.1f MB streamed, %.1f MB eldobva (lassu fogyaszto)\n",
                total / 1048576.0, dropped / 1048576.0);
        return 0;
    }

    if (!strcmp(argv[1], "streamon")) {
        /* minimal stream bring-up, order configurable: streamon [startfirst] */
        int startfirst = argc > 2 && !strcmp(argv[2], "startfirst");
        libusb_claim_interface(u.h, 1);
        /* stop acquisition first: unlocks transport-layer params */
        fprintf(stderr, "ACQ_STOP -> %d\n", u3v_write32(&u, REG_ACQ_STOP, 1));
        /* force Mono8 + fps limit + throughput limit off (USB2-working recipe) */
        int want_mono = 0;
        for (int i = 2; i < argc; i++) if (!strcmp(argv[i], "mono")) want_mono = 1;
        if (want_mono) {
            u3v_write32(&u, REG_PIXEL_FORMAT, 0x01080001);
            uint32_t pf3; u3v_read32(&u, REG_PIXEL_FORMAT, &pf3);
            fprintf(stderr, "pixelformat -> 0x%08x\n", pf3);
            u3v_write32(&u, 0x10040000, 0);       /* throughput limit off */
            u3v_write32(&u, 0x102200e0, 1);       /* frame rate enable */
            float fps2 = 2.0f; uint32_t fv2; memcpy(&fv2, &fps2, 4);
            u3v_write32(&u, REG_FRAMERATE, fv2);
            uint32_t rv2; u3v_read32(&u, REG_FRAMERATE, &rv2);
            fprintf(stderr, "fps -> %.2f\n", *(float *)&rv2);
        }
        /* SIRM plan for CURRENT geometry/format (after any mono switch) */
        {
            uint32_t si_info; u3v_read32(&u, u.sirm + 0x00, &si_info);
            uint32_t alignment = 1u << ((si_info & 0xff000000) >> 24);
            uint8_t b8[8];
            u3v_readmem(&u, u.sirm + 0x08, 8, b8);   /* device-reported req payload */
            uint32_t req_payload = (uint32_t)get64le(b8);
            uint32_t w2, h2;
            u3v_read32(&u, REG_WIDTH, &w2);
            u3v_read32(&u, REG_HEIGHT, &h2);
            uint32_t aligned_max = (1u << 20) / alignment * alignment;
            uint32_t payload_sz = req_payload < aligned_max ? req_payload : aligned_max;
            uint32_t payload_cnt = req_payload / payload_sz;
            uint32_t transfer1 = ((req_payload % payload_sz) + alignment - 1) / alignment * alignment;
            uint32_t leader_sz = alignment, trailer_sz = alignment;
            fprintf(stderr, "plan: %ux%u payload=%u cnt=%u t1=%u leader=%u trailer=%u (req=%u)\n",
                   w2, h2, payload_sz, payload_cnt, transfer1, leader_sz, trailer_sz, req_payload);
            u3v_write32(&u, u.sirm + 0x18, leader_sz);
            u3v_write32(&u, u.sirm + 0x2c, trailer_sz);
            u3v_write32(&u, u.sirm + 0x1c, payload_sz);
            u3v_write32(&u, u.sirm + 0x20, payload_cnt);
            u3v_write32(&u, u.sirm + 0x24, transfer1);
            u3v_write32(&u, u.sirm + 0x28, 0);
        }
        /* force a clean 0->1 edge on the stream enable bit */
        { uint32_t c2; u3v_read32(&u, u.sirm + 0x04, &c2);
          if (c2 & 1) fprintf(stderr, "pre-disable -> %d\n", u3v_write32(&u, u.sirm + 0x04, 0)); }
        if (startfirst) {
            fprintf(stderr, "ACQ_START -> %d\n", u3v_write32(&u, REG_ACQ_START, 1));
            fprintf(stderr, "enable -> %d\n", u3v_write32(&u, u.sirm + 0x04, 1));
        } else {
            fprintf(stderr, "ACQ_START -> %d\n", u3v_write32(&u, REG_ACQ_START, 1));
            fprintf(stderr, "enable -> %d\n", u3v_write32(&u, u.sirm + 0x04, 1));
        }
        for (int i = 0; i < 12; i++) {
            uint8_t tmp[8192]; int rx = 0;
            int r2 = libusb_bulk_transfer(u.h, EP_STREAM, tmp, sizeof(tmp), &rx, 2000);
            if (r2) { fprintf(stderr, "read: %s\n", libusb_error_name(r2)); continue; }
            fprintf(stderr, "read: %d bytes |", rx);
            for (int j = 0; j < 8 && j < rx; j++) fprintf(stderr, " %02x", tmp[j]);
            fprintf(stderr, "\n");
        }
        return 0;
    }

    if (!strcmp(argv[1], "unstick")) {
        /* nyers kibogozas bootstrap nelkul: full USB + EP + kontroll takaritas */
        fprintf(stderr, "[unstick] start\n");
        libusb_init(NULL);
        libusb_device_handle *h = libusb_open_device_with_vid_pid(NULL, VID, PID);
        if (!h) { printf("nincs kamera\n"); return 1; }
        fprintf(stderr, "[unstick] device opened\n");
        libusb_detach_kernel_driver(h, 0);
        libusb_detach_kernel_driver(h, 1);
        for (int i = 0; i < 2; i++) libusb_claim_interface(h, i);
        libusb_clear_halt(h, 0x01); libusb_clear_halt(h, 0x81);
        libusb_clear_halt(h, EP_STREAM | 0x80);
        uint8_t tmp[65536]; int rx, total = 0;
        for (int i = 0; i < 3000; i++) {
            int r2 = libusb_bulk_transfer(h, EP_STREAM | 0x80, tmp, sizeof(tmp), &rx, 200);
            if (r2 == LIBUSB_ERROR_TIMEOUT) break;
            if (r2) break;
            total += rx;
        }
        printf("stream drained: %d bytes\n", total);
        { int d = 0;
          for (int i = 0; i < 32; i++) {
              int r2 = libusb_bulk_transfer(h, 0x81, tmp, sizeof(tmp), &rx, 100);
              if (r2 == LIBUSB_ERROR_TIMEOUT) break;
              d += rx;
          }
          printf("ctrl drained: %d bytes\n", d); }
        printf("USB reset...\n");
        { int rr = libusb_reset_device(h);
          printf("reset result: %d (%s)\n", rr, libusb_error_name(rr)); }
        sleep(3);
        /* parser-flush: ha a parancsfeldolgozo egy félbeszakadt iras testere var,
         * nullakat adunk neki (rossz magicu csomagot egyebkent eldobja a spec szerint) */
        for (int round = 0; round < 6; round++) {
            uint8_t z[512];
            memset(z, 0, sizeof(z));
            int sent;
            libusb_bulk_transfer(h, 0x01, z, 64, &sent, 500);
            usleep(50000);
            libusb_bulk_transfer(h, 0x01, z, 256, &sent, 500);
            usleep(50000);
            /* nyers 8 bajtos olvasas 0x01d8-rol (SBRM pointer) */
            {
                uint8_t cmd[24]; static uint8_t rsp[512]; uint16_t pid = 1;
                put32le(cmd + 0, 0x43563355); put16le(cmd + 4, 1 << 14);
                put16le(cmd + 6, 0x0800); put16le(cmd + 8, 12); put16le(cmd + 10, pid++);
                put64le(cmd + 12, 0x01d8); put16le(cmd + 20, 0); put16le(cmd + 22, 8);
                int r2 = libusb_bulk_transfer(h, 0x81, rsp, 512, &rx, 3000);
                if (!r2) {
                    uint16_t st = get16le(rsp + 4);
                    printf("round %d: read ack status=0x%04x rx=%d sbrm=0x%llx\n",
                           round, st, rx, (unsigned long long)get64le(rsp + 12));
                    if (st == 0) { printf("PARSER MEGVAN!\n"); break; }
                } else {
                    printf("round %d: rsp err %s\n", round, libusb_error_name(r2));
                }
            }
        }
        libusb_close(h);
        libusb_exit(NULL);
        return 0;
    }

    if (!strcmp(argv[1], "drainstream")) {
        /* stream EP kimeritese: ha a kamera elakadt streamel, ez oldja ki a kontrollt */
        u3v_t u;
        if (u3v_open(&u, 1)) return 1;
        libusb_claim_interface(u.h, 1);
        libusb_clear_halt(u.h, EP_STREAM | 0x80);
        uint8_t tmp[65536]; int rx, total = 0, loops = 0;
        for (;;) {
            int r2 = libusb_bulk_transfer(u.h, EP_STREAM | 0x80, tmp, sizeof(tmp), &rx, 500);
            if (r2 == LIBUSB_ERROR_TIMEOUT) break;
            if (r2) { printf("err %s\n", libusb_error_name(r2)); break; }
            total += rx; loops++;
            if (loops > 4000) break;
        }
        printf("stream EP drained: %d bytes (%d reads)\n", total, loops);
        sleep(1);
        printf("probe read: %d\n", u3v_read32(&u, 0x0004, (uint32_t[1]){0}));
        u3v_close(&u);
        return 0;
    }

    if (!strcmp(argv[1], "stop")) {
        /* tiszta leallitas: acq stop + stream off (kamera nem streamel tovabb) */
        u3v_t u;
        if (u3v_open(&u, 1)) return 1;
        libusb_claim_interface(u.h, 1);
        printf("ACQ_STOP -> %d\n", u3v_write32(&u, REG_ACQ_STOP, 1));
        printf("stream off -> %d\n", u3v_write32(&u, u.sirm + 0x04, 0));
        u3v_close(&u);
        printf("kamera leallitva\n");
        return 0;
    }

    if (!strcmp(argv[1], "devreset")) {
        u3v_t u;
        if (u3v_open(&u, 1)) return 1;
        uint32_t x; u3v_read32(&u, REG_ACQ_STOP, &x);
        fprintf(stderr, "ACQ_STOP -> %d\n", u3v_write32(&u, REG_ACQ_STOP, 1));
        fprintf(stderr, "stream off -> %d\n", u3v_write32(&u, u.sirm + 0x04, 0));
        fprintf(stderr, "DeviceReset -> %d (device re-enumerates!)\n", u3v_write32(&u, 0x100A0000, 1));
        u3v_close(&u);
        return 0;
    }

    if (!strcmp(argv[1], "epdebug")) {
        libusb_claim_interface(u.h, 1);
        uint32_t c1; u3v_read32(&u, u.sirm + 0x04, &c1);
        fprintf(stderr, "SIRM control = 0x%08x\n", c1);
        sleep(2);
        u3v_read32(&u, u.sirm + 0x04, &c1);
        fprintf(stderr, "SIRM control after 2s = 0x%08x\n", c1);
        uint32_t lim; u3v_read32(&u, 0x10040000, &lim);
        fprintf(stderr, "ThroughputLimitMode = %u\n", lim);
        { uint32_t tm, ts, em, ea, fpen, frv;
          u3v_read32(&u, 0x10300050, &tm);   /* TriggerMode */
          u3v_read32(&u, 0x10301000, &ts);   /* TriggerSource */
          u3v_read32(&u, 0x10230000, &em);   /* ExposureMode */
          u3v_read32(&u, 0x1b1c1000, &ea);   /* ExposureAuto */
          u3v_read32(&u, 0x102200e0, &fpen); /* FrameRateEnable */
          u3v_read32(&u, REG_FRAMERATE, &frv);
          fprintf(stderr, "TriggerMode=%u TriggerSource=%u ExposureMode=%u ExposureAuto=%u FrEnable=%u fps=%.2f\n",
                 tm, ts, em, ea, fpen, *(float *)&frv); }
        { uint8_t b8[8]; uint32_t hbv;
          u3v_readmem(&u, 0x10010388, 4, b8);
          memcpy(&hbv, b8, 4);
          fprintf(stderr, "DeviceLinkHeartbeatTimeout = %f us\n", *(float *)&hbv);
          u3v_read32(&u, 0x01e8, &hbv);
          fprintf(stderr, "ABRM heartbeat timeout = %u ms\n", hbv); }
        { uint32_t fpen; u3v_read32(&u, 0x102200e0, &fpen);
          if (fpen) { u3v_write32(&u, 0x102200e0, 0); fprintf(stderr, "FrEnable -> 0\n"); } }
        { uint32_t a; u3v_read32(&u, REG_ACQ_STOP, &a);
          fprintf(stderr, "(acq stop reg read=0x%x)\n", a); }
        const char *st[7] = {"TriggerWait","AcqActive","AcqTransfer","FrameTriggerWait","FrameActive","ExposureActive","ReadoutActive"};
        for (int rep = 0; rep < 8; rep++) {
            fprintf(stderr, "  poll %d:", rep);
            for (int i = 0; i < 7; i++) {
                uint32_t sv; u3v_read32(&u, 0x10220300 + i * 4, &sv);
                fprintf(stderr, " %s=%u", st[i], sv & 1);
            }
            fprintf(stderr, "\n");
            usleep(200000);
        }
        fprintf(stderr, "SIRM dump:\n");
        for (int i = 0; i < 0x30; i += 4) {
            uint32_t x; u3v_read32(&u, u.sirm + i, &x);
            fprintf(stderr, "  +0x%02x: 0x%08x\n", i, x);
        }
        fprintf(stderr, "clear_halt: %s\n", libusb_error_name(libusb_clear_halt(u.h, EP_STREAM)));
        { uint32_t e1, e2, g1, g2, fr;
          u3v_read32(&u, 0x10240000, &e1);  /* exposure time */
          u3v_read32(&u, 0x10410000, &g1);  /* gain */
          u3v_read32(&u, 0x102200e0, &fr);
          fprintf(stderr, "exposure=%08x gain=%08x frEnable=%u\n", e1, g1, fr);
          sleep(4);
          u3v_read32(&u, 0x10240000, &e2);
          u3v_read32(&u, 0x10410000, &g2);
          fprintf(stderr, "after 4s:  exposure=%08x gain=%08x\n", e2, g2);
          float fe1, fe2; memcpy(&fe1, &e1, 4); memcpy(&fe2, &e2, 4);
          fprintf(stderr, "exposure float: %.1f -> %.1f us\n", fe1, fe2); }
        uint8_t tmp[4096]; int rx = 0;
        int r2 = libusb_bulk_transfer(u.h, EP_STREAM, tmp, sizeof(tmp), &rx, 3000);
        fprintf(stderr, "raw read: %s rx=%d\n", libusb_error_name(r2), rx);
        if (rx > 0) { fprintf(stderr, "first bytes:"); for (int i = 0; i < 16 && i < rx; i++) fprintf(stderr, " %02x", tmp[i]); fprintf(stderr, "\n"); }
        return 0;
    }

    if (!strcmp(argv[1], "rawdump")) {
        /* enable stream + acquisition, then record raw EP bulk data for N ms */
        libusb_claim_interface(u.h, 1);
        u3v_write32(&u, REG_ACQ_STOP, 1);
        u3v_write32(&u, u.sirm + 0x04, 0);
        { uint8_t tmp[65536]; int rx;
          for (int i = 0; i < 32; i++) {
              int r2 = libusb_bulk_transfer(u.h, EP_STREAM, tmp, sizeof(tmp), &rx, 100);
              if (r2 == LIBUSB_ERROR_TIMEOUT) break;
          } }
        u3v_write32(&u, u.sirm + 0x04, 1);
        u3v_write32(&u, REG_ACQ_START, 1);

        uint8_t *buf = malloc(4 * 1024 * 1024);
        int total = 0;
        for (int i = 0; i < 40; i++) {
            int rx = 0;
            int r2 = libusb_bulk_transfer(u.h, EP_STREAM, buf + total, 4 * 1024 * 1024 - total, &rx, 500);
            if (r2 == LIBUSB_ERROR_TIMEOUT) { fprintf(stderr, "[%d] timeout\n", i); continue; }
            if (r2) { fprintf(stderr, "[%d] err %s\n", i, libusb_error_name(r2)); break; }
            fprintf(stderr, "[%d] +%d bytes, first 16: ", i, rx);
            for (int j = 0; j < 16 && j < rx; j++) fprintf(stderr, "%02x ", buf[total + j]);
            fprintf(stderr, "\n");
            total += rx;
        }
        fprintf(stderr, "total %d bytes\n", total);
        u3v_write32(&u, REG_ACQ_STOP, 1);
        u3v_write32(&u, u.sirm + 0x04, 0);
        return 0;
    }

    if (!strcmp(argv[1], "sirmplan") && argc >= 7) {
        /* sirmplan payload count transfer1 leader trailer */
        uint32_t payload = (uint32_t)strtoul(argv[2], NULL, 0);
        uint32_t count = (uint32_t)strtoul(argv[3], NULL, 0);
        uint32_t t1 = (uint32_t)strtoul(argv[4], NULL, 0);
        uint32_t ldr = (uint32_t)strtoul(argv[5], NULL, 0);
        uint32_t trl = (uint32_t)strtoul(argv[6], NULL, 0);
        libusb_claim_interface(u.h, 1);
        fprintf(stderr, "plan: payload=%u count=%u t1=%u leader=%u trailer=%u\n",
               payload, count, t1, ldr, trl);
        u3v_write32(&u, u.sirm + 0x18, ldr);
        u3v_write32(&u, u.sirm + 0x2c, trl);
        u3v_write32(&u, u.sirm + 0x1c, payload);
        u3v_write32(&u, u.sirm + 0x20, count);
        u3v_write32(&u, u.sirm + 0x24, t1);
        u3v_write32(&u, u.sirm + 0x28, 0);
        u3v_write32(&u, u.sirm + 0x04, 1);
        u3v_read32(&u, u.sirm + 0x04, &v);
        fprintf(stderr, "control = 0x%08x\n", v);
        return 0;
    }

    if (!strcmp(argv[1], "linkinfo")) {
        uint8_t b8[8];
        u3v_readmem(&u, 0x10050000, 8, b8);
        uint64_t speed = get64le(b8);
        fprintf(stderr, "DeviceLinkSpeed = %llu bps (%.1f MB/s)\n",
               (unsigned long long)speed, speed / 8.0 / 1e6);
        u3v_read32(&u, 0x10040010, &v);
        fprintf(stderr, "DeviceLinkThroughputLimit = %u\n", v);
        u3v_read32(&u, 0x10040000, &v);
        fprintf(stderr, "DeviceLinkThroughputLimitMode(reg+0) = %u\n", v);
        /* optional: disable throughput limit */
        if (argc > 2 && !strcmp(argv[2], "nolimit")) {
            u3v_write32(&u, 0x10040000, 0);
            u3v_read32(&u, 0x10040000, &v);
            fprintf(stderr, "ThroughputLimitMode now = %u\n", v);
        }
        u3v_read32(&u, REG_FRAMERATE, &v);
        fprintf(stderr, "AcquisitionFrameRate float = %f\n", *(float *)&v);
        u3v_read32(&u, 0x102200e0, &v);
        fprintf(stderr, "FrameRateEnable = %u\n", v);
        return 0;
    }

    if (argc < 2 || !strcmp(argv[1], "info")) {
        u3v_read32(&u, REG_PIXEL_FORMAT, &v);
        char pfn[8]; pfnc_name(v, pfn);
        fprintf(stderr, "pixel format: 0x%08x ('%s')\n", v, pfn);
        u3v_read32(&u, REG_WIDTH_MAX, &v);  fprintf(stderr, "width max:    %u\n", v);
        u3v_read32(&u, REG_HEIGHT_MAX, &v); fprintf(stderr, "height max:   %u\n", v);
        u3v_read32(&u, REG_WIDTH, &v);      fprintf(stderr, "width:        %u\n", v);
        u3v_read32(&u, REG_HEIGHT, &v);     fprintf(stderr, "height:       %u\n", v);
        u3v_read32(&u, REG_FRAMERATE, &v);  fprintf(stderr, "framerate:    %u (x1000?)\n", v);
        u3v_read32(&u, u.sirm + SIRM_PAYLOAD_SIZE, &v);
        fprintf(stderr, "SIRM payload: %u\n", v);
    } else if (!strcmp(argv[1], "snap")) {
        /* clean start: stop acquisition + stream, drain stream endpoint */
        libusb_claim_interface(u.h, 1);
        u3v_write32(&u, REG_ACQ_STOP, 1);
        uint32_t c0;
        u3v_read32(&u, u.sirm + 0x04, &c0);
        if (c0 & 1) u3v_write32(&u, u.sirm + 0x04, 0);
        { uint8_t tmp[65536]; int rx;
          for (int i = 0; i < 16; i++) {
              int r2 = libusb_bulk_transfer(u.h, EP_STREAM, tmp, sizeof(tmp), &rx, 100);
              if (r2 == LIBUSB_ERROR_TIMEOUT) break;
          } }

        /* read current geometry */
        uint32_t w, h, pf, sirm_payload, max_leader, max_trailer;
        u3v_read32(&u, REG_WIDTH, &w);
        u3v_read32(&u, REG_HEIGHT, &h);
        u3v_read32(&u, REG_PIXEL_FORMAT, &pf);
        u3v_read32(&u, u.sirm + SIRM_PAYLOAD_SIZE, &sirm_payload);
        u3v_read32(&u, u.sirm + SIRM_MAX_LEADER, &max_leader);
        u3v_read32(&u, u.sirm + SIRM_MAX_TRAILER, &max_trailer);
        char pfn[8]; pfnc_name(pf, pfn);
        fprintf(stderr, "grab %ux%u fmt='%s' payload=%u leader_max=%u trailer_max=%u\n",
               w, h, pfn, sirm_payload, max_leader, max_trailer);

        if (strcmp(pfn, "Mono8") != 0) {
            /* switch to Mono8 (PFNC 0x01080001) for the test */
            fprintf(stderr, "switching to Mono8\n");
            u3v_write32(&u, REG_PIXEL_FORMAT, 0x01080001);
            u3v_read32(&u, REG_PIXEL_FORMAT, &pf);
            pfnc_name(pf, pfn);
            fprintf(stderr, "pixel format now: 0x%08x ('%s')\n", pf, pfn);
        }

        /* configure SIRM per USB3 Vision spec: host declares the chunk plan.
         * alignment from SI_INFO bits [31:24] (exponent), max transfer 1MB. */
        int r;
        uint32_t si_info = 0;
        u3v_read32(&u, u.sirm + 0x00, &si_info);
        uint32_t alignment = 1u << ((si_info & 0xff000000) >> 24);
        uint32_t max_transfer = 1u << 20;   /* 1 MB */
        uint32_t aligned_max = max_transfer / alignment * alignment;
        uint32_t req_payload = w * h;       /* Mono8 bytes */
        uint32_t req_leader = 0x34, req_trailer = 0x20;

        uint32_t leader_sz = (req_leader < 1) ? aligned_max : ((req_leader + alignment - 1) / alignment * alignment);
        uint32_t trailer_sz = (req_trailer < 1) ? aligned_max : ((req_trailer + alignment - 1) / alignment * alignment);
        uint32_t payload_sz = req_payload < aligned_max ? req_payload : aligned_max;
        uint32_t payload_cnt = req_payload / payload_sz;
        uint32_t transfer1 = ((req_payload % payload_sz) + alignment - 1) / alignment * alignment;
        uint32_t transfer2 = 0;
        fprintf(stderr, "SIRM plan: align=%u payload=%u count=%u transfer1=%u leader=%u trailer=%u\n",
               alignment, payload_sz, payload_cnt, transfer1, leader_sz, trailer_sz);

        uint32_t ctrl;
        u3v_read32(&u, u.sirm + 0x04, &ctrl);
        if (ctrl & 1) { /* ensure stream disabled while configuring */
            u3v_write32(&u, u.sirm + 0x04, 0);
        }
        fprintf(stderr, "[grab] SIRM plan writes...\n");
        u3v_write32(&u, u.sirm + 0x18, leader_sz);    /* MAX_LEADER_SIZE */
        u3v_write32(&u, u.sirm + 0x2c, trailer_sz);   /* MAX_TRAILER_SIZE */
        u3v_write32(&u, u.sirm + 0x1c, payload_sz);   /* PAYLOAD_SIZE */
        u3v_write32(&u, u.sirm + 0x20, payload_cnt);  /* PAYLOAD_COUNT */
        u3v_write32(&u, u.sirm + 0x24, transfer1);    /* TRANSFER1_SIZE */
        u3v_write32(&u, u.sirm + 0x28, transfer2);    /* TRANSFER2_SIZE */

        r = libusb_claim_interface(u.h, 1);
        fprintf(stderr, "claim stream if1: %s\n", libusb_error_name(r));

        u3v_read32(&u, u.sirm + 0x04, &ctrl);
        if (ctrl & 1) u3v_write32(&u, u.sirm + 0x04, 0);  /* disable first */
        r = u3v_write32(&u, u.sirm + 0x04, 1);        /* stream enable */
        fprintf(stderr, "stream enable: %d\n", r);

        /* read back stream-relevant SIRM regs */
        {
            uint32_t x;
            fprintf(stderr, "SIRM after config:\n");
            for (int i = 0; i < 0x30; i += 4) {
                u3v_read32(&u, u.sirm + i, &x);
                fprintf(stderr, "  +0x%02x: 0x%08x\n", i, x);
            }
        }

        r = u3v_write32(&u, REG_ACQ_START, 1);           /* acquisition start */
        fprintf(stderr, "acq start: %d\n", r);

        /* stream wire layout: leader(leader_sz) | payload chunks | trailer(trailer_sz) */
        uint8_t *leader = malloc(leader_sz);
        uint8_t *frame = malloc(req_payload);
        uint8_t *trailer = malloc(trailer_sz);
        int rx = 0;

        r = libusb_bulk_transfer(u.h, EP_STREAM, leader, leader_sz, &rx, 3000);
        fprintf(stderr, "leader: r=%s rx=%d\n", libusb_error_name(r), rx);
        if (r == 0 && rx >= 16) {
            uint16_t cid = get16le(leader);
            uint32_t dtype = get32le(leader + 4);
            uint64_t psize = get64le(leader + 8);
            fprintf(stderr, "leader: cid=%u dtype=%u payload_size=%llu\n",
                   cid, dtype, (unsigned long long)psize);
        }

        uint64_t got = 0;
        while (got < req_payload) {
            r = libusb_bulk_transfer(u.h, EP_STREAM, frame + got,
                                     (int)(req_payload - got), &rx, 3000);
            if (r) { fprintf(stderr, "payload: %s at %llu\n", libusb_error_name(r),
                            (unsigned long long)got); break; }
            got += rx;
        }
        fprintf(stderr, "payload: %llu bytes\n", (unsigned long long)got);

        r = libusb_bulk_transfer(u.h, EP_STREAM, trailer, trailer_sz, &rx, 3000);
        fprintf(stderr, "trailer: r=%s rx=%d\n", libusb_error_name(r), rx);
        if (r == 0 && rx >= 20) {
            uint64_t xfer = get64le(trailer + 4);
            uint32_t err = get32le(trailer + 12);
            fprintf(stderr, "trailer: transferred=%llu errors=0x%08x\n",
                   (unsigned long long)xfer, err);
            fprintf(stderr, "trailer raw:");
            for (int i = 0; i < rx; i++) fprintf(stderr, " %02x", trailer[i]);
            fprintf(stderr, "\n");
        }

        u3v_write32(&u, REG_ACQ_STOP, 1);
        u3v_write32(&u, u.sirm + 0x04, 0);

        /* write PGM (Mono8) */
        if (got == (uint64_t)w * h && argc > 2) {
            FILE *f = fopen(argv[2], "wb");
            fprintf(f, "P5\n%u %u\n255\n", w, h);
            fwrite(frame, 1, w * h, f);
            fclose(f);
            fprintf(stderr, "wrote %s\n", argv[2]);
        }

        free(leader); free(frame); free(trailer);
    }

    u3v_close(&u);
    return 0;
}
