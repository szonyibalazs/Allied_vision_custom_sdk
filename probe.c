/*
 * Minimal USB3 Vision / GenCP probe for Allied Vision cameras (no SDK).
 * GenCP layout (little-endian): magic "U3VC" (0x43563355 LE), 12-byte header,
 * READMEM cmd 0x0800 / ack 0x0801, REQUEST_ACK flag 1<<14.
 *
 * Aravis-ordered init: manufacturer name -> SBRM ptr -> capability -> manifest.
 * Usage: probe [reset] [out.zip]   (downloads the camera's GenICam XML, default genicam.zip)
 */
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

#define VID 0x1ab2
#define PID 0x0001

static uint16_t pkt_id = 1;

static void put16le(uint8_t *p, uint16_t v) { p[0] = v & 0xff; p[1] = v >> 8; }
static void put32le(uint8_t *p, uint32_t v) { put16le(p, v & 0xffff); put16le(p + 2, v >> 16); }
static void put64le(uint8_t *p, uint64_t v) { put32le(p, v & 0xffffffff); put32le(p + 4, v >> 32); }
static uint16_t get16le(const uint8_t *p) { return p[0] | (p[1] << 8); }
static uint64_t get64le(const uint8_t *p)
{
    uint64_t v = 0;
    for (int i = 7; i >= 0; i--) v = (v << 8) | p[i];
    return v;
}

static const char *statname(uint16_t s)
{
    switch (s) {
    case 0x0000: return "SUCCESS";
    case 0x8001: return "NOT_IMPLEMENTED";
    case 0x8002: return "INVALID_PARAMETER";
    case 0x8003: return "INVALID_ADDRESS";
    case 0x8005: return "BAD_ALIGNMENT";
    case 0x8006: return "ACCESS_DENIED";
    case 0x8007: return "BUSY";
    case 0x800E: return "INVALID_HEADER";
    case 0x8FFF: return "GENERIC_ERROR";
    default: return "?";
    }
}

static int readmem(libusb_device_handle *h, uint64_t addr, uint16_t len,
                   uint8_t *out, int outmax, int retries)
{
    uint8_t cmd[24];
    static uint8_t rsp[65536];

    for (int attempt = 0; attempt <= retries; attempt++) {
        put32le(cmd + 0, 0x43563355);  /* magic "U3VC" */
        put16le(cmd + 4, 1 << 14);     /* flags: REQUEST_ACK */
        put16le(cmd + 6, 0x0800);      /* command: READ_MEMORY_CMD */
        put16le(cmd + 8, 12);          /* body size */
        put16le(cmd + 10, pkt_id++);   /* packet id */
        put64le(cmd + 12, addr);
        put16le(cmd + 20, 0);
        put16le(cmd + 22, len);

        int sent = 0, r;
        r = libusb_bulk_transfer(h, 0x01, cmd, sizeof(cmd), &sent, 2000);
        if (r) { fprintf(stderr, "ctrl out: %s\n", libusb_error_name(r)); return r; }

        int rx = 0;
        r = libusb_bulk_transfer(h, 0x81, rsp, sizeof(rsp), &rx, 2000);
        if (r) { fprintf(stderr, "ctrl in: %s\n", libusb_error_name(r)); return r; }

        if (rx < 12 || memcmp(rsp, "U3VC", 4) != 0) {
            fprintf(stderr, "bad response (magic), rx=%d\n", rx); return -1;
        }
        uint16_t status = get16le(rsp + 4);
        uint16_t ack_id = get16le(rsp + 10);
        if (status != 0) {
            fprintf(stderr, "status=0x%04x (%s) ack_id=%u\n", status, statname(status), ack_id);
            continue; /* retry */
        }
        int avail = rx - 12;
        int n = len < avail ? len : avail;
        memcpy(out, rsp + 12, n);
        return n;
    }
    return -2;
}

static void hexdump(const char *name, uint64_t addr, const uint8_t *b, int n)
{
    printf("== %s (0x%04llX, %d bytes)\n", name, (unsigned long long)addr, n);
    for (int i = 0; i < n; i += 16) {
        printf("  %04llX: ", (unsigned long long)(addr + i));
        for (int j = 0; j < 16 && i + j < n; j++) printf("%02X ", b[i + j]);
        printf(" |");
        for (int j = 0; j < 16 && i + j < n; j++) {
            uint8_t c = b[i + j];
            putchar(c >= 0x20 && c < 0x7f ? c : '.');
        }
        printf("|\n");
    }
}

int main(int argc, char **argv)
{
    libusb_context *ctx = NULL;
    int r = libusb_init(&ctx);
    if (r) { fprintf(stderr, "libusb_init: %d\n", r); return 1; }

    libusb_device_handle *h = libusb_open_device_with_vid_pid(ctx, VID, PID);
    if (!h) { fprintf(stderr, "device %04x:%04x not found\n", VID, PID); return 1; }

    libusb_detach_kernel_driver(h, 0);
    r = libusb_claim_interface(h, 0);
    if (r) { fprintf(stderr, "claim if0: %s\n", libusb_error_name(r)); return 1; }

    if (argc > 1 && !strcmp(argv[1], "reset")) {
        printf("USB reset...\n");
        r = libusb_reset_device(h);
        printf("reset: %s\n", libusb_error_name(r));
        usleep(2000000);
    }

    /* drain stale pending data from the control IN endpoint */
    {
        static uint8_t drain[65536];
        int rx, drained = 0;
        for (int i = 0; i < 32; i++) {
            r = libusb_bulk_transfer(h, 0x81, drain, sizeof(drain), &rx, 100);
            if (r == LIBUSB_ERROR_TIMEOUT) break;
            drained += rx;
        }
        if (drained) printf("drained %d stale bytes from ctrl IN\n", drained);
    }

    uint8_t buf[4096];
    int n;
    uint64_t sbrm = 0, manifest = 0;

    /* Aravis-ordered init reads */
    n = readmem(h, 0x0004, 64, buf, sizeof(buf), 2);          /* manufacturer name */
    if (n > 0) { buf[n] = 0; printf("manufacturer: '%s'\n", buf); }
    else { printf("manufacturer read FAILED\n"); goto out; }

    n = readmem(h, 0x01d8, 8, buf, sizeof(buf), 2);           /* SBRM address */
    if (n > 0) { sbrm = get64le(buf); printf("SBRM address: 0x%llx\n", (unsigned long long)sbrm); }

    n = readmem(h, 0x01c4, 4, buf, sizeof(buf), 2);           /* device capability */
    if (n > 0) printf("device capability: 0x%08x\n", *(uint32_t *)buf);

    n = readmem(h, 0x01d0, 8, buf, sizeof(buf), 2);           /* manifest table address */
    if (n > 0) { manifest = get64le(buf); printf("manifest table: 0x%llx\n", (unsigned long long)manifest); }

    if (sbrm) {
        n = readmem(h, sbrm + 0x0014, 4, buf, sizeof(buf), 2); /* SBRM max cmd transfer */
        if (n > 0) printf("max cmd transfer: %u\n", *(uint32_t *)buf);
        n = readmem(h, sbrm + 0x0018, 4, buf, sizeof(buf), 2); /* SBRM max ack transfer */
        if (n > 0) printf("max ack transfer: %u\n", *(uint32_t *)buf);
        n = readmem(h, sbrm + 0x0020, 8, buf, sizeof(buf), 2); /* SIRM address */
        if (n > 0) printf("SIRM address: 0x%llx\n", (unsigned long long)get64le(buf));
    }

    /* manifest: +0x00 = n entries (u64), +0x08 = first entry
     * entry: subminor(2) minor(1) major(1) schema(4) address(8) size(8)... */
    if (manifest) {
        n = readmem(h, manifest + 0x08, 32, buf, sizeof(buf), 2);
        if (n > 0) {
            uint64_t xml_addr = get64le(buf + 8);
            uint64_t xml_size = get64le(buf + 16);
            printf("XML entry: address=0x%llx size=%llu schema=0x%08x\n",
                   (unsigned long long)xml_addr, (unsigned long long)xml_size,
                   *(uint32_t *)(buf + 4));
            /* download full XML archive (chunk <= max ack payload) */
            uint64_t got = 0;
            uint8_t xb[512];
            uint32_t chunk = 480;   /* fits under 512-byte max ack */
            printf("--- downloading XML archive (%llu bytes) ---\n",
                   (unsigned long long)xml_size);
            const char *outname = "genicam.zip";
            for (int ai = 1; ai < argc; ai++)
                if (strcmp(argv[ai], "reset")) outname = argv[ai];
            FILE *zf = fopen(outname, "wb");
            while (got < xml_size) {
                if (xml_size - got < chunk) chunk = (uint32_t)(xml_size - got);
                n = readmem(h, xml_addr + got, chunk, xb, sizeof(xb), 2);
                if (n <= 0) { printf("(XML read failed at %llu, ret=%d)\n", (unsigned long long)got, n); break; }
                if (zf) fwrite(xb, 1, n, zf);
                got += n;
            }
            if (zf) fclose(zf);
            printf("--- done, %llu bytes -> %s ---\n", (unsigned long long)got, outname);
        }
    }

out:
    libusb_close(h);
    libusb_exit(ctx);
    printf("probe done\n");
    return 0;
}
