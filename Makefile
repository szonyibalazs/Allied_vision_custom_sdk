CC      ?= gcc
CFLAGS  ?= -O2 -Wall
LDLIBS   = -lusb-1.0 -lpthread

all: u3v probe

u3v: u3v.c
	$(CC) $(CFLAGS) -o $@ $< $(LDLIBS)

probe: probe.c
	$(CC) $(CFLAGS) -o $@ $< -lusb-1.0

clean:
	rm -f u3v probe

.PHONY: all clean
