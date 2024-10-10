CC=gcc
CFLAGS=-Ofast -Wall
REMOVERS?=0
LIBUSB?=1

ifeq ($(REMOVERS),1)
CFLAGS+=-DREMOVERS
endif
ifeq ($(LIBUSB),1)
CFLAGS+=-DLIBUSB_1
LNKUSB=libusb-1.0
else
LNKUSB=usb
endif

SRCC=jcp2.c
SRCC+=jcp_handler.c
SRCH=dumpver.h flashstub.h romdump.h turbow.h univbin.h
SRCH+=jcp_handler.h
OBJS=$(SRCC:.c=.o) 

all: .depend jcp2 

jcp2: $(OBJS) $(SRCH)
	gcc -o jcp2 $(OBJS) -l$(LNKUSB)

%.o: %.c
	$(CC) $(CFLAGS) -c $<

clean:
	rm -f *~ $(OBJS) jcp2 .depend

.depend: $(SRCC)
	$(CC) -MM $(SRCC) > .depend

-include .depend
