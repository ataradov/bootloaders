UNAME := $(shell uname)

SRCS = \
  main.c \

HDRS = \
  usb_hid.h \

ifeq ($(UNAME), Linux)
  BIN = hid_boot
  SRCS += usb_hid_lin.c
  LIBS += -ludev
else
  BIN = hid_boot.exe
  SRCS += usb_hid_win.c
  LIBS += -lhid -lsetupapi
endif

CFLAGS += -W -Wall -Wextra -O2 -std=gnu99
CFLAGS += -Wno-deprecated-declarations

all: $(BIN)

$(BIN): $(SRCS) $(HDRS)
	gcc $(CFLAGS) $(SRCS) $(LIBS) -o $(BIN)

clean:
	rm -rvf $(BIN)

