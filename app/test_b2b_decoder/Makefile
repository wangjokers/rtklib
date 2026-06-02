CC ?= gcc

CFLAGS ?= -std=c99 -O2 -Wall -Wextra
CFLAGS += -DENAGLO -DENAGAL -DENAQZS -DENACMP -I../../src

LDLIBS ?= -lm
ifeq ($(OS),Windows_NT)
CFLAGS += -DWIN32
LDLIBS += -lwinmm
EXEEXT := .exe
else
EXEEXT :=
endif

TARGET := test_b2b_decoder$(EXEEXT)

# Default build keeps stage 1 isolated from rtklib/src. Set USE_RTKCMN=1 after
# the target rtkcmn.c is build-clean if you want to link the original helpers.
ifeq ($(USE_RTKCMN),1)
CFLAGS += -DB2B_USE_RTKCMN
SRCS := test_b2b_decoder.c b2b_decoder.c ../../src/rtkcmn.c
else
SRCS := test_b2b_decoder.c b2b_decoder.c
endif

.PHONY: all

all: $(TARGET)

$(TARGET): $(SRCS) b2b_decoder.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)
