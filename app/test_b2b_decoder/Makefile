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
SINO_RECEIVER_TARGET := test_sino_receiver$(EXEEXT)
UNICORE_RECEIVER_TARGET := test_unicore_receiver$(EXEEXT)
REPLAY_TARGET := test_b2b_replay$(EXEEXT)
STAGE4E_DIAG_TARGET := test_b2b_rinex4_cnv1_diag$(EXEEXT)
CBIAS_TARGET := test_b2b_cbias$(EXEEXT)
OPTIONS_TARGET := test_b2b_options$(EXEEXT)
RECEIVER_COMMON_SRCS := test_rtklib_support.c ../../src/b2b.c \
	../../src/rcvraw.c ../../src/rcv/unicore.c ../../src/rcv/sinan.c
REPLAY_SRCS := test_b2b_replay.c $(RECEIVER_COMMON_SRCS) \
	../../src/postpos.c ../../src/options.c
STAGE4E_DIAG_SRCS := test_b2b_rinex4_cnv1_diag.c ../../src/b2b.c \
	../../src/rcvraw.c $(wildcard ../../src/rcv/*.c) ../../src/rtkcmn.c \
	../../src/rinex.c ../../src/ephemeris.c ../../src/preceph.c \
	../../src/pntpos.c ../../src/sbas.c ../../src/ionex.c
CBIAS_SRCS := test_b2b_cbias.c ../../src/b2b.c ../../src/rtkcmn.c
OPTIONS_SRCS := test_b2b_options.c ../../src/options.c

# Default build keeps stage 1 isolated from rtklib/src. Set USE_RTKCMN=1 after
# the target rtkcmn.c is build-clean if you want to link the original helpers.
ifeq ($(USE_RTKCMN),1)
CFLAGS += -DB2B_USE_RTKCMN
SRCS := test_b2b_decoder.c b2b_decoder.c ../../src/rtkcmn.c
else
SRCS := test_b2b_decoder.c b2b_decoder.c
endif

.PHONY: all sino-receiver unicore-receiver replay stage4e-diag cbias options

all: $(TARGET)

sino-receiver: $(SINO_RECEIVER_TARGET)

unicore-receiver: $(UNICORE_RECEIVER_TARGET)

replay: $(REPLAY_TARGET)

stage4e-diag: $(STAGE4E_DIAG_TARGET)

cbias: $(CBIAS_TARGET)

options: $(OPTIONS_TARGET)

$(TARGET): $(SRCS) b2b_decoder.h
	$(CC) $(CFLAGS) -o $@ $(SRCS) $(LDLIBS)

$(SINO_RECEIVER_TARGET): test_sino_receiver.c $(RECEIVER_COMMON_SRCS)
	$(CC) $(CFLAGS) -I../../src/rcv -o $@ test_sino_receiver.c $(RECEIVER_COMMON_SRCS) $(LDLIBS)

$(UNICORE_RECEIVER_TARGET): test_unicore_receiver.c $(RECEIVER_COMMON_SRCS)
	$(CC) $(CFLAGS) -I../../src/rcv -o $@ test_unicore_receiver.c $(RECEIVER_COMMON_SRCS) $(LDLIBS)

$(REPLAY_TARGET): $(REPLAY_SRCS)
	$(CC) $(CFLAGS) -DB2B_REPLAY_TEST_ONLY -I../../src/rcv \
		-ffunction-sections -fdata-sections \
		-o $@ $(REPLAY_SRCS) -Wl,--gc-sections $(LDLIBS)

$(STAGE4E_DIAG_TARGET): $(STAGE4E_DIAG_SRCS)
	$(CC) $(CFLAGS) -I../../src/rcv -ffunction-sections -fdata-sections \
		-o $@ $(STAGE4E_DIAG_SRCS) -Wl,--gc-sections $(LDLIBS)

$(CBIAS_TARGET): $(CBIAS_SRCS)
	$(CC) $(CFLAGS) -ffunction-sections -fdata-sections \
		-o $@ $(CBIAS_SRCS) -Wl,--gc-sections $(LDLIBS)

$(OPTIONS_TARGET): $(OPTIONS_SRCS)
	$(CC) $(CFLAGS) -DB2B_REPLAY_TEST_ONLY -ffunction-sections \
		-fdata-sections -o $@ $(OPTIONS_SRCS) -Wl,--gc-sections $(LDLIBS)
