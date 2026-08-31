CC ?= gcc
PYTHON ?= python

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
SINO_MESSAGE72_TARGET := ../../build/realtime-fixture/test_sino_message72$(EXEEXT)
RTKSVR_REPLAY_TARGET := ../../build/realtime-fixture/test_rtksvr_file_replay$(EXEEXT)
PHASE2A_TARGET := ../../build/realtime-fixture/test_rtcm1127_pcode$(EXEEXT)
MSM_EVENT_TARGET := ../../build/realtime-fixture/test_rtcm_msm_event$(EXEEXT)
PNTPOS_BDS_TARGET := ../../build/realtime-fixture/test_pntpos_bds_prange$(EXEEXT)
RTPPP_TARGET := ../../build/realtime-fixture/rtppp$(EXEEXT)
RTKSVR_REPLAY_LDLIBS := $(LDLIBS)
ifeq ($(OS),Windows_NT)
RTKSVR_REPLAY_LDLIBS += -lws2_32
endif
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
RTKSVR_REPLAY_SRCS := test_rtksvr_file_replay.c \
	../../src/b2b.c ../../src/convgpx.c ../../src/convkml.c \
	../../src/convrnx.c ../../src/datum.c ../../src/download.c \
	../../src/ephemeris.c ../../src/geoid.c ../../src/gis.c ../../src/ionex.c \
	../../src/lambda.c ../../src/options.c ../../src/pntpos.c ../../src/postpos.c \
	../../src/ppp_ar.c ../../src/ppp.c ../../src/preceph.c \
	$(wildcard ../../src/rcv/*.c) ../../src/rcvraw.c ../../src/rinex.c \
	../../src/rtcm.c ../../src/rtcm2.c ../../src/rtcm3.c ../../src/rtcm3e.c \
	../../src/rtkcmn.c ../../src/rtkpos.c ../../src/rtksvr.c ../../src/sbas.c \
	../../src/solution.c ../../src/stream.c ../../src/streamsvr.c \
	../../src/tides.c ../../src/tle.c

# Default build keeps stage 1 isolated from rtklib/src. Set USE_RTKCMN=1 after
# the target rtkcmn.c is build-clean if you want to link the original helpers.
ifeq ($(USE_RTKCMN),1)
CFLAGS += -DB2B_USE_RTKCMN
SRCS := test_b2b_decoder.c b2b_decoder.c ../../src/rtkcmn.c
else
SRCS := test_b2b_decoder.c b2b_decoder.c
endif

.PHONY: all sino-receiver unicore-receiver replay stage4e-diag cbias options \
	message72 fixture-test realtime-replay phase2a-realtime pntpos-bds-prange \
	msm-event phase3a-rtppp

all: $(TARGET)

sino-receiver: $(SINO_RECEIVER_TARGET)

unicore-receiver: $(UNICORE_RECEIVER_TARGET)

replay: $(REPLAY_TARGET)

stage4e-diag: $(STAGE4E_DIAG_TARGET)

cbias: $(CBIAS_TARGET)

options: $(OPTIONS_TARGET)

message72: $(SINO_MESSAGE72_TARGET)

fixture-test:
	$(PYTHON) -m unittest -v test_extract_krec_fixture.py

realtime-replay: $(RTKSVR_REPLAY_TARGET)

phase2a-realtime: $(PHASE2A_TARGET)

msm-event: $(MSM_EVENT_TARGET)

pntpos-bds-prange: $(PNTPOS_BDS_TARGET)

phase3a-rtppp: $(RTPPP_TARGET)

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

$(SINO_MESSAGE72_TARGET): test_sino_message72.c $(RECEIVER_COMMON_SRCS)
	mkdir -p ../../build/realtime-fixture
	$(CC) $(CFLAGS) -I../../src/rcv -o $@ test_sino_message72.c \
		$(RECEIVER_COMMON_SRCS) $(LDLIBS)

$(RTKSVR_REPLAY_TARGET): $(RTKSVR_REPLAY_SRCS)
	mkdir -p ../../build/realtime-fixture
	$(CC) $(CFLAGS) -DTRACE -DB2B_RTKSVR_TEST -pthread -o $@ \
		$(RTKSVR_REPLAY_SRCS) \
		$(RTKSVR_REPLAY_LDLIBS) -pthread

$(PHASE2A_TARGET): test_rtcm1127_pcode.c $(RTKSVR_REPLAY_SRCS)
	mkdir -p ../../build/realtime-fixture
	$(CC) $(CFLAGS) -DTRACE -pthread -o $@ test_rtcm1127_pcode.c \
		$(filter-out test_rtksvr_file_replay.c,$(RTKSVR_REPLAY_SRCS)) \
		$(RTKSVR_REPLAY_LDLIBS) -pthread

$(MSM_EVENT_TARGET): test_rtcm_msm_event.c $(RTKSVR_REPLAY_SRCS)
	mkdir -p ../../build/realtime-fixture
	$(CC) $(CFLAGS) -DTRACE -pthread -o $@ test_rtcm_msm_event.c \
		$(filter-out test_rtksvr_file_replay.c,$(RTKSVR_REPLAY_SRCS)) \
		$(RTKSVR_REPLAY_LDLIBS) -pthread

$(PNTPOS_BDS_TARGET): test_pntpos_bds_prange.c $(RTKSVR_REPLAY_SRCS)
	mkdir -p ../../build/realtime-fixture
	$(CC) $(CFLAGS) -DB2B_PNTPOS_TEST -pthread -o $@ \
		test_pntpos_bds_prange.c \
		$(filter-out test_rtksvr_file_replay.c,$(RTKSVR_REPLAY_SRCS)) \
		$(RTKSVR_REPLAY_LDLIBS) -pthread

$(RTPPP_TARGET): ../rtppp/rtppp.c $(RTKSVR_REPLAY_SRCS)
	mkdir -p ../../build/realtime-fixture
	$(CC) $(CFLAGS) -DTRACE -pthread -o $@ ../rtppp/rtppp.c \
		$(filter-out test_rtksvr_file_replay.c,$(RTKSVR_REPLAY_SRCS)) \
		$(RTKSVR_REPLAY_LDLIBS) -pthread
