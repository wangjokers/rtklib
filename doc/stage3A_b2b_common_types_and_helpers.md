# Stage 3A: B2b Common Types And Helpers

Date: 2026-06-03

This note records the stage 3A landing of PPP-B2b common data structures and helper APIs in the target `rtklib` project.

## Scope

Stage 3A only adds common B2b types and helper skeletons. It does not connect receiver decoding, raw stream dispatch, navigation updates, or PPP correction use.

Changed target areas:

- `rtklib/src/rtklib.h`
- `rtklib/src/b2b.c`

## Added Common Types

### `B2bmask_t`

`B2bmask_t` stores the latest PPP-B2b MASK message state for a receiver decoder context:

- `recv_time`: receiver/header time of the MASK message in GPST.
- `ref_time`: B2b reference epoch converted from BDT seconds-of-day to GPST.
- `MASK_BD[63]`: BeiDou mask bits in B2b slot order.
- `MASK_GPS[37]`: GPS mask bits in B2b slot order.
- `MASK_GAL[37]`: Galileo mask bits in B2b slot order.
- `MASK_GLO[37]`: GLONASS mask bits in B2b slot order.
- `IOD_SSR`: mask issue of data for orbit/code/clock consistency checks.
- `IODP`: mask issue of data for clock consistency checks.
- `satnum`: number of valid mapped RTKLIB satellite numbers.
- `satno[B2B_MAXSAT]`: B2b mask slots mapped to RTKLIB satellite numbers.

This type is intended for receiver decoder state. It is not stored in `nav_t`.

### `B2bssr_t`

`B2bssr_t` stores one satellite's PPP-B2b SSR products:

- `t0[0]`: orbit reference time.
- `t0[1]`: code-bias reference time.
- `t0[2]`: clock reference time.
- `udi[0..2]`: update intervals for orbit, code bias, and clock.
- `iodssr[0..2]`: B2b IOD SSR values by product type.
- `iodp[]`: B2b IODP values.
- `iodn`: broadcast ephemeris IODN/IODC associated with B2b orbit.
- `iodcorr[0]`: orbit correction IOD.
- `iodcorr[1]`: clock correction IOD.
- `deph[3]`: radial, along-track, and cross-track orbit corrections in meters.
- `ddeph[3]`: orbit correction rates in meters per second.
- `ura`: B2b URAI/URA indicator.
- `cbias[MAXCODE+1]`: code biases indexed by RTKLIB `CODE_*`; index 0 is unused.
- `dclk[3]`: clock correction coefficients.
- `update`: decoder update flag.

`B2bssr_t` is intentionally separate from standard `ssr_t`. B2b products must not be stored in `nav->ssr[]`.

## `nav_t.B2bssr[MAXSAT+1]` Indexing

Stage 3A adds:

```c
B2bssr_t B2bssr[MAXSAT+1];
```

This array uses B2b-specific `[sat]` indexing:

- valid `sat` values are `1..MAXSAT`;
- element `0` is unused;
- this differs from standard RTKLIB SSR storage such as `nav->ssr[sat-1]`.

This explicit `MAXSAT+1` size is required to avoid the boundary risk in reference code paths that use `nav->B2bssr[sat]`.

## Helper APIs

### `int b2b_slot2satno(int slot)`

Converts a PPP-B2b broadcast slot to an RTKLIB satellite number.

Slot ranges:

| B2b slot range | System |
|---|---|
| `1..63` | BeiDou |
| `64..100` | GPS |
| `101..137` | Galileo |
| `138..174` | GLONASS |

The helper returns `0` for out-of-range slots or satellites unsupported by the current compile-time RTKLIB system configuration.

### `int b2b_mask2satno(B2bmask_t *mask)`

Merges the four system MASK arrays into B2b slot order, converts active slots to RTKLIB satellite numbers, fills `mask->satno[]`, updates `mask->satnum`, and returns the number of valid satellites.

This prepares the satellite list needed by future CLOCK message decoding.

### `gtime_t b2b_tod2time(gtime_t header_time, double bdt_sod)`

Converts B2b payload BDT seconds-of-day to RTKLIB GPST.

The helper:

1. converts the receiver frame header time from GPST to BDT;
2. uses the BDT calendar date as the initial product date;
3. attaches `bdt_sod`;
4. applies the half-day cross-day rule used by the stage 1 decoder;
5. converts the result back to GPST.

## Not Implemented In Stage 3A

Stage 3A has not added:

- Unicore/UM980 receiver decoder integration;
- SinoGNSS/K803W receiver decoder integration;
- `input_raw()` / `input_rawf()` dispatch;
- `STRFMT_UNICORE` or `STRFMT_SINO` stream format branches;
- `raw_t` B2b message counters;
- `raw->nav.B2bssr[]` to main `nav->B2bssr[]` update helper;
- `postpos.c` or `rtksvr.c` B2b update paths;
- `ephemeris.c::satpos_B2b()`;
- `ppp.c::corr_meas()` code-bias use;
- any claim that PPP-B2b processing is complete.

## Stage 3B Recommendation

The next minimal step should be stage 3B:

1. Add a small Unicore receiver decoder module for `PPPB2BINFO1/2/3/4`.
2. Keep frame sync, CRC, and message decode helpers `static`.
3. Write decoded products only to `raw->nav.B2bssr[sat]`.
4. Keep `input_raw()` / `input_rawf()` dispatch out until the decoder compiles independently.
5. Validate against the existing stage 1 `test_b2b_decoder` output before touching PPP flow.
