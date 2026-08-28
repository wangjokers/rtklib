#!/usr/bin/env bash
set -euo pipefail

# Minimal Linux build for the current target snapshot. This builds the RTKLIB
# core (including rtksvr, Sino, RTCM3, and PPP-B2b sources) as a static library
# and links both the post-processing and realtime file-stream entries.

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/.." && pwd)"
build_dir="${repo_root}/build/raspberry-pi"
cc="${CC:-gcc}"
ar="${AR:-ar}"

common_flags=(
    -std=c99 -O2 -Wall -Wextra -pthread
    -DENAGLO -DENAGAL -DENAQZS -DENACMP -DTRACE
    -I"${repo_root}/src"
)

core_sources=(
    src/b2b.c
    src/convgpx.c
    src/convkml.c
    src/convrnx.c
    src/datum.c
    src/download.c
    src/ephemeris.c
    src/geoid.c
    src/gis.c
    src/ionex.c
    src/lambda.c
    src/options.c
    src/pntpos.c
    src/postpos.c
    src/ppp_ar.c
    src/ppp.c
    src/preceph.c
    src/rcv/binex.c
    src/rcv/crescent.c
    src/rcv/javad.c
    src/rcv/novatel.c
    src/rcv/nvs.c
    src/rcv/rt17.c
    src/rcv/septentrio.c
    src/rcv/sinan.c
    src/rcv/skytraq.c
    src/rcv/ss2.c
    src/rcv/ublox.c
    src/rcv/unicore.c
    src/rcvraw.c
    src/rinex.c
    src/rtcm.c
    src/rtcm2.c
    src/rtcm3.c
    src/rtcm3e.c
    src/rtkcmn.c
    src/rtkpos.c
    src/rtksvr.c
    src/sbas.c
    src/solution.c
    src/stream.c
    src/streamsvr.c
    src/tides.c
    src/tle.c
)

mkdir -p "${build_dir}/obj"
objects=()

for source in "${core_sources[@]}"; do
    object="${build_dir}/obj/${source#src/}"
    object="${object%.c}.o"
    mkdir -p "$(dirname "${object}")"
    "${cc}" "${common_flags[@]}" -c "${repo_root}/${source}" -o "${object}"
    objects+=("${object}")
done

"${ar}" rcs "${build_dir}/librtklib_b2b.a" "${objects[@]}"
"${cc}" "${common_flags[@]}" \
    "${repo_root}/src/rnx2rtkp.c" \
    "${build_dir}/librtklib_b2b.a" \
    -lm -pthread -o "${build_dir}/rnx2rtkp"

"${cc}" "${common_flags[@]}" \
    "${repo_root}/app/rtppp/rtppp.c" \
    "${build_dir}/librtklib_b2b.a" \
    -lm -pthread -o "${build_dir}/rtppp"

"${build_dir}/rnx2rtkp" -? >/dev/null 2>&1
"${build_dir}/rtppp" --help >/dev/null

printf 'Linux build probe passed.\n'
printf 'Core library: %s\n' "${build_dir}/librtklib_b2b.a"
printf 'Post-process executable: %s\n' "${build_dir}/rnx2rtkp"
printf 'Realtime executable: %s\n' "${build_dir}/rtppp"
