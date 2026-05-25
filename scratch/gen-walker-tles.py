#!/usr/bin/env python3
"""Generate Walker-Star TLE files at 600 km, 97.7 deg inclination
for 3, 6, 12 and 24 satellites (matching the 3GPP TR 38.821 / paper
reference scenarios M/P = 3/3, 6/6, 12/6, 24/12).

Outputs:
    scratch/tle-data/walker-3sat.tle
    scratch/tle-data/walker-6sat.tle
    scratch/tle-data/walker-12sat.tle
    scratch/tle-data/walker-24sat.tle
"""

import math
from pathlib import Path

MU = 398600.4418     # Earth gravitational parameter, km^3/s^2
RE = 6378.137        # Earth equatorial radius, km
ALTITUDE_KM = 600.0
INCLINATION_DEG = 97.7
EPOCH_YEAR = 26      # 2026
EPOCH_DAY = 100.0    # day-of-year (2026-04-10), matches Sateliot epoch window

# (M total satellites, P orbital planes); Walker-Star -> RAAN spread over 180 deg
CONFIGS = [(3, 3), (6, 6), (12, 6), (24, 12)]


def mean_motion_rev_per_day(altitude_km: float) -> float:
    a = RE + altitude_km
    period_s = 2.0 * math.pi * math.sqrt(a ** 3 / MU)
    return 86400.0 / period_s


def tle_checksum(line: str) -> int:
    s = 0
    for c in line[:68]:
        if c.isdigit():
            s += int(c)
        elif c == "-":
            s += 1
    return s % 10


def make_line1(sat_num: int, elem_set: int = 999) -> str:
    line = (
        "1 "                              # cols 1-2
        + f"{sat_num:05d}"                # cols 3-7
        + "U "                            # cols 8-9
        + "99001A  "                      # cols 10-17 (intl designator)
        + " "                             # col 18
        + f"{EPOCH_YEAR:02d}"             # cols 19-20
        + f"{EPOCH_DAY:012.8f}"           # cols 21-32
        + " "                             # col 33
        + " .00000000"                    # cols 34-43 (mean motion 1st deriv)
        + " "                             # col 44
        + " 00000-0"                      # cols 45-52 (2nd deriv)
        + " "                             # col 53
        + " 00000-0"                      # cols 54-61 (BSTAR)
        + " "                             # col 62
        + "0"                             # col 63 (ephemeris type)
        + " "                             # col 64
        + f"{elem_set:4d}"                # cols 65-68
    )
    assert len(line) == 68, f"line1 len={len(line)}: {line!r}"
    return line + str(tle_checksum(line))


def make_line2(sat_num: int, inc_deg: float, raan_deg: float,
               mean_anom_deg: float, mean_motion: float) -> str:
    ecc_str = "0000000"   # eccentricity = 0 (circular), implied leading 0.
    aop_deg = 0.0
    rev_num = 0
    line = (
        "2 "
        + f"{sat_num:05d}"
        + " "
        + f"{inc_deg:8.4f}"
        + " "
        + f"{raan_deg:8.4f}"
        + " "
        + ecc_str
        + " "
        + f"{aop_deg:8.4f}"
        + " "
        + f"{mean_anom_deg:8.4f}"
        + " "
        + f"{mean_motion:11.8f}"
        + f"{rev_num:5d}"
    )
    assert len(line) == 68, f"line2 len={len(line)}: {line!r}"
    return line + str(tle_checksum(line))


def generate_walker(M: int, P: int):
    """Walker-Star M/P at fixed altitude/inclination. Returns list of (name, l1, l2)."""
    assert M % P == 0, "M must be divisible by P"
    sats_per_plane = M // P
    n = mean_motion_rev_per_day(ALTITUDE_KM)
    delta_phi = 180.0 * P / M   # inter-plane phase shift (paper Eq.: 180 * P_orb / M)
    sat_id = 70000              # synthetic catalog number range
    entries = []
    for p in range(P):
        raan = (p * 180.0 / P) % 360.0
        for s in range(sats_per_plane):
            ma = (s * 360.0 / sats_per_plane + p * delta_phi) % 360.0
            name = f"WALKER_{M}SAT_P{p+1:02d}_S{s+1:02d}"
            entries.append((
                name,
                make_line1(sat_id),
                make_line2(sat_id, INCLINATION_DEG, raan, ma, n),
            ))
            sat_id += 1
    return entries


def main():
    out_dir = Path(__file__).resolve().parent / "tle-data"
    out_dir.mkdir(parents=True, exist_ok=True)
    n = mean_motion_rev_per_day(ALTITUDE_KM)
    print(f"Mean motion @ {ALTITUDE_KM:.0f} km: {n:.8f} rev/day "
          f"(period {86400.0/n/60.0:.2f} min)")
    for M, P in CONFIGS:
        entries = generate_walker(M, P)
        out = out_dir / f"walker-{M}sat.tle"
        with out.open("w") as f:
            for name, l1, l2 in entries:
                f.write(f"{name}\n{l1}\n{l2}\n")
        print(f"Wrote {out}  ({M} satellites, {P} planes)")


if __name__ == "__main__":
    main()
