#!/usr/bin/env python3
"""Plot UE-scaling KPIs (latency, retransmissions, collisions, connect rate) vs UE count.

Usage:
    scratch/plot-ue-scaling.py results/ue-scaling-<timestamp>.csv [out_dir]

Outputs PNGs next to the CSV (or in out_dir if given).
"""

import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    csv_path = Path(argv[1])
    out_dir = Path(argv[2]) if len(argv) > 2 else csv_path.parent
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    with csv_path.open() as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) if v not in ("", None) else 0.0 for k, v in r.items()})
    rows.sort(key=lambda r: r["numUes"])
    ues = [r["numUes"] for r in rows]

    # 1) Latency vs UEs (mean ping RTT and mean connection-establishment delay)
    fig, ax1 = plt.subplots(figsize=(8, 5))
    ax1.plot(ues, [r["meanPingRttMs"] for r in rows], "o-", color="tab:blue", label="Mean ping RTT")
    ax1.plot(ues, [r["meanConnDelayMs"] for r in rows], "s--", color="tab:orange", label="Mean RRC-connect delay")
    ax1.set_xlabel("Number of UEs")
    ax1.set_ylabel("Latency (ms)")
    ax1.set_title("Latency vs UE count (4-sat Sateliot, Set-2)")
    ax1.set_xscale("log")
    ax1.grid(True, which="both", linestyle="--", alpha=0.5)
    ax1.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "latency-vs-ues.png", dpi=140)
    plt.close(fig)

    # 2) Retransmissions and collisions vs UEs
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(ues, [r["meanRaTimeoutsPerUe"] for r in rows], "o-", color="tab:red", label="Mean RA timeouts / UE")
    ax2 = ax.twinx()
    ax2.plot(ues, [r["collisionEvents"] for r in rows], "s--", color="tab:purple", label="NPRACH collision events")
    ax.set_xlabel("Number of UEs")
    ax.set_ylabel("RA timeouts / UE", color="tab:red")
    ax2.set_ylabel("Collision events", color="tab:purple")
    ax.set_xscale("log")
    ax.set_title("Retransmissions and NPRACH collisions vs UE count")
    ax.grid(True, which="both", linestyle="--", alpha=0.5)
    lines, labels = ax.get_legend_handles_labels()
    l2, lab2 = ax2.get_legend_handles_labels()
    ax.legend(lines + l2, labels + lab2, loc="upper left")
    fig.tight_layout()
    fig.savefig(out_dir / "retransmissions-vs-ues.png", dpi=140)
    plt.close(fig)

    # 3) Connect rate (saturation indicator)
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.plot(ues, [r["connectRatePct"] for r in rows], "o-", color="tab:green", label="Connect-rate %")
    ax.plot(ues, [r["pingLossPct"] for r in rows], "s--", color="tab:gray", label="Ping loss %")
    ax.set_xlabel("Number of UEs")
    ax.set_ylabel("%")
    ax.set_xscale("log")
    ax.set_ylim(0, 105)
    ax.set_title("Saturation: connect rate and ping loss vs UE count")
    ax.grid(True, which="both", linestyle="--", alpha=0.5)
    ax.legend()
    fig.tight_layout()
    fig.savefig(out_dir / "saturation-vs-ues.png", dpi=140)
    plt.close(fig)

    print(f"Wrote: {out_dir/'latency-vs-ues.png'}")
    print(f"Wrote: {out_dir/'retransmissions-vs-ues.png'}")
    print(f"Wrote: {out_dir/'saturation-vs-ues.png'}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
