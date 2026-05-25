#!/usr/bin/env python3
"""Overlay UE-scaling KPIs across multiple Walker-Star constellation sizes.

Pass one CSV per constellation; the script extracts the satellite count from
the filename pattern 'ue-scaling-walker-<N>sat-*' (or from a 'walker-<N>sat'
substring) and uses it as the curve label.

Usage:
    scratch/plot-constellation-compare.py results/ue-scaling-walker-3sat-*.csv \
                                          results/ue-scaling-walker-6sat-*.csv \
                                          results/ue-scaling-walker-12sat-*.csv \
                                          results/ue-scaling-walker-24sat-*.csv

Outputs PDFs into results/ (or the directory of the first CSV):
    latency-comparison.pdf
    retransmissions-comparison.pdf
    saturation-comparison.pdf
"""

import csv
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


SAT_RE = re.compile(r"walker-(\d+)sat", re.IGNORECASE)


def load(path: Path):
    rows = []
    with path.open() as f:
        for r in csv.DictReader(f):
            rows.append({k: float(v) if v not in ("", None) else 0.0 for k, v in r.items()})
    rows.sort(key=lambda r: r["numUes"])
    return rows


def label_from_path(path: Path) -> str:
    m = SAT_RE.search(path.name)
    if not m:
        return path.stem
    return f"{m.group(1)} sat."


def sort_key(path: Path) -> int:
    m = SAT_RE.search(path.name)
    return int(m.group(1)) if m else 1 << 30


def overlay(ax, datasets, x_key, y_key, marker, **kw):
    for label, rows in datasets:
        xs = [r[x_key] for r in rows]
        ys = [r[y_key] for r in rows]
        ax.plot(xs, ys, marker=marker, label=label, **kw)


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 2
    paths = sorted([Path(p) for p in argv[1:]], key=sort_key)
    datasets = [(label_from_path(p), load(p)) for p in paths]
    out_dir = paths[0].parent

    # ----- 1) Latency comparison: ping RTT (top) + RRC-connect delay (bottom)
    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    overlay(ax_top, datasets, "numUes", "meanPingRttMs", "o", linestyle="--")
    ax_top.set_ylabel("Priemerné RTT pingu (ms)")
    ax_top.set_xscale("log")
    ax_top.grid(True, which="both", linestyle="--", alpha=0.5)
    ax_top.legend(title="Konštelácia")
    overlay(ax_bot, datasets, "numUes", "meanConnDelayMs", "s", linestyle="--")
    ax_bot.set_xlabel("Počet UE")
    ax_bot.set_ylabel("Priemerné oneskorenie pripojenia RRC (ms)")
    ax_bot.set_xscale("log")
    ax_bot.set_xticks([1, 10, 50, 100, 250])
    ax_bot.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax_bot.set_xticklabels(["1", "10", "50", "100", "250"])
    ax_bot.grid(True, which="both", linestyle="--", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_dir / "latency-comparison.pdf", dpi=140)
    plt.close(fig)

    # ----- 2) Retransmissions: RA timeouts/UE (top) + NPRACH collisions (bottom)
    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    overlay(ax_top, datasets, "numUes", "meanRaTimeoutsPerUe", "o", linestyle="--")
    ax_top.set_ylabel("Priemerný počet vypršaní RA / UE")
    ax_top.set_xscale("log")
    ax_top.grid(True, which="both", linestyle="--", alpha=0.5)
    ax_top.legend(title="Konštelácia")
    overlay(ax_bot, datasets, "numUes", "collisionEvents", "s", linestyle="--")
    ax_bot.set_xlabel("Počet UE")
    ax_bot.set_ylabel("NPRACH kolízie")
    ax_bot.set_xscale("log")
    ax_bot.set_xticks([1, 10, 50, 100, 250])
    ax_bot.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax_bot.set_xticklabels(["1", "10", "50", "100", "250"])
    ax_bot.grid(True, which="both", linestyle="--", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_dir / "retransmissions-comparison.pdf", dpi=140)
    plt.close(fig)

    # ----- 3) Saturation: connect rate % (top) + ping loss % (bottom)
    fig, (ax_top, ax_bot) = plt.subplots(2, 1, figsize=(8, 8), sharex=True)
    overlay(ax_top, datasets, "numUes", "connectRatePct", "o", linestyle="--")
    ax_top.set_ylabel("Úspešnosť pripojenia (%)")
    ax_top.set_ylim(0, 105)
    ax_top.set_xscale("log")
    ax_top.grid(True, which="both", linestyle="--", alpha=0.5)
    ax_top.legend(title="Konštelácia")
    overlay(ax_bot, datasets, "numUes", "pingLossPct", "s", linestyle="--")
    ax_bot.set_xlabel("Počet UE")
    ax_bot.set_ylabel("Strata pingu (%)")
    ax_bot.set_ylim(0, 105)
    ax_bot.set_xscale("log")
    ax_bot.set_xticks([1, 10, 50, 100, 250])
    ax_bot.get_xaxis().set_major_formatter(matplotlib.ticker.ScalarFormatter())
    ax_bot.set_xticklabels(["1", "10", "50", "100", "250"])
    ax_bot.grid(True, which="both", linestyle="--", alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_dir / "saturation-comparison.pdf", dpi=140)
    plt.close(fig)

    for name in ("latency-comparison.pdf", "retransmissions-comparison.pdf", "saturation-comparison.pdf"):
        print(f"Wrote: {out_dir/name}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
