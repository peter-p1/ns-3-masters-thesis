#!/usr/bin/env bash
# Run the NTN NB-IoT scenario across an increasing UE population and collect KPIs
# into a CSV. Used for the saturation / latency / retransmission analysis.
#
# Usage: scratch/run-ue-scaling.sh [sim_time_seconds] [seed]
#
# Output: results/ue-scaling-<timestamp>.csv

set -euo pipefail

SIM_TIME="${1:-60}"        # seconds
SEED="${2:-21}"
PACKET_INTERVAL="${3:-10}" # seconds
JITTER_MS="${4:-1000}"
# Constellation selection (defaults to the 4-sat Sateliot set used so far).
CONSTELLATION_LABEL="${5:-sateliot}"
TLE_FILE="${6:-scratch/tle-data/sateliot.tle}"
CONSTELLATION_SIZE="${7:-4}"

UE_COUNTS=(1 10 50 100 250)

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$REPO_ROOT"

STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_DIR="$REPO_ROOT/results"
mkdir -p "$OUT_DIR"
CSV="$OUT_DIR/ue-scaling-${CONSTELLATION_LABEL}-$STAMP.csv"

echo "numUes,connectedUes,connectRatePct,meanConnDelayMs,meanPingRttMs,totalRaStarts,totalRaTimeouts,meanRaTimeoutsPerUe,collisionEvents,collidedAttempts,pingTx,pingRx,pingLossPct,wallSeconds" > "$CSV"

# Build once up front so per-run timing is the actual simulation cost.
./ns3 build > /dev/null

for N in "${UE_COUNTS[@]}"; do
  SIM_NAME="SCALE-${CONSTELLATION_LABEL}-${N}ue"
  echo "==> [${CONSTELLATION_LABEL}, ${CONSTELLATION_SIZE} sats] Running with ${N} UEs (simTime=${SIM_TIME}s, jitter=${JITTER_MS}ms)..."
  T0=$(date +%s)
  ARGS="--simName=${SIM_NAME} --numUes=${N} --simTime=${SIM_TIME}s --packetInterval=${PACKET_INTERVAL}s --maxStartJitterMs=${JITTER_MS} --randomSeed=${SEED} --tleFile=${TLE_FILE} --constellationSize=${CONSTELLATION_SIZE}"
  RAW="$OUT_DIR/${SIM_NAME}-stdout.log"

  # Tolerant: if the sim crashes (ns-3 LTE state-machine race at high UE
  # density), log it and continue with the next UE count instead of aborting
  # the whole sweep. The CSV simply omits that row.
  if ! ./ns3 run "scratch/lena-nb-5G-ntn-scenario ${ARGS}" > "$RAW" 2>&1; then
    T1=$(date +%s); WALL=$((T1 - T0))
    echo "    -> FAILED after ${WALL}s (sim crashed; see ${RAW##*/})"
    continue
  fi
  T1=$(date +%s)
  WALL=$((T1 - T0))

  # Extract KPIs (single-line, parseable)
  get_kpi() {
    grep -E "^\[KPI\] $1=" "$RAW" | tail -1 | sed -E "s/^\[KPI\] $1=//" | tr -d ' '
  }
  CU=$(get_kpi connectedUes)
  CR=$(get_kpi connectRatePct)
  MC=$(get_kpi meanConnDelayMs)
  MR=$(get_kpi meanPingRttMs)
  RS=$(get_kpi totalRaStarts)
  RT=$(get_kpi totalRaTimeouts)
  MRT=$(get_kpi meanRaTimeoutsPerUe)
  CE=$(get_kpi collisionEvents)
  CA=$(get_kpi collidedAttempts)
  PTRX=$(grep -E "^\[KPI\] pingTx=" "$RAW" | tail -1 | sed -E 's/^\[KPI\] pingTx=([0-9]+), pingRx=([0-9]+)$/\1,\2/' | tr -d ' ')
  PL=$(get_kpi pingLossPct)

  echo "${N},${CU},${CR},${MC},${MR},${RS},${RT},${MRT},${CE},${CA},${PTRX},${PL},${WALL}" >> "$CSV"
  echo "    -> connected=${CU}/${N}, meanConn=${MC}ms, meanPing=${MR}ms, RAtimeouts=${RT}, collisions=${CE}, wall=${WALL}s"
done

echo
echo "Wrote: $CSV"
