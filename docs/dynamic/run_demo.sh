#!/bin/bash
# Deterministic presentation demo (read-only trace; no wall-clock values). Usage: run_demo.sh <path-to-tdr_dynamic> [A|B]
# A (main):    n0=15, regime F, rho=0.8, omega=900 s, G3 + A-PM (W~=1.5 -> W=6000 s, eps~=2), scenario sigma=0: accepted (primary), rejected-promise, stale, out-of-scan.
# B (expiry):  same but omega=1200 s, sigma=5: additionally shows EXPIRED requests and a longer FIFO wait.
# Optimiser = identity; prototype = truck-only insertion; default seed 20261109, 150 VNS iterations for the base plan.
X=${1:?tdr_dynamic binary}; V=${2:-A}
COMMON="--n=15 --regime=F --rho=0.8 --w-tilde=1.5 --eps-tilde=2 --scan=g3 --admission=apm --vns-iters=150 --scenarios=1"
if [ "$V" = "B" ]; then $X $COMMON --omega=1200 --sigma=5 --trace; else $X $COMMON --omega=900 --sigma=0 --trace; fi
