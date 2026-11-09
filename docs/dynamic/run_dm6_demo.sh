#!/bin/bash
# DM6 reproducible demo: three paired experiments (identical seeds/scenarios across arms). Usage: run_dm6_demo.sh <path-to-tdr_dynamic_experiment> <out-dir>
# Wall-clock (runtime.csv) is the only non-deterministic output.
X=${1:?experiment binary}; O=${2:?out dir}
$X --out=$O/E1_baseline_W2 --overwrite                                                    # n=20 rho=1 omega=60 W~=2 eps~={0.5,2,8}
$X --out=$O/E2_wide_window_W8 --w-tilde=8 --overwrite                                      # W effectively slack: isolates the tolerance E
$X --out=$O/E3_slow_optimiser --omega=600 --rho=2 --w-tilde=1.5 --eps-list=2 --overwrite   # long queue: exercises expired / stale / queue wait
