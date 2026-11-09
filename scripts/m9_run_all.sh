#!/usr/bin/env bash
# M9: reproduces every run of the validation (results/m9 and results/m9_sensitivity) from config/m9/*.cfg with the tdr_static binary.
# usage: scripts/m9_run_all.sh PATH_TO_tdr_static [main|sens|all]      (main ~ 3 min, sens ~ 4 min on one core)
set -euo pipefail
BIN=${1:?path to tdr_static}; WHAT=${2:-all}
cd "$(dirname "$0")/.."
if [[ $WHAT == main || $WHAT == all ]]; then
  for n in 10 20 50; do for s in 1 2 3; do
    "$BIN" run --config config/m9/n${n}_seed${s}.cfg --out results/m9/n${n}_seed${s} --quiet --overwrite
  done; done
fi
if [[ $WHAT == sens || $WHAT == all ]]; then
  # LABELLED SENSITIVITY RUNS (diagnostics only; the defaults and the main protocol are untouched): seed set 1, one factor changed per run.
  for n in 10 20 50; do
    "$BIN" run --config config/m9/n${n}_seed1.cfg --set experiment.name=m9_sens_corner_n${n} --set instance.depot_placement=corner \
        --out results/m9_sensitivity/corner_n${n} --quiet --overwrite
    "$BIN" run --config config/m9/n${n}_seed1.cfg --set experiment.name=m9_sens_endurance3600_n${n} --set vehicle.endurance_s=3600 \
        --out results/m9_sensitivity/endurance3600_n${n} --quiet --overwrite
  done
  # iteration-budget diagnostic: vns.iter_max 8000 against the protocol's 2000, same instances and run indices
  "$BIN" run --config config/m9/n20_seed1.cfg --set experiment.name=m9_sens_iter8000_n20 --set vns.iter_max=8000 \
      --out results/m9_sensitivity/iter8000_n20 --quiet --overwrite
  "$BIN" run --config config/m9/n50_seed1.cfg --set experiment.name=m9_sens_iter8000_n50_ds01 --set vns.iter_max=8000 --set experiment.num_datasets=2 --set instance.dataset_index=0 \
      --out results/m9_sensitivity/iter8000_n50_ds01 --quiet --overwrite
  "$BIN" run --config config/m9/n50_seed1.cfg --set experiment.name=m9_sens_iter8000_n50_ds23 --set vns.iter_max=8000 --set experiment.num_datasets=2 --set instance.dataset_index=2 \
      --out results/m9_sensitivity/iter8000_n50_ds23 --quiet --overwrite
fi
