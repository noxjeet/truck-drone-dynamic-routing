# Static reproduction: documentation index and specification coverage (frozen at M10, 9 Nov 2026)

Start with `static_reproduction_summary.md` (thesis-ready). `MILESTONES.md` is the chronological log: figures inside a milestone section are the figures **at that milestone** (later sections and the table below supersede them where they differ; stale statements were corrected at M10 and are listed in `MILESTONES.md`, section M10).

## 1. Milestone status (authoritative)
| Milestone | Content | Signed off | Main document |
|---|---|---|---|
| M0 | decisions D1-D10, plan | 9 Nov 2026 (user) | `DECISIONS.md` ("Static reproduction") |
| M1 | infrastructure, evaluator profile, P1/P2 performance follow-up | 9 Nov 2026 | `phase_spec_M1.md`, `performance_profile_M1.md`, `performance_followup_P1_P2.md` |
| M2 | N4, N5 (+ bounded N4 re-anchoring) | 9 Nov 2026 08:06 IST | `operators_N4_N5.md` |
| M3 | N8, N7 | 9 Nov 2026 | `operators_N7_N8.md` |
| M4 | N3, N6 (A8 revised) | 9 Nov 2026 12:30 IST | `operators_N3_N6.md` |
| M5 | N1, N2 (customer-node launch omission kept as a limitation) | 9 Nov 2026 13:19 IST | `operators_N1_N2.md` |
| M6 | four shakes | 9 Nov 2026 13:56 IST | `shakes.md` |
| M7 | VNS driver (A38 reporting condition) | 9 Nov 2026 14:34 IST | `vns.md` |
| M8 | runner, writer, configuration, CLI (+ closure: exit-1 test, Windows validation) | 9 Nov 2026 15:35 IST | `runner_cli.md`, `output_schema.md`, `windows_validation.md` |
| M9 | validation and comparison with the paper (**not** a numerical reproduction) | 9 Nov 2026 16:13 IST | `validation_vs_paper.md` |
| M10 | documentation freeze | see `MILESTONES.md` | this index, `static_reproduction_summary.md` |

## 2. Specification coverage
| Component | Specification (contract, assumptions) | Code | Tests (hand-derived / oracle) |
|---|---|---|---|
| Mathematical model, units, timing, objective, seeds | `math_spec.md` | `include/model`, `graph`, `flp`, `timing`, `util` | `test_model`, `test_graph`, `test_flp`, `test_timing`, `test_evaluator_differential` |
| Initial solution (TSP, k-means, FLP pool, greedy trips) | `README.md`, `DECISIONS.md`, `experiment_methodology.md` section 7 | `include/init`, `include/tsp` | `test_tsp`, `test_init` |
| Instance generator, configuration types, search infrastructure | `phase_spec_M1.md`, `configuration_reference.md` | `include/instance`, `include/config/static_config.hpp`, `include/search/{move_types,route_ops,candidate_window,drone_assignment}.hpp`, `include/flp/flp_arc_index.hpp` | `test_search_infra`, `test_fuzz_search_infra` |
| N4, N5 | `operators_N4_N5.md` (A1-A6) | `neighbourhood_n4.hpp`, `neighbourhood_n5.hpp` | `test_n4_n5`, `test_n4_n5_oracle` |
| N7, N8 | `operators_N7_N8.md` (A3, A7) | `neighbourhood_n7.hpp`, `neighbourhood_n8.hpp` | `test_n7_n8`, `test_n7_n8_oracle` |
| N3, N6 | `operators_N3_N6.md` (A8-A12) | `neighbourhood_n3.hpp`, `neighbourhood_n6.hpp` | `test_n3_n6`, `test_n3_n6_oracle` |
| N1, N2 | `operators_N1_N2.md` (A13-A18) | `neighbourhood_n1.hpp`, `neighbourhood_n2.hpp` | `test_n1_n2`, `test_n1_n2_oracle` |
| Shakes | `shakes.md` (A19-A29) | `shakes.hpp`, `trip_exchange.hpp`, `trip_relocate.hpp` | `test_shakes`, `test_shakes_oracle` |
| VNS | `vns.md` (A30-A38) | `vns.hpp` | `test_vns`, `test_vns_oracle` |
| Configuration (48 keys) | `configuration_reference.md`, `runner_cli.md` section 2 | `include/config/experiment_config.hpp` | `test_experiment_config` |
| Runner, writer, CLI, exit codes | `runner_cli.md` (A39-A44) | `include/runner/*`, `apps/tdr_static.cpp` | `test_runner_cli` |
| Output schemas | `output_schema.md` | `include/runner/result_writer.hpp` | `test_runner_cli` (key sets pinned) |
| Experiment protocol | `experiment_methodology.md`, `validation_vs_paper.md` section 1 | `config/m9/*.cfg`, `scripts/m9_run_all.sh` | `m9_protocol_reproduces` |
| Validation vs the paper | `validation_vs_paper.md` (A45-A47) | `scripts/m9_analyse.py`, `m9_sensitivity.py`, `m9_exact_truck_only.py`, `bench/m9_feasibility_probe.cpp` | raw outputs `m9_*_raw.txt` |
| Cross-platform | `windows_validation.md` | - | Windows binaries under Wine |
| Test evidence per milestone | `test_and_validation.md` | `tests/`, `tests/mutation/` | - |

## 3. Where each class of statement lives
| Class | Location |
|---|---|
| Paper facts | `[PAPER]` tags in `math_spec.md` and the config header; paper extraction and planning documents in the project (outside the repository) |
| Approved decisions | `DECISIONS.md` (D1-D10 and numbered items), `MILESTONES.md`, `paper1_decision_register.md` addenda |
| Implementation assumptions | A1-A47 in the component documents above; `configuration_reference.md` (class A) |
| Deviations from the paper | `static_reproduction_summary.md` section 3; `DECISIONS.md` items 53-66 |
| Unavailable data | `validation_vs_paper.md` section 2 |
| Unresolved questions and unapproved proposals | `validation_vs_paper.md` section 8 (P1-P3, Q1); `static_reproduction_summary.md` sections 3 and 6; `DECISIONS.md` Stage-2 open items |

## 4. Raw outputs
`*_raw.txt` in this folder are verbatim command outputs referenced by the documents; `results/m9/` and `results/m9_sensitivity/` hold the experiment outputs of M9.
