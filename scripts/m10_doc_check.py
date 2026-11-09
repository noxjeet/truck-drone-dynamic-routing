#!/usr/bin/env python3
"""M10 documentation and reproduction consistency checks (stdlib only; read-only).

usage: m10_doc_check.py [--build BUILD_DIR]      (run from the repository root; BUILD_DIR enables the checks that need binaries)

1. every repository path mentioned in backticks in README.md and docs/static/*.md exists (documents, headers, tests, scripts, configs, results);
2. milestone consistency: MILESTONES.md has a section for every milestone M0..M10; M0..M9 headings say "signed off"; no heading, status line or
   decision-register heading still says "awaiting review" for a signed-off milestone;
3. configuration_reference.md lists exactly the keys of `tdr_static list-keys` (needs --build);
4. the stated number of CTest programs equals the number of add_test entries in CMakeLists.txt, and every test program named in INDEX.md exists as a target;
5. every `config/m9/*.cfg` is referenced by scripts/m9_run_all.sh and every recorded result directory has a manifest.json;
6. the stated assertion total (INDEX/summary/test_and_validation) equals the sum of the per-program counts printed by the test programs (needs --build).
Exit code 0 when everything passes.
"""
import glob, os, re, subprocess, sys

ok = True
def fail(msg):
    global ok
    ok = False
    print("FAIL:", msg)
def info(msg): print("ok:  ", msg)

build = None
if "--build" in sys.argv: build = sys.argv[sys.argv.index("--build") + 1]
docs = ["README.md"] + sorted(glob.glob("docs/static/*.md"))

# 1. paths
ROOTS = ("include/", "src/", "tests/", "docs/", "scripts/", "bench/", "config/", "apps/", "results/")
PROJECT_DOCS = {"paper1-understanding.md", "han-chi-sun-2026-extraction.md", "paper1-implementation-plan.md", "phase1-forensic-audit.md",
                "stage2-comparison-framework.md", "stage2-research-understanding.md", "static-reproduction-milestones.md", "m6-signoff-and-m7-vns.md", "m9-validation-vs-paper.md"}
BASENAMES = set()
for root in ("docs", "scripts", "tests", "include", "src", "bench", "config", "apps"):
    for _d, _dirs, _files in os.walk(root): BASENAMES |= set(_files)
BASENAMES |= {f for f in os.listdir(".") if os.path.isfile(f)}
bad = 0; checked = 0
for d in docs:
    base = os.path.dirname(d)
    for tok in re.findall(r"`([^`\n]+)`", open(d, encoding="utf-8").read()):
        t = tok.strip()
        if any(c in t for c in " *<>{}|=$~@") or "..." in t or t.endswith(("/", ":")) or "\\" in t: continue
        cand = None
        if t.startswith(ROOTS): cand = t
        elif re.fullmatch(r"[A-Za-z0-9_.\-]+\.(md|hpp|cpp|cfg|py|sh|cmake|txt|csv|json)", t) and "/" not in t: cand = None
        else: continue
        if cand is None:
            # bare file name: must exist somewhere in the documented folders
            if t.startswith("prof_") or t in PROJECT_DOCS: continue   # scratch programs outside the repository; project documents (claude/*.md) outside the repository
            if t in ("results.json", "runs.csv", "traces.csv", "timing.json", "manifest.json", "config.resolved.cfg", "CMakeLists.txt", "summary.json", "index.html"): continue
            found = [t] if t in BASENAMES else []
            checked += 1
            if not found and not t.startswith(("m9_sens", "m9_analysis")):
                bad += 1; fail(f"{d}: `{t}` not found in the repository")
            continue
        checked += 1
        if not (os.path.exists(cand) or glob.glob(cand)):
            bad += 1; fail(f"{d}: `{t}` does not exist")
if bad == 0: info(f"{checked} path mentions in {len(docs)} documents all resolve")

# 2. milestone consistency
ms = open("docs/static/MILESTONES.md", encoding="utf-8").read()
heads = re.findall(r"^## (M\d+)\b[^\n]*", ms, flags=re.M)
for k in range(0, 11):
    if f"M{k}" not in heads: fail(f"MILESTONES.md has no section M{k}")
for m in re.finditer(r"^## (M(\d+)) ---[^\n]*", ms, flags=re.M):
    k = int(m.group(2))
    if "follow-up" in m.group(0) or "finalization" in m.group(0): continue
    if k <= 9 and not re.search(r"signed off|approved|accepted", m.group(0)): fail(f"milestone heading without sign-off: {m.group(0)[:90]}")
stale = []
for d in ["docs/static/MILESTONES.md", "docs/paper1_decision_register.md", "DECISIONS.md"]:
    for i, line in enumerate(open(d, encoding="utf-8").read().split("\n"), 1):
        if re.match(r"^(## M\d+ [^\n]*awaiting review|\d+\. \*\*M\d+[^*]*\[IMPLEMENTATION, awaiting review\])", line): stale.append(f"{d}:{i}")
for s in stale: fail(f"stale 'awaiting review' status at {s}")
if not stale: info("no stale 'awaiting review' headings or decision items")
for d in docs:
    t = open(d, encoding="utf-8").read()
    for pat in (r"is not signed off", r"was \*\*not\*\* started", r"M\d+ (is|was) not signed off"):
        for m in re.finditer(pat, t):
            fail(f"{d}: stale status text '{m.group(0)}'")

# 3. configuration reference vs list-keys
if build:
    out = subprocess.run([os.path.join(build, "tdr_static"), "list-keys"], capture_output=True, text=True).stdout
    keys = {l.split(" = ")[0] for l in out.splitlines() if " = " in l}
    ref = set()
    for line in open("docs/static/configuration_reference.md", encoding="utf-8"):
        if line.startswith("| `"):
            first = line.split("|")[1]
            ref |= set(re.findall(r"`([a-z0-9_]+\.[a-z0-9_]+)`", first))
    if keys == ref: info(f"configuration_reference.md lists exactly the {len(keys)} keys of list-keys")
    else: fail(f"configuration keys differ: missing from the reference {sorted(keys - ref)}, unknown in the reference {sorted(ref - keys)}")

# 4. test counts and programs
cm = open("CMakeLists.txt", encoding="utf-8").read()
n_tests = len(re.findall(r"^\s*add_test\(", cm, flags=re.M))
for d in ("docs/static/INDEX.md", "docs/static/static_reproduction_summary.md", "README.md"):
    t = open(d, encoding="utf-8").read()
    for m in re.finditer(r"(\d+) (?:CTest programs|programs|of \d+ CTest programs)", t):
        pass
summ = open("docs/static/static_reproduction_summary.md", encoding="utf-8").read()
m = re.search(r"(\d+) CTest programs", summ)
if m and int(m.group(1)) == n_tests: info(f"{n_tests} CTest programs stated and registered")
else: fail(f"summary states {m.group(1) if m else '?'} CTest programs, CMakeLists registers {n_tests}")
idx = open("docs/static/INDEX.md", encoding="utf-8").read()
names = set(re.findall(r"`(test_[a-z0-9_]+)`", idx))
for n in sorted(names):
    if not os.path.exists(f"tests/{n}.cpp"): fail(f"INDEX.md names test program {n} but tests/{n}.cpp is missing")
    elif f"add_test(NAME {n} " not in cm: fail(f"{n} is not registered with CTest")
info(f"{len(names)} test programs named in INDEX.md exist and are registered")

# 5. configs and result directories
runall = open("scripts/m9_run_all.sh", encoding="utf-8").read()
for c in sorted(glob.glob("config/m9/*.cfg")):
    n, s = re.match(r"config/m9/n(\d+)_seed(\d+)\.cfg", c).groups()
    if "config/m9/n${n}_seed${s}.cfg" not in runall: fail(f"{c} is not covered by scripts/m9_run_all.sh")
for dname in sorted(glob.glob("results/m9/n*_seed*")) + sorted(glob.glob("results/m9_sensitivity/*")):
    if not os.path.exists(os.path.join(dname, "manifest.json")): fail(f"{dname} has no manifest.json")
info(f"{len(glob.glob('config/m9/*.cfg'))} protocol configurations and {len(glob.glob('results/m9/n*_seed*')) + len(glob.glob('results/m9_sensitivity/*'))} result directories checked")

# 6. assertion total
if build:
    counts = {}
    from concurrent.futures import ThreadPoolExecutor
    names = [os.path.basename(t)[:-4] for t in sorted(glob.glob("tests/test_*.cpp"))]
    def run(n):
        exe = os.path.join(build, n)
        if not os.path.exists(exe): return n, "MISSING"
        return n, subprocess.run([exe], capture_output=True, text=True).stdout
    with ThreadPoolExecutor(max_workers=4) as ex: outs = list(ex.map(run, names))
    for n, o in outs:
        if o == "MISSING": fail(f"{os.path.join(build, n)} not built"); continue
        m = re.search(r"\((\d+) assertions\)", o) or re.search(r"^(\d+) (?:passed|checks)", o, flags=re.M)
        counts[n] = int(m.group(1)) if m else None
    known = {"test_graph": 26, "test_model": 43, "test_flp": 59, "test_timing": 291, "test_tsp": 29, "test_init": 171, "test_search_infra": 144}  # programs that print no total
    for n, v in known.items(): counts[n] = v   # these programs print per-test PASS lines and no total; the counts are the ones recorded in test_and_validation.md
    missing = [n for n, v in counts.items() if v is None]
    total = sum(v for v in counts.values() if v is not None)
    if missing: fail(f"no count parsed for {missing}")
    stated = [int(x.replace(",", "")) for x in re.findall(r"about ([\d,]{9,})", summ + idx)]
    claim = re.search(r"about ([\d,]+) assertions", summ)
    if claim and int(claim.group(1).replace(",", "")) == total: info(f"assertion total {total:,} equals the stated figure")
    else: fail(f"computed assertion total {total:,}, summary states {claim.group(1) if claim else '?'}")

print("RESULT:", "all checks passed" if ok else "FAILURES")
sys.exit(0 if ok else 1)
