#!/bin/bash
# [DM8] Regenerates the frozen PRE-HOOK copy of include/search/*.hpp from the tag static-baseline-m10 (verbatim except the namespace and include
# prefix, exactly like tests/reference/timing_engine_reference.hpp) and VERIFIES it: the inverse substitution must reproduce `git show <tag>:<file>`
# byte for byte. Usage: tests/reference/search_pre_hook/generate.sh [--check]
set -e
TAG=static-baseline-m10
ROOT=$(git rev-parse --show-toplevel)
DST=$ROOT/tests/reference/search_pre_hook
FILES=$(git ls-tree --name-only $TAG include/search/ | sed 's|include/search/||')
rc=0
for f in $FILES; do
  git show $TAG:include/search/$f | sed -e 's/tdr::search\b/tdr::search_ref/g' -e 's|#include "search/|#include "search_pre_hook/|' > /tmp/sph_$f
  if [ "$1" != "--check" ]; then cp /tmp/sph_$f $DST/$f; fi
  # inverse of the (committed) copy must equal the tagged original
  sed -e 's/tdr::search_ref/tdr::search/g' -e 's|#include "search_pre_hook/|#include "search/|' $DST/$f | cmp -s - <(git show $TAG:include/search/$f) || { echo "MISMATCH $f"; rc=1; }
  cmp -s /tmp/sph_$f $DST/$f || { echo "STALE $f"; rc=1; }
  rm -f /tmp/sph_$f
done
[ $rc = 0 ] && echo "search_pre_hook: $(echo $FILES | wc -w) files verbatim from $TAG (namespace tdr::search_ref)"
exit $rc
