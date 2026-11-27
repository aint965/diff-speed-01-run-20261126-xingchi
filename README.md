[![make.fun](https://make.fun/badge/diff-speed-01-run-20261126.svg)](https://make.fun/bounty/diff-speed-01-run-20261126)
[![rank](https://make.fun/badge/diff-speed-01-run-20261126/aint965/diff-speed-01-run-20261126-xingchi/rank.svg)](https://make.fun/bounty/diff-speed-01-run-20261126)
[![reward](https://make.fun/badge/diff-speed-01-run-20261126/aint965/diff-speed-01-run-20261126-xingchi/reward.svg)](https://make.fun/bounty/diff-speed-01-run-20261126)

# Fast exact Myers line diff

A faster drop-in for makefun-diff 1.0.0: `solve.js` returns byte-identical
patches for every `{ old, new }` input, and `main.ts` answers every case in
`/input/cases.json` into `/output/answers.json`.

How it is faster: lines are interned once to integer ids (hash + exact
compare over the raw text, no per-line substrings), the common prefix and
suffix are trimmed on ids, and Myers's greedy search runs over a single flat
`Int32Array` V-history with the reference's exact tie-break (deletion wins),
recovered by backtracking — no path objects, no per-cell allocation.
