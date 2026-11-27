// Fast exact drop-in for makefun-diff 1.0.0 (MIT).
//
// Same answer as makefun-diff 1.0.0 on every input, computed faster:
//   - lines are interned to integer ids once per case (the part string
//     itself is the Map key, so V8 hashes it once in C++ — hash cached in
//     the string header — instead of a JS hash loop over every code unit),
//     so the whole search compares ints; only the last line of a text can
//     lack its newline, so bare lines go through a tiny separate slot and
//     can never collide with a "\n" line;
//   - the common prefix / suffix are trimmed on ids;
//   - a windowed greedy script gives a valid upper bound Dupper on the
//     edit distance — the best of four greedy scripts (forward/reverse x
//     insertion-first/deletion-first), each a valid script, so the min is
//     a valid bound and the tightest of the four; Myers's greedy search
//     then runs only inside the band of diagonals that can still reach
//     both ends within Dupper (a path on diagonal k at depth d needs at
//     least |k - (N-M)| more edits, so a cell with d + |k - (N-M)| >
//     Dupper cannot lie on any shortest script and is skipped — the
//     reference's own first path to both ends is inside the band, so the
//     script is unchanged);
//   - the search runs over a COMPACT V-history: one Int32Array holding
//     only the band (one padded segment per depth, ~19 MB for the
//     practice cases) instead of the full d*d triangle (~78 MB, of which
//     only ~4.7M of ~19.6M cells are ever written — touching that span
//     costs ~20 ms per case in page faults). The pad slot on each side
//     reads as 0, exactly like an unwritten cell of the full layout, so
//     the search and the backtrack make the same choices; the full layout
//     is kept as the fallback for bounds above 6000. The search keeps
//     the reference's exact rule: furthest into the old text wins,
//     deletion wins a tie, no move past either text's end, no
//     edge-diagonal pruning, every equal run followed to its end; the
//     edit script is recovered by backtracking that history with the same
//     choice rule, so the script is the reference's first path to both
//     ends, not merely a shortest one;
//   - BIDIRECTIONAL Myers was measured and REJECTED (see the round's
//     notes): with the band already in place, a bidirectional search
//     visits 1.35-1.65x MORE cells than the forward search (the top level
//     alone is ~77% of the forward total, and the recursion re-searches
//     both halves — the geometric series sums to ~7.2M cells vs ~4.7M
//     forward on the practice set, even with the best-case even splits),
//     so it is slower, not faster. Myers's linear-space algorithm is a
//     space optimization, not a time one.
//   - the patch is built exactly as the reference builds it (3 kept lines
//     of context, runs at most 6 kept lines apart share a hunk, deletions
//     before insertions, "%" after a line with no newline);
//   - nothing is kept between cases: the interning state is reset at the
//     top of run(), and every other table is allocated fresh inside run(),
//     so each case is answered alone. If the band ever failed to reach
//     both ends (it cannot when the bound is valid), the search restarts
//     unbanded rather than answering wrong.
//
// The hot functions (tokenize, greedyUpper, myers) live at module scope so
// V8 keeps one optimized copy across cases instead of re-optimizing a
// fresh closure per case.

// ---- interning, module scope: V8 optimizes tokenize once, not once per ----
// ---- case (the closures inside run() were re-created and re-optimized for
// ---- every case). The part string itself is the Map key, so V8 hashes it
// ---- once in C++ (hash cached in the string header) instead of a JS FNV
// ---- loop over every code unit. Only the last line of a text can lack its
// ---- newline, so bare lines go through a tiny separate slot (at most two
// ---- per case) and can never collide with a "\n" line. State is reset at
// ---- the top of run(), so nothing is kept between cases.
const internMap = new Map();
let internNextId = 0;
let internBarePart = null;
let internBareId = -1;

// A text's lines: ids, and each line's [start, end) in the text (end just
// past the "\n" when there is one). The empty text has no lines.
function tokenize(s) {
  const parts = s.split('\n');
  const last = parts.length - 1;
  const n = last + (parts[last] !== '' ? 1 : 0);
  const ids = new Int32Array(n);
  const starts = new Int32Array(n);
  const ends = new Int32Array(n);
  let start = 0;
  for (let i = 0; i < last; i++) {
    const part = parts[i];
    let id = internMap.get(part);
    if (id === undefined) { id = internNextId++; internMap.set(part, id); }
    ids[i] = id;
    starts[i] = start;
    ends[i] = start + part.length + 1;
    start += part.length + 1;
  }
  if (parts[last] !== '') {
    const part = parts[last];
    let id;
    if (part === internBarePart) id = internBareId;
    else { id = internNextId++; internBarePart = part; internBareId = id; }
    ids[last] = id;
    starts[last] = start;
    ends[last] = start + part.length;
  }
  return { ids, starts, ends, text: s };
}

// ---- a windowed greedy script: its length bounds the edit distance ----
// Every line is consumed exactly once as a keep, a deletion or an
// insertion, so the count is the length of a valid script, hence >= D.
// A match is taken only when the following line confirms it, so repeated
// lines cannot drag the alignment far off. aMid/bMid are the middle's
// line ids, padded with sentinels (-1 / -2) that match nothing.
function greedyUpper(aMid, bMid, N, M) {
  const W = 64;
  let i = 0;
  let j = 0;
  let edits = 0;
  while (i < N && j < M) {
    if (aMid[i] === bMid[j]) { i++; j++; continue; }
    let k = -1;
    const lim = j + W < M - 1 ? j + W : M - 1;
    for (let t = j + 1; t <= lim; t++) {
      if (bMid[t] === aMid[i] && (i + 1 >= N || t + 1 >= M || aMid[i + 1] === bMid[t + 1])) { k = t; break; }
    }
    if (k !== -1) { edits += (k - j); j = k; continue; }
    let k2 = -1;
    const lim2 = i + W < N - 1 ? i + W : N - 1;
    for (let t = i + 1; t <= lim2; t++) {
      if (aMid[t] === bMid[j] && (t + 1 >= N || j + 1 >= M || aMid[t + 1] === bMid[j + 1])) { k2 = t; break; }
    }
    if (k2 !== -1) { edits += (k2 - i); i = k2; continue; }
    edits += 2;
    i++;
    j++;
  }
  edits += (N - i) + (M - j);
  return edits;
}

// Reverse greedy (same algorithm from the end) — a different valid script,
// sometimes tighter in repeated-run regions.
function greedyUpperRev(aMid, bMid, N, M) {
  const W = 64;
  let i = N - 1;
  let j = M - 1;
  let edits = 0;
  while (i >= 0 && j >= 0) {
    if (aMid[i] === bMid[j]) { i--; j--; continue; }
    let k = -1;
    const lim = j - W > 0 ? j - W : 0;
    for (let t = j - 1; t >= lim; t--) {
      if (bMid[t] === aMid[i] && (i - 1 < 0 || t - 1 < 0 || aMid[i - 1] === bMid[t - 1])) { k = t; break; }
    }
    if (k !== -1) { edits += (j - k); j = k; continue; }
    let k2 = -1;
    const lim2 = i - W > 0 ? i - W : 0;
    for (let t = i - 1; t >= lim2; t--) {
      if (aMid[t] === bMid[j] && (t - 1 < 0 || j - 1 < 0 || aMid[t - 1] === bMid[j - 1])) { k2 = t; break; }
    }
    if (k2 !== -1) { edits += (i - k2); i = k2; continue; }
    edits += 2;
    i--;
    j--;
  }
  edits += (i + 1) + (j + 1);
  return edits;
}

// Forward greedy, deletion scan first (opposite scan order).
function greedyUpperDel(aMid, bMid, N, M) {
  const W = 64;
  let i = 0;
  let j = 0;
  let edits = 0;
  while (i < N && j < M) {
    if (aMid[i] === bMid[j]) { i++; j++; continue; }
    let k2 = -1;
    const lim2 = i + W < N - 1 ? i + W : N - 1;
    for (let t = i + 1; t <= lim2; t++) {
      if (aMid[t] === bMid[j] && (t + 1 >= N || j + 1 >= M || aMid[t + 1] === bMid[j + 1])) { k2 = t; break; }
    }
    if (k2 !== -1) { edits += (k2 - i); i = k2; continue; }
    let k = -1;
    const lim = j + W < M - 1 ? j + W : M - 1;
    for (let t = j + 1; t <= lim; t++) {
      if (bMid[t] === aMid[i] && (i + 1 >= N || t + 1 >= M || aMid[i + 1] === bMid[t + 1])) { k = t; break; }
    }
    if (k !== -1) { edits += (k - j); j = k; continue; }
    edits += 2;
    i++;
    j++;
  }
  edits += (N - i) + (M - j);
  return edits;
}

// Reverse greedy, deletion scan first.
function greedyUpperRevDel(aMid, bMid, N, M) {
  const W = 64;
  let i = N - 1;
  let j = M - 1;
  let edits = 0;
  while (i >= 0 && j >= 0) {
    if (aMid[i] === bMid[j]) { i--; j--; continue; }
    let k2 = -1;
    const lim2 = i - W > 0 ? i - W : 0;
    for (let t = i - 1; t >= lim2; t--) {
      if (aMid[t] === bMid[j] && (t - 1 < 0 || j - 1 < 0 || aMid[t - 1] === bMid[j - 1])) { k2 = t; break; }
    }
    if (k2 !== -1) { edits += (i - k2); i = k2; continue; }
    let k = -1;
    const lim = j - W > 0 ? j - W : 0;
    for (let t = j - 1; t >= lim; t--) {
      if (bMid[t] === aMid[i] && (i - 1 < 0 || t - 1 < 0 || aMid[i - 1] === bMid[t - 1])) { k = t; break; }
    }
    if (k !== -1) { edits += (j - k); j = k; continue; }
    edits += 2;
    i--;
    j--;
  }
  edits += (i + 1) + (j + 1);
  return edits;
}

// The bound is the best of the four greedy scripts (forward/reverse x
// insertion-first/deletion-first); each is a valid script, so the min is
// a valid upper bound, and it is the tightest of the four.

// ---- Myers's forward search inside the band, or null when the band ----
// ---- is exhausted (the bound was wrong); the caller then retries ----
// ---- unbanded. V[d][k] is the furthest x on diagonal k with d edits,
// ---- after its snake; -1 when no path. One flat array, depth d at
// ---- offset d*d, diagonal k at +k+d.
function myers(aMid, bMid, N, M, Dupper) {
  const K0 = N - M;
  const pre = Math.min(Dupper, 6000) + 2;
  let cap = pre * pre;
  let V = new Int32Array(cap);
  let D = -1;
  for (let d = 0; D === -1; d++) {
    if (d > Dupper) return null;
    const need = (d + 1) * (d + 1);
    if (need > cap) {
      let nc = cap;
      while (nc < need) nc *= 2;
      const nV = new Int32Array(nc);
      nV.set(V);
      V = nV;
      cap = nc;
    }
    const base = d * d;
    const slack = Dupper - d;
    let klo = K0 - slack;
    if (klo < -d) klo = -d;
    let khi = K0 + slack;
    if (khi > d) khi = d;
    if (((klo - d) & 1) !== 0) klo++;
    if (d === 0) {
      let x = 0;
      let y = 0;
      while (aMid[x] === bMid[y]) { x++; y++; }
      V[0] = x;
      if (x === N && y === M) D = 0;
      continue;
    }
    const prevBase = (d - 1) * (d - 1);
    let k = klo;
    // k == -d: insertion only, from diagonal -d+1.
    if (k === -d) {
      const xp = V[prevBase];
      if (xp >= 0 && xp + d - 1 < M) {
        let x = xp;
        let y = x + d;
        while (aMid[x] === bMid[y]) { x++; y++; }
        V[base] = x;
        if (x === N && y === M) { D = d; break; }
      } else V[base] = -1;
      k += 2;
    }
    // Middle cells: both candidates live, indices in range.
    const kMidEnd = khi === d ? khi - 2 : khi;
    let ii = prevBase + k + d - 2;
    let iw = base + k + d;
    for (; k <= kMidEnd; k += 2) {
      const xm = V[ii];
      const xp = V[ii + 2];
      ii += 2;
      const canRemove = xm >= 0 && xm < N;
      const canAdd = xp >= 0 && xp - k - 1 < M;
      if (!canRemove && !canAdd) { V[iw] = -1; iw += 2; continue; }
      // Furthest into the old text wins; deletion wins a tie.
      let x;
      if (canRemove && (!canAdd || xm + 1 >= xp)) x = xm + 1;
      else x = xp;
      let y = x - k;
      while (aMid[x] === bMid[y]) { x++; y++; }
      V[iw] = x;
      iw += 2;
      if (k === K0 && x === N) { D = d; break; }
    }
    if (D !== -1) break;
    // k == d: deletion only, from diagonal d-1.
    if (khi === d) {
      const xm = V[prevBase + 2 * d - 2];
      if (xm >= 0 && xm < N) {
        let x = xm + 1;
        let y = x - d;
        while (aMid[x] === bMid[y]) { x++; y++; }
        V[base + 2 * d] = x;
        if (x === N && y === M) D = d;
      } else V[base + 2 * d] = -1;
    }
  }
  return D === -1 ? null : { D, V };
}

// ---- Compact banded Myers: the same search, but V stores only the band ----
// ---- (one padded segment per depth) instead of the full d*d triangle.
// ---- V index for (d,k) = offA[d] + 1 + (k - kloA[d]) / 2; the pad slot
// ---- on each side reads as 0, exactly like an unwritten cell of the
// ---- full layout. The full layout spans ~78 MB for the practice cases
// ---- (only ~4.7M of ~19.6M cells are ever written), and touching that
// ---- span costs ~20 ms per case in page faults; the compact layout spans
// ---- ~19 MB. Only used when Dupper <= 6000 (the practice/scored range);
// ---- larger bounds fall back to the full layout above.
function myersCompact(aMid, bMid, N, M, Dupper) {
  const K0 = N - M;
  const kloA = new Int32Array(Dupper + 2);
  const khiA = new Int32Array(Dupper + 2);
  const offA = new Int32Array(Dupper + 2);
  let total = 0;
  for (let d = 0; d <= Dupper; d++) {
    const slack = Dupper - d;
    let klo = K0 - slack;
    if (klo < -d) klo = -d;
    let khi = K0 + slack;
    if (khi > d) khi = d;
    if (((klo - d) & 1) !== 0) klo++;
    kloA[d] = klo;
    khiA[d] = khi;
    offA[d] = total;
    total += (khi - klo) / 2 + 1 + 2;
  }
  const V = new Int32Array(total);
  let D = -1;
  for (let d = 0; D === -1; d++) {
    if (d > Dupper) return null;
    const klo = kloA[d];
    const khi = khiA[d];
    const base = offA[d] + 1;
    if (d === 0) {
      let x = 0;
      let y = 0;
      while (aMid[x] === bMid[y]) { x++; y++; }
      V[base] = x;
      if (x === N && y === M) D = 0;
      continue;
    }
    const prevOff = offA[d - 1] + 1;
    const kloPrev = kloA[d - 1];
    let k = klo;
    // k == -d: insertion only, from diagonal -d+1.
    if (k === -d) {
      const xp = V[prevOff + ((-d + 1 - kloPrev) >> 1)];
      if (xp >= 0 && xp + d - 1 < M) {
        let x = xp;
        let y = x + d;
        while (aMid[x] === bMid[y]) { x++; y++; }
        V[base] = x;
        if (x === N && y === M) { D = d; break; }
      } else V[base] = -1;
      k += 2;
    }
    // Middle cells: both candidates live, indices in range.
    const kMidEnd = khi === d ? khi - 2 : khi;
    let ii = prevOff + ((k - 1 - kloPrev) >> 1);
    let iw = base + ((k - klo) >> 1);
    for (; k <= kMidEnd; k += 2) {
      const xm = V[ii];
      const xp = V[ii + 1];
      ii += 1;
      const canRemove = xm >= 0 && xm < N;
      const canAdd = xp >= 0 && xp - k - 1 < M;
      if (!canRemove && !canAdd) { V[iw] = -1; iw += 1; continue; }
      // Furthest into the old text wins; deletion wins a tie.
      let x;
      if (canRemove && (!canAdd || xm + 1 >= xp)) x = xm + 1;
      else x = xp;
      let y = x - k;
      while (aMid[x] === bMid[y]) { x++; y++; }
      V[iw] = x;
      iw += 1;
      if (k === K0 && x === N) { D = d; break; }
    }
    if (D !== -1) break;
    // k == d: deletion only, from diagonal d-1.
    if (khi === d) {
      const xm = V[prevOff + ((d - 1 - kloPrev) >> 1)];
      if (xm >= 0 && xm < N) {
        let x = xm + 1;
        let y = x - d;
        while (aMid[x] === bMid[y]) { x++; y++; }
        V[base + ((d - klo) >> 1)] = x;
        if (x === N && y === M) D = d;
      } else V[base + ((d - klo) >> 1)] = -1;
    }
  }
  return D === -1 ? null : { D, V, kloA, offA, compact: true };
}

/** makefun-diff 1.0.0's answer for one case's input. */
export function run(input) {
  if (input === null || typeof input !== 'object' || typeof input.old !== 'string' || typeof input.new !== 'string') {
    throw new TypeError('makefun-diff: the input is { old, new }, two strings');
  }
  const oldText = input.old;
  const newText = input.new;

  // ---- fresh interning state for this case (nothing kept between cases) ----
  internMap.clear();
  internNextId = 0;
  internBarePart = null;
  internBareId = -1;

  const A = tokenize(oldText);
  const B = tokenize(newText);
  const aIds = A.ids;
  const bIds = B.ids;
  const n = aIds.length;
  const m = bIds.length;

  // ---- common prefix, then common suffix of what is left ----
  let prefix = 0;
  while (prefix < n && prefix < m && aIds[prefix] === bIds[prefix]) prefix++;
  let suffix = 0;
  while (suffix < n - prefix && suffix < m - prefix && aIds[n - 1 - suffix] === bIds[m - 1 - suffix]) suffix++;
  const N = n - prefix - suffix;
  const M = m - prefix - suffix;
  const po = prefix;
  const qo = prefix;

  // ---- the middle's lines, padded with sentinels that match nothing ----
  // aMid[N] is -1, bMid[M] is -2, so a snake stops exactly at either end.
  const aMid = new Int32Array(N + 1);
  aMid.set(aIds.subarray(po, po + N));
  aMid[N] = -1;
  const bMid = new Int32Array(M + 1);
  bMid.set(bIds.subarray(qo, qo + M));
  bMid[M] = -2;

  // ---- the middle's runs: kind 0 keep, 1 delete, 2 insert (forward order) ----
  const rk = [];
  const rc = [];
  function pushRun(kind, count) {
    if (count <= 0) return;
    const L = rk.length;
    if (L > 0 && rk[L - 1] === kind) rc[L - 1] += count;
    else { rk.push(kind); rc.push(count); }
  }

  if (N === 0 && M === 0) {
    // nothing between the prefix and the suffix
  } else if (N === 0) {
    rk.push(2); rc.push(M);
  } else if (M === 0) {
    rk.push(1); rc.push(N);
  } else {
    const Dupper = Math.min(
      greedyUpper(aMid, bMid, N, M),
      greedyUpperRev(aMid, bMid, N, M),
      greedyUpperDel(aMid, bMid, N, M),
      greedyUpperRevDel(aMid, bMid, N, M),
    );
    let found = Dupper <= 6000
      ? myersCompact(aMid, bMid, N, M, Dupper)
      : myers(aMid, bMid, N, M, Dupper);
    if (found === null) found = myers(aMid, bMid, N, M, N + M);
    if (found === null) throw new Error('makefun-diff: the search ended without reaching both ends');
    const V = found.V;
    const D = found.D;
    const compact = !!found.compact;
    const kloA = found.kloA;
    const offA = found.offA;

    // Backtrack the same choices from (N, M) at depth D, collecting runs
    // in reverse, then flip them into forward order.
    const bk = [];
    const bc = [];
    function pushRev(kind, count) {
      if (count <= 0) return;
      const L = bk.length;
      if (L > 0 && bk[L - 1] === kind) bc[L - 1] += count;
      else { bk.push(kind); bc.push(count); }
    }
    let x = N;
    let y = M;
    let d = D;
    while (d > 0) {
      const k = x - y;
      let xm;
      let xp;
      if (compact) {
        const prevOff = offA[d - 1] + 1;
        const kloPrev = kloA[d - 1];
        xm = k > -d ? V[prevOff + ((k - 1 - kloPrev) >> 1)] : -1;
        xp = k < d ? V[prevOff + ((k + 1 - kloPrev) >> 1)] : -1;
      } else {
        const prevBase = (d - 1) * (d - 1);
        xm = k > -d ? V[prevBase + k + d - 2] : -1;
        xp = k < d ? V[prevBase + k + d] : -1;
      }
      const canRemove = xm >= 0 && xm < N;
      const canAdd = xp >= 0 && xp - k - 1 < M;
      let del;
      if (canRemove && (!canAdd || xm + 1 >= xp)) del = true;
      else if (canAdd) del = false;
      else throw new Error('makefun-diff: backtrack found no predecessor');
      let x0;
      let y0;
      if (del) { x0 = xm + 1; y0 = xm - (k - 1); }
      else { x0 = xp; y0 = xp - (k + 1) + 1; }
      let s = 0;
      while (x > x0 && y > y0) { x--; y--; s++; }
      pushRev(0, s);
      if (del) { pushRev(1, 1); x--; }
      else { pushRev(2, 1); y--; }
      d--;
    }
    pushRev(0, x);
    for (let i = bk.length - 1; i >= 0; i--) pushRun(bk[i], bc[i]);
  }

  // ---- full runs: prefix keep, middle, suffix keep ----
  const frk = [];
  const frc = [];
  function pushFull(kind, count) {
    if (count <= 0) return;
    const L = frk.length;
    if (L > 0 && frk[L - 1] === kind) frc[L - 1] += count;
    else { frk.push(kind); frc.push(count); }
  }
  if (prefix > 0) pushFull(0, prefix);
  for (let i = 0; i < rk.length; i++) pushFull(rk[i], rc[i]);
  if (suffix > 0) pushFull(0, suffix);

  // ---- change runs: each a stretch of deletions and insertions ----
  const cO = [];
  const cP = [];
  const cD = [];
  const cI = [];
  let o = 0;
  let p = 0;
  let deleted = 0;
  let inserted = 0;
  for (let i = 0; i < frk.length; i++) {
    const kind = frk[i];
    const count = frc[i];
    if (kind === 0) { o += count; p += count; continue; }
    const L = cO.length;
    let ci = L - 1;
    if (ci < 0 || cO[ci] + cD[ci] !== o || cP[ci] + cI[ci] !== p) {
      cO.push(o); cP.push(p); cD.push(0); cI.push(0);
      ci = L;
    }
    if (kind === 1) { cD[ci] += count; o += count; deleted += count; }
    else { cI[ci] += count; p += count; inserted += count; }
  }

  // ---- hunks: change runs at most 6 kept lines apart share one ----
  const hStart = [];
  const hEnd = [];
  for (let i = 0; i < cO.length; i++) {
    const L = hStart.length;
    if (L > 0) {
      const prev = hEnd[L - 1] - 1;
      if (cO[i] - (cO[prev] + cD[prev]) <= 6) { hEnd[L - 1] = i + 1; continue; }
    }
    hStart.push(i);
    hEnd.push(i + 1);
  }

  // ---- the patch ----
  const out = [`makefun-diff/1 old=${n} new=${m} hunks=${hStart.length} deleted=${deleted} inserted=${inserted}\n`];
  function pushWritten(op, T, idx) {
    const s = T.text;
    const st = T.starts[idx];
    const en = T.ends[idx];
    if (s.charCodeAt(en - 1) === 10) out.push(op + s.slice(st, en));
    else out.push(op + s.slice(st, en) + '\n%\n');
  }
  for (let hi = 0; hi < hStart.length; hi++) {
    const first = hStart[hi];
    const last = hEnd[hi] - 1;
    const oldStart = Math.max(0, cO[first] - 3);
    const newStart = cP[first] - (cO[first] - oldStart);
    const after = Math.min(3, n - (cO[last] + cD[last]));
    const oldEnd = cO[last] + cD[last] + after;
    const newEnd = cP[last] + cI[last] + after;
    out.push(`@ ${oldStart + 1} ${oldEnd - oldStart} ${newStart + 1} ${newEnd - newStart}\n`);
    let at = oldStart;
    for (let ci = first; ci <= last; ci++) {
      for (; at < cO[ci]; at++) pushWritten('=', A, at);
      for (let j = 0; j < cD[ci]; j++) pushWritten('-', A, cO[ci] + j);
      for (let j = 0; j < cI[ci]; j++) pushWritten('+', B, cP[ci] + j);
      at = cO[ci] + cD[ci];
    }
    for (; at < oldEnd; at++) pushWritten('=', A, at);
  }
  return out.join('');
}
