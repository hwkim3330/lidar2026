/* ═══════════════════════════════════════════════
   ilp-core.js — TSN/GCL ILP Solver Engine + Visualizations
   Shared ES module for solver.html & automotive.html
   ═══════════════════════════════════════════════ */

/* ── Color Mapping ─────────────────────────────── */
const FLOW_COLORS_HEX = {
  control: "#3B82F6", sensor: "#0d9488", video: "#db2777",
  camera: "#3B82F6", lidar: "#0d9488", radar: "#d97706",
  brake: "#dc2626", powertrain: "#7c3aed", infotainment: "#db2777"
};

export function flowColor(flowId) {
  const id = flowId.toLowerCase();
  if (id.includes("ctrl") || id.includes("camera")) return FLOW_COLORS_HEX.control;
  if (id.includes("sensor") || id.includes("lidar")) return FLOW_COLORS_HEX.sensor;
  if (id.includes("video") || id.includes("infotain")) return FLOW_COLORS_HEX.video;
  if (id.includes("radar")) return FLOW_COLORS_HEX.radar;
  if (id.includes("brake")) return FLOW_COLORS_HEX.brake;
  if (id.includes("powertrain") || id.includes("pwr")) return FLOW_COLORS_HEX.powertrain;
  return "#7b61ff";
}

export function flowType(flowId) {
  const id = flowId.toLowerCase();
  if (id.includes("ctrl") || id.includes("camera")) return "control";
  if (id.includes("sensor") || id.includes("lidar")) return "sensor";
  if (id.includes("video") || id.includes("infotain")) return "video";
  if (id.includes("radar")) return "radar";
  if (id.includes("brake")) return "brake";
  if (id.includes("powertrain") || id.includes("pwr")) return "powertrain";
  return "unknown";
}

export function getFlowColorsHex() { return FLOW_COLORS_HEX; }

/* ── Tooltip ───────────────────────────────────── */
let tooltipEl = null;

export function initTooltip() {
  tooltipEl = document.getElementById("tooltip");
}

export function showTip(evt, html) {
  if (!tooltipEl) return;
  tooltipEl.innerHTML = html;
  tooltipEl.classList.add("show");
  const x = Math.min(evt.clientX + 14, window.innerWidth - 300);
  const y = Math.min(evt.clientY - 10, window.innerHeight - 200);
  tooltipEl.style.left = x + "px";
  tooltipEl.style.top = y + "px";
}

export function hideTip() {
  if (tooltipEl) tooltipEl.classList.remove("show");
}

/* ── ILP Solver Helpers ────────────────────────── */
function round3(v) { return Math.round(v * 1000) / 1000; }
function txTimeUs(bytes, mbps) { return ((bytes + 38) * 8) / mbps; }
function isTsn(pri, dl) { return pri >= 6 || dl !== null; }
function gateMask(p) { const b = Array(8).fill('0'); b[7 - Math.max(0, Math.min(7, p))] = '1'; return b.join(''); }

export function generateKPaths(adj, src, dst, k, maxD) {
  const found = [];
  (function dfs(n, d, vis, path) {
    if (found.length >= 2000 || d > maxD) return;
    if (n === dst) { found.push(path.slice()); return; }
    for (const e of (adj.get(n) || [])) {
      if (vis.has(e.to)) continue;
      vis.add(e.to); path.push(e.lid);
      dfs(e.to, d + 1, vis, path);
      path.pop(); vis.delete(e.to);
    }
  })(src, 0, new Set([src]), []);
  found.sort((a, b) => a.length - b.length || a.join('|').localeCompare(b.join('|')));
  const u = [], s = new Set();
  for (const p of found) { const k2 = p.join('>'); if (!s.has(k2)) { s.add(k2); u.push(p); } if (u.length >= k) break; }
  return u;
}

export function expandPackets(model) {
  const lm = new Map(model.links.map(l => [l.id, l]));
  const adj = new Map();
  for (const l of model.links) { if (!adj.has(l.from)) adj.set(l.from, []); adj.get(l.from).push({ to: l.to, lid: l.id }); }
  const pkts = [];
  for (const f of model.flows) {
    if (!f.period_us || f.period_us <= 0) throw new Error(`flow ${f.id}: period_us must be > 0`);
    let cp = f.candidate_paths || (f.path ? [f.path] : null);
    if (!cp && f.src && f.dst) {
      cp = generateKPaths(adj, f.src, f.dst, Math.max(1, f.k_paths || 2), model.nodes.length + 2);
      if (!cp.length) throw new Error(`No route: ${f.src}->${f.dst}`);
    }
    if (!cp || !Array.isArray(cp) || cp.length === 0) throw new Error(`flow ${f.id}: set candidate_paths/path OR src+dst`);
    for (const p of cp) {
      if (!Array.isArray(p) || p.length === 0) throw new Error(`flow ${f.id}: empty path`);
      for (const lid of p) if (!lm.has(lid)) throw new Error(`flow ${f.id}: unknown link ${lid}`);
    }
    const reps = Math.max(1, Math.ceil(model.cycle_time_us / f.period_us));
    for (let k = 0; k < reps; k++) {
      const rel = k * f.period_us;
      pkts.push({
        pid: `${f.id}#${k}`, fid: f.id, pri: f.priority, tt: f.traffic_type,
        rel, dl: f.deadline_us == null ? null : rel + f.deadline_us,
        tsn: isTsn(f.priority, f.deadline_us),
        routes: cp.map((pl, ri) => ({ ri, hops: pl.map(lid => ({ lid, tx: txTimeUs(f.payload_bytes, lm.get(lid).rate_mbps), pd: lm.get(lid).prop_delay_us })) }))
      });
    }
  }
  return pkts;
}

/* ── Build GCL result from scheduled packets ── */
function buildResult(model, pkts, schedHops, method, stats) {
  const linkRows = Object.fromEntries(model.links.map(l => [l.id, []]));
  const pktRows = [];
  for (let p = 0; p < pkts.length; p++) {
    const pk = pkts[p], sh = schedHops[p];
    const selR = sh.route, rt = pk.routes[selR], hops = [];
    for (let h = 0; h < rt.hops.length; h++) {
      const hp = rt.hops[h], s = sh.starts[h], e = round3(s + hp.tx);
      hops.push({ link_id: hp.lid, start_us: round3(s), end_us: e, duration_us: round3(hp.tx) });
      linkRows[hp.lid].push({ type: 'flow', note: pk.pid, priority: pk.pri, start_us: round3(s), end_us: e, duration_us: round3(hp.tx) });
      if (pk.tsn) linkRows[hp.lid].push({ type: 'guard', note: 'guard band', priority: pk.pri, start_us: e, end_us: round3(e + model.guard_band_us), duration_us: round3(model.guard_band_us) });
    }
    const lH = rt.hops.at(-1), fin = hops.at(-1).end_us + lH.pd, e2e = round3(fin - pk.rel);
    const ok = pk.dl == null || fin <= pk.dl + 1e-6;
    pktRows.push({ packet_id: pk.pid, flow_id: pk.fid, priority: pk.pri, selected_route: selR, release_us: round3(pk.rel), end_us: round3(fin), e2e_delay_us: e2e, deadline_abs_us: pk.dl == null ? null : round3(pk.dl), slack_us: pk.dl == null ? null : round3(pk.dl - fin), status: pk.dl == null ? 'BE' : ok ? 'OK' : 'MISS', hops });
  }
  pktRows.sort((a, b) => a.end_us - b.end_us);

  const noBe = !!model.no_be;
  const gcl = { cycle_time_us: model.cycle_time_us, base_time_us: 0, links: {} };
  for (const lnk of model.links) {
    const rows = linkRows[lnk.id].slice().sort((a, b) => a.start_us - b.start_us);
    const entries = []; let cur = 0, idx = 0;
    for (const r of rows) {
      if (r.start_us > cur && !noBe) entries.push({ index: idx++, gate_mask: '00001111', start_us: round3(cur), end_us: round3(r.start_us), duration_us: round3(r.start_us - cur), note: 'best-effort gap' });
      else if (r.start_us > cur && noBe) entries.push({ index: idx++, gate_mask: '00000000', start_us: round3(cur), end_us: round3(r.start_us), duration_us: round3(r.start_us - cur), note: 'all-gates-closed gap' });
      entries.push({ index: idx++, gate_mask: r.type === 'guard' ? '00000000' : gateMask(r.priority), start_us: r.start_us, end_us: r.end_us, duration_us: r.duration_us, note: r.note });
      cur = Math.max(cur, r.end_us);
    }
    if (cur < model.cycle_time_us && !noBe) entries.push({ index: idx++, gate_mask: '00001111', start_us: round3(cur), end_us: round3(model.cycle_time_us), duration_us: round3(model.cycle_time_us - cur), note: 'best-effort remainder' });
    else if (cur < model.cycle_time_us && noBe) entries.push({ index: idx++, gate_mask: '00000000', start_us: round3(cur), end_us: round3(model.cycle_time_us), duration_us: round3(model.cycle_time_us - cur), note: 'all-gates-closed remainder' });
    gcl.links[lnk.id] = { from: lnk.from, to: lnk.to, entries };
  }

  let worstUtil = 0;
  for (const lnk of model.links) {
    let act = 0; for (const e of gcl.links[lnk.id].entries) if (!e.note.includes('best-effort') && !e.note.includes('all-gates-closed')) act += e.duration_us;
    worstUtil = Math.max(worstUtil, act / model.cycle_time_us * 100);
  }

  // Structural validity check independent from deadline checks.
  let overlapConflicts = 0;
  for (const lnk of model.links) {
    const rows = linkRows[lnk.id].slice().sort((a, b) => a.start_us - b.start_us);
    for (let i = 1; i < rows.length; i++) {
      if (rows[i].start_us < rows[i - 1].end_us - 1e-9) overlapConflicts++;
    }
  }
  const outStats = { ...(stats || {}), overlap_conflicts: overlapConflicts };

  // Generate per-switch board configs (max 8 entries per port for LAN9662/LAN9692)
  const MAX_GCL_ENTRIES = 8;
  const switchNodes = model.nodes.filter(n => n.type === 'switch').map(n => n.id);
  let boardConfigs = null;
  if (switchNodes.length > 0) {
    boardConfigs = {};
    for (const swId of switchNodes) {
      const egressLinks = model.links.filter(l => l.from === swId);
      const ports = {};
      for (const el of egressLinks) {
        const rawEntries = gcl.links[el.id]?.entries || [];
        // Compact entries: remove guard bands, merge consecutive same-mask,
        // then merge smallest gaps to fit within MAX_GCL_ENTRIES
        let compact = [];
        for (const e of rawEntries) {
          if (e.note.includes('guard')) continue; // skip guard bands
          if (compact.length > 0 && compact[compact.length - 1].gate_mask === e.gate_mask) {
            // merge consecutive same-mask entries
            const prev = compact[compact.length - 1];
            prev.end_us = e.end_us;
            prev.duration_us = round3(prev.end_us - prev.start_us);
            prev.note += ' + ' + e.note;
          } else {
            compact.push({ ...e });
          }
        }
        // If still too many, merge the smallest closed/idle entries
        while (compact.length > MAX_GCL_ENTRIES) {
          // Find smallest entry that is idle/closed/gap
          let minIdx = -1, minDur = Infinity;
          for (let i = 0; i < compact.length; i++) {
            if ((compact[i].gate_mask === '00000000' || compact[i].note.includes('gap') || compact[i].note.includes('closed'))
                && compact[i].duration_us < minDur && i > 0) {
              minDur = compact[i].duration_us; minIdx = i;
            }
          }
          if (minIdx < 0) {
            // Merge last two entries as fallback
            minIdx = compact.length - 1;
          }
          // Merge with previous
          if (minIdx > 0) {
            const prev = compact[minIdx - 1];
            const cur = compact[minIdx];
            prev.end_us = cur.end_us;
            prev.duration_us = round3(prev.end_us - prev.start_us);
            compact.splice(minIdx, 1);
          } else {
            compact.splice(minIdx, 1);
          }
        }
        // Re-index
        compact.forEach((e, i) => { e.index = i; });

        // Collect PCPs used on this port
        const pcpsUsed = new Set();
        for (const e of rawEntries) {
          if (e.note && !e.note.includes('best-effort') && !e.note.includes('guard') && !e.note.includes('all-gates-closed')) {
            const matchedRow = linkRows[el.id]?.find(r => r.note === e.note && r.type === 'flow');
            if (matchedRow) pcpsUsed.add(matchedRow.priority);
          }
        }
        const pcpToQueue = {};
        for (const p of pcpsUsed) pcpToQueue[p] = p;
        ports[el.id] = {
          to: el.to,
          rate_mbps: el.rate_mbps,
          entries: compact,
          pcps_used: [...pcpsUsed].sort((a, b) => b - a),
          pcp_to_queue: pcpToQueue
        };
      }
      boardConfigs[swId] = {
        ports,
        cycle_time_us: model.cycle_time_us,
        guard_band_us: model.guard_band_us,
        no_be: noBe
      };
    }
  }

  return {
    method,
    objective: round3(pktRows.filter(p => p.status !== 'BE').reduce((a, p) => a + p.e2e_delay_us, 0)),
    worst_util_percent: round3(worstUtil), packetRows: pktRows, gcl, stats: outStats,
    boardConfigs
  };
}

/* ═══════════════════════════════════════════════
   GREEDY SCHEDULER — Priority-based list scheduler
   No GLPK dependency, runs in <1ms
   ═══════════════════════════════════════════════ */
export function solveGreedy(model) {
  if (!model.processing_delay_us) model.processing_delay_us = 3;
  if (!model.guard_band_us) model.guard_band_us = 3;
  const t0 = performance.now();
  const pkts = expandPackets(model);
  let fallbackCount = 0;

  // Link occupancy tracker: for each link, list of [start, end] intervals
  const linkOcc = Object.fromEntries(model.links.map(l => [l.id, []]));

  function findEarliest(lid, earliest, duration, guard) {
    const occ = linkOcc[lid];
    let t = earliest;
    const total = duration + guard;
    const margin = model.tx_margin_us || 0;
    while (true) {
      let moved = false;
      for (const [s, e] of occ) {
        if (t < e + margin && t + total + margin > s) { t = e + margin; moved = true; break; }
      }
      if (!moved) return t;
    }
  }

  // Sort: highest priority first, then by release time, then by deadline
  const order = pkts.map((pk, i) => i);
  order.sort((a, b) => {
    const pa = pkts[a], pb = pkts[b];
    if (pa.pri !== pb.pri) return pb.pri - pa.pri; // higher priority first
    if (pa.rel !== pb.rel) return pa.rel - pb.rel;
    const da = pa.dl ?? Infinity, db = pb.dl ?? Infinity;
    return da - db;
  });

  const schedHops = new Array(pkts.length);

  for (const pi of order) {
    const pk = pkts[pi];
    let bestRoute = 0, bestEnd = Infinity, bestStarts = null;

    // Try each candidate route
    for (let ri = 0; ri < pk.routes.length; ri++) {
      const rt = pk.routes[ri];
      const starts = [];
      let t = pk.rel;
      let valid = true;

      for (let h = 0; h < rt.hops.length; h++) {
        const hp = rt.hops[h];
        const guard = pk.tsn ? model.guard_band_us : 0;
        const s = findEarliest(hp.lid, t, hp.tx, guard);
        if (s + hp.tx > model.cycle_time_us) { valid = false; break; }
        starts.push(s);
        t = s + hp.tx + hp.pd + model.processing_delay_us;
      }

      if (valid) {
        const lastH = rt.hops.at(-1);
        const fin = starts.at(-1) + lastH.tx + lastH.pd;
        if (fin < bestEnd) { bestEnd = fin; bestRoute = ri; bestStarts = starts; }
      }
    }

    if (!bestStarts) {
      // Fallback: just place at release time (may overlap)
      const rt = pk.routes[0];
      bestStarts = []; let t = pk.rel;
      for (const hp of rt.hops) { bestStarts.push(t); t += hp.tx + hp.pd + model.processing_delay_us; }
      bestRoute = 0;
      fallbackCount++;
    }

    // Commit to link occupancy
    const rt = pk.routes[bestRoute];
    for (let h = 0; h < rt.hops.length; h++) {
      const hp = rt.hops[h], guard = pk.tsn ? model.guard_band_us : 0;
      linkOcc[hp.lid].push([bestStarts[h], bestStarts[h] + hp.tx + guard]);
    }
    // Keep occupancy sorted for efficient conflict checking
    for (let h = 0; h < rt.hops.length; h++) {
      linkOcc[rt.hops[h].lid].sort((a, b) => a[0] - b[0]);
    }

    schedHops[pi] = { route: bestRoute, starts: bestStarts };
  }

  const elapsed = Math.round(performance.now() - t0);
  return buildResult(model, pkts, schedHops, 'Greedy (priority-based list scheduler)', {
    constraints: 0, variables: 0, binaries: 0, status: 'heuristic', runtime_ms: elapsed, fallback_packets: fallbackCount
  });
}

/* ═══════════════════════════════════════════════
   ILP SOLVER — Exact via GLPK/WASM
   Two modes: fixed-route (tight LP) or multi-route (big-M)
   ═══════════════════════════════════════════════ */
export async function solveILP(model, glpk, opts = {}) {
  if (!glpk) throw new Error('GLPK not ready');
  if (!model.processing_delay_us) model.processing_delay_us = 3;
  if (!model.guard_band_us) model.guard_band_us = 3;
  const tmlim = opts.tmlim || 15;

  const pkts = expandPackets(model);
  if (pkts.length > 70) throw new Error(`Too many packets (${pkts.length}). Reduce flows or increase period.`);

  // Check if all packets have exactly 1 route → use tight formulation
  const allSingleRoute = pkts.every(pk => pk.routes.length === 1);

  const vars = new Set(), sub = [], bins = [], obj = [];
  let ci = 0;
  const sv = (p, h) => `s_${p}_${h}`;
  const yv = (l, a, b) => `y_${l.replace(/[^a-zA-Z0-9]/g, '_')}_${a}_${b}`;
  const av = n => { vars.add(n); return n; };
  const ac = (pre, terms, bnd) => { sub.push({ name: `${pre}_${ci++}`, vars: terms, bnds: bnd }); };
  const ops = [];

  if (allSingleRoute) {
    /* ── Fixed-route formulation: NO z-variables, per-pair tight M ── */
    for (let p = 0; p < pkts.length; p++) {
      const pk = pkts[p], rt = pk.routes[0];
      let earliestArr = pk.rel;
      for (let h = 0; h < rt.hops.length; h++) {
        const hp = rt.hops[h], s = av(sv(p, h));
        // Tight lower bound
        ac('lb', [{ name: s, coef: 1 }], { type: glpk.GLP_LO, lb: earliestArr, ub: 0 });
        // Tight upper bound
        let latestStart = model.cycle_time_us - hp.tx;
        if (pk.dl != null) {
          let tailTime = 0;
          for (let h2 = rt.hops.length - 1; h2 > h; h2--)
            tailTime += rt.hops[h2].tx + rt.hops[h2].pd + model.processing_delay_us;
          latestStart = Math.min(latestStart, pk.dl - hp.tx - hp.pd - tailTime);
        }
        ac('ub', [{ name: s, coef: 1 }], { type: glpk.GLP_UP, lb: 0, ub: latestStart });
        // Chain
        if (h < rt.hops.length - 1) {
          const sn = av(sv(p, h + 1));
          ac('ch', [{ name: sn, coef: 1 }, { name: s, coef: -1 }], { type: glpk.GLP_LO, lb: hp.tx + hp.pd + model.processing_delay_us, ub: 0 });
        }
        const blk = hp.tx + (pk.tsn ? model.guard_band_us : 0);
        ops.push({ oi: ops.length, p, r: 0, h, lid: hp.lid, sn: s, tx: hp.tx, blk, earliest: earliestArr, latest: latestStart });
        earliestArr += hp.tx + hp.pd + model.processing_delay_us;
      }
      // Deadline
      const last = rt.hops.length - 1, sL = sv(p, last), lH = rt.hops[last];
      if (pk.dl != null) ac('dl', [{ name: sL, coef: 1 }], { type: glpk.GLP_UP, lb: 0, ub: pk.dl - lH.tx - lH.pd });
      // Objective: minimize sum of last-hop start times for TSN packets
      if (pk.tsn) obj.push({ name: sL, coef: 1 });
    }

    // Pairwise ordering with per-pair tight M and window pruning
    for (const lnk of model.links) {
      const lo = ops.filter(o => o.lid === lnk.id);
      for (let a = 0; a < lo.length; a++) for (let b = a + 1; b < lo.length; b++) {
        const oa = lo[a], ob = lo[b];
        // Tight window pruning: skip if execution windows can't overlap
        if (oa.latest + oa.blk <= ob.earliest || ob.latest + ob.blk <= oa.earliest) continue;
        const y = av(yv(lnk.id, oa.oi, ob.oi)); bins.push(y);
        // Per-pair tight M: just enough to make constraint trivial when inactive
        const Mab = Math.max(oa.latest - ob.earliest + oa.blk, ob.latest - oa.earliest + ob.blk);
        // y=0: a before b → s_b >= s_a + blk_a  (with -Mab*y relaxation)
        // y=1: b before a → s_a >= s_b + blk_b  (with +Mab*y relaxation)
        ac('na', [{ name: ob.sn, coef: 1 }, { name: oa.sn, coef: -1 }, { name: y, coef: -Mab }], { type: glpk.GLP_LO, lb: oa.blk - Mab, ub: 0 });
        ac('nb', [{ name: oa.sn, coef: 1 }, { name: ob.sn, coef: -1 }, { name: y, coef: Mab }], { type: glpk.GLP_LO, lb: ob.blk, ub: 0 });
      }
    }
  } else {
    /* ── Multi-route formulation: big-M with z-variables ── */
    const M = model.cycle_time_us + model.guard_band_us + model.processing_delay_us + 100;
    const zv2 = (p, r) => `z_${p}_${r}`;
    for (let p = 0; p < pkts.length; p++) {
      const pk = pkts[p], zt = [];
      for (let r = 0; r < pk.routes.length; r++) {
        const rt = pk.routes[r], z = av(zv2(p, r));
        bins.push(z); zt.push({ name: z, coef: 1 });
        for (let h = 0; h < rt.hops.length; h++) {
          const hp = rt.hops[h], s = av(`s_${p}_${r}_${h}`);
          ac('lb', [{ name: s, coef: 1 }, { name: z, coef: -M }], { type: glpk.GLP_LO, lb: pk.rel - M, ub: 0 });
          ac('ub', [{ name: s, coef: 1 }, { name: z, coef: M }], { type: glpk.GLP_UP, lb: 0, ub: model.cycle_time_us - hp.tx + M });
          if (h < rt.hops.length - 1) {
            const sn = av(`s_${p}_${r}_${h + 1}`);
            ac('ch', [{ name: sn, coef: 1 }, { name: s, coef: -1 }, { name: z, coef: -M }], { type: glpk.GLP_LO, lb: hp.tx + hp.pd + model.processing_delay_us - M, ub: 0 });
          }
          ops.push({ oi: ops.length, p, r, h, lid: hp.lid, sn: s, zn: z, tx: hp.tx, blk: hp.tx + (pk.tsn ? model.guard_band_us : 0) });
        }
        const last = rt.hops.length - 1, sL = av(`s_${p}_${r}_${last}`), lH = rt.hops[last];
        if (pk.dl != null) ac('dl', [{ name: sL, coef: 1 }, { name: z, coef: M }], { type: glpk.GLP_UP, lb: 0, ub: pk.dl - lH.tx - lH.pd + M });
        if (pk.tsn) { obj.push({ name: sL, coef: 1 }); obj.push({ name: z, coef: lH.tx + lH.pd }); }
      }
      ac('sel', zt, { type: glpk.GLP_FX, lb: 1, ub: 1 });
    }
    for (const lnk of model.links) {
      const lo = ops.filter(o => o.lid === lnk.id);
      for (let a = 0; a < lo.length; a++) for (let b = a + 1; b < lo.length; b++) {
        const oa = lo[a], ob = lo[b];
        if (oa.p === ob.p && oa.r === ob.r) continue;
        const pa = pkts[oa.p], pb = pkts[ob.p];
        const aEnd = pa.dl ?? model.cycle_time_us;
        const bEnd = pb.dl ?? model.cycle_time_us;
        if (aEnd <= pb.rel || bEnd <= pa.rel) continue;
        const y = av(yv(lnk.id, oa.oi, ob.oi)); bins.push(y);
        ac('na', [{ name: ob.sn, coef: 1 }, { name: oa.sn, coef: -1 }, { name: y, coef: -M }, { name: oa.zn, coef: -M }, { name: ob.zn, coef: -M }], { type: glpk.GLP_LO, lb: oa.blk - 3 * M, ub: 0 });
        ac('nb', [{ name: oa.sn, coef: 1 }, { name: ob.sn, coef: -1 }, { name: y, coef: M }, { name: oa.zn, coef: -M }, { name: ob.zn, coef: -M }], { type: glpk.GLP_LO, lb: ob.blk - 2 * M, ub: 0 });
      }
    }
  }

  const lp = {
    name: 'tsn_ilp',
    objective: { direction: glpk.GLP_MIN, name: 'obj', vars: obj.length ? obj : [{ name: av('dum'), coef: 0 }] },
    subjectTo: sub, binaries: bins,
    bounds: Array.from(vars).map(n => ({ name: n, type: glpk.GLP_LO, lb: 0, ub: 0 }))
  };

  const solved = await glpk.solve(lp, { msglev: glpk.GLP_MSG_OFF, presol: true, tmlim });
  if (!solved?.result || ![glpk.GLP_OPT, glpk.GLP_FEAS].includes(solved.result.status))
    throw new Error('ILP infeasible (status=' + (solved?.result?.status ?? '?') + ')');

  const rv = solved.result.vars;

  // Build scheduled hops from ILP solution
  const schedHops = [];
  if (allSingleRoute) {
    for (let p = 0; p < pkts.length; p++) {
      const starts = [];
      for (let h = 0; h < pkts[p].routes[0].hops.length; h++) starts.push(Number(rv[sv(p, h)] || 0));
      schedHops.push({ route: 0, starts });
    }
  } else {
    const zv2 = (p, r) => `z_${p}_${r}`;
    for (let p = 0; p < pkts.length; p++) {
      const pk = pkts[p]; let selR = 0, bz = -1;
      for (let r = 0; r < pk.routes.length; r++) { const v = Number(rv[zv2(p, r)] || 0); if (v > bz) { bz = v; selR = r; } }
      const starts = [];
      for (let h = 0; h < pk.routes[selR].hops.length; h++) starts.push(Number(rv[`s_${p}_${selR}_${h}`] || 0));
      schedHops.push({ route: selR, starts });
    }
  }

  const statusLabel = solved.result.status === glpk.GLP_OPT ? 'optimal' : 'feasible (time limit)';
  return buildResult(model, pkts, schedHops,
    'ILP (GLPK v' + (typeof glpk.version === 'function' ? glpk.version() : (glpk.version || '?')) + ', ' + statusLabel + ')',
    { constraints: sub.length, variables: vars.size, binaries: bins.length, status: solved.result.status, runtime_ms: Math.round(solved.time * 1000) }
  );
}

/* ═══════════════════════════════════════════════
   RENDER: METRICS
   ═══════════════════════════════════════════════ */
export function renderMetrics(model, result, containerId = "metricsArea") {
  const area = document.getElementById(containerId);
  if (!area) return;
  const tsnPkts = result.packetRows.filter(p => p.status !== "BE");
  const overlapConflicts = Number(result.stats?.overlap_conflicts || 0);
  const fallbackPackets = Number(result.stats?.fallback_packets || 0);
  const allOk = tsnPkts.every(p => p.status === "OK") && overlapConflicts === 0 && fallbackPackets === 0;
  const avgDelay = tsnPkts.length ? (tsnPkts.reduce((s, p) => s + p.e2e_delay_us, 0) / tsnPkts.length).toFixed(2) : "-";

  const items = [
    { val: model.nodes.length, label: "Nodes", sub: `${model.nodes.filter(n=>n.type==="switch").length} switches`, cls: "" },
    { val: model.links.length, label: "Links", sub: "directional", cls: "" },
    { val: model.flows.length, label: "Flows", sub: `${result.packetRows.length} pkts/cycle`, cls: "" },
    { val: result.method.split("(")[0], label: "Solver", sub: `${result.stats.variables} vars, ${result.stats.binaries} bins`, cls: "" },
    { val: result.objective + " us", label: "Objective", sub: "TSN delay sum", cls: "warn" },
    { val: result.worst_util_percent + "%", label: "Worst Util", sub: "link utilization", cls: result.worst_util_percent > 80 ? "warn" : "" },
    { val: avgDelay + " us", label: "Avg TSN Delay", sub: `${tsnPkts.length} TSN packets`, cls: "" },
    { val: allOk ? "ALL OK" : "MISS", label: "Feasibility", sub: `${result.stats.constraints} constraints, overlaps=${overlapConflicts}`, cls: allOk ? "ok" : "warn" }
  ];

  area.innerHTML = items.map(m => `
    <div class="metric-card ${m.cls}">
      <div class="metric-val">${m.val}</div>
      <div class="metric-label">${m.label}</div>
      <div class="metric-sub">${m.sub}</div>
    </div>
  `).join("");
}

/* ═══════════════════════════════════════════════
   RENDER: TOPOLOGY (Force Graph)
   ═══════════════════════════════════════════════ */
export function renderTopology(model, result, opts = {}) {
  const containerId = opts.containerId || "topoContainer";
  const legendId = opts.legendId || "topoLegend";
  const nodePositions = opts.nodePositions || null;  // fixed positions map
  const nodeColors = opts.nodeColors || null;         // custom color map
  const height = opts.height || 380;

  const container = document.getElementById(containerId);
  if (!container) return;
  container.innerHTML = "";

  const W = container.clientWidth;
  const H = height;

  // Build legend
  const legendEl = document.getElementById(legendId);
  if (legendEl) {
    legendEl.innerHTML = "";
    const flowTypes = [...new Set(model.flows.map(f => f.traffic_type))];
    flowTypes.forEach(ft => {
      const c = FLOW_COLORS_HEX[ft] || "#7b61ff";
      const prio = model.flows.find(f => f.traffic_type === ft);
      legendEl.innerHTML += `<div class="legend-item"><div class="legend-dot" style="background:${c}"></div>${ft} (P${prio?.priority ?? '?'})</div>`;
    });
    if (!nodeColors) {
      legendEl.innerHTML += `<div class="legend-item"><div class="legend-dot" style="background:#6366f1"></div>switch</div>`;
      legendEl.innerHTML += `<div class="legend-item"><div class="legend-dot" style="background:#059669"></div>endstation</div>`;
    }
  }

  const svg = d3.select(container).append("svg")
    .attr("viewBox", `0 0 ${W} ${H}`)
    .attr("preserveAspectRatio", "xMidYMid meet");

  // Defs
  const defs = svg.append("defs");
  const glow = defs.append("filter").attr("id", "glow");
  glow.append("feGaussianBlur").attr("stdDeviation", "3").attr("result", "blur");
  glow.append("feMerge").selectAll("feMergeNode")
    .data(["blur", "SourceGraphic"]).enter()
    .append("feMergeNode").attr("in", d => d);

  defs.append("marker")
    .attr("id", "arrowhead")
    .attr("viewBox", "0 0 10 7").attr("refX", 35).attr("refY", 3.5)
    .attr("markerWidth", 8).attr("markerHeight", 6)
    .attr("orient", "auto")
    .append("polygon").attr("points", "0 0, 10 3.5, 0 7").attr("fill", "#94a3b8");

  // Build unique links
  const linkPairs = new Map();
  model.links.forEach(l => {
    const key = [l.from, l.to].sort().join("-");
    if (!linkPairs.has(key)) linkPairs.set(key, []);
    linkPairs.get(key).push(l);
  });

  const nodes = model.nodes.map(n => {
    const custom = nodeColors ? nodeColors[n.id] : null;
    return {
      id: n.id, type: n.type,
      color: custom ? custom.fill : (n.type === "switch" ? "#6366f1" : "#059669"),
      stroke: custom ? custom.stroke : (n.type === "switch" ? "#a5b4fc" : "#6ee7b7"),
      label: custom?.label || n.id,
      shortLabel: custom?.shortLabel || (n.type === "switch" ? "SW" : "ES")
    };
  });

  const uniqueLinks = [];
  linkPairs.forEach((links, key) => {
    uniqueLinks.push({
      source: links[0].from, target: links[0].to,
      bidir: links.length > 1, ids: links.map(l => l.id)
    });
  });

  // Set positions
  if (nodePositions) {
    nodes.forEach(n => {
      if (nodePositions[n.id]) {
        n.fx = nodePositions[n.id].x;
        n.fy = nodePositions[n.id].y;
      }
    });
  } else {
    // Auto positions for generic topology
    const defaultPos = {
      s1: { x: W / 2, y: 80 }, s2: { x: W / 2 - 140, y: 240 }, s3: { x: W / 2 + 140, y: 240 },
      esA: { x: W / 2 - 30, y: 20 }, esB: { x: W / 2 - 240, y: 310 }, esC: { x: W / 2 + 240, y: 310 }
    };
    nodes.forEach(n => {
      if (defaultPos[n.id]) { n.fx = defaultPos[n.id].x; n.fy = defaultPos[n.id].y; }
    });
  }

  const simulation = d3.forceSimulation(nodes)
    .force("link", d3.forceLink(uniqueLinks).id(d => d.id).distance(120))
    .force("charge", d3.forceManyBody().strength(-200))
    .force("center", d3.forceCenter(W / 2, H / 2))
    .alphaDecay(0.05);

  // Draw links
  const linkG = svg.append("g");
  const linkLine = linkG.selectAll("line")
    .data(uniqueLinks).enter().append("line")
    .attr("class", "topo-link")
    .attr("marker-end", "url(#arrowhead)");

  // Draw flow paths from result
  const flowPaths = [];
  if (result) {
    result.packetRows.forEach(p => {
      if (!flowPaths.some(fp => fp.flow_id === p.flow_id)) {
        const pathNodes = [];
        p.hops.forEach(h => {
          const link = model.links.find(l => l.id === h.link_id);
          if (link) {
            if (pathNodes.length === 0) pathNodes.push(link.from);
            pathNodes.push(link.to);
          }
        });
        flowPaths.push({ flow_id: p.flow_id, path: pathNodes, color: flowColor(p.flow_id) });
      }
    });
  }

  // Flow path lines (curved)
  const flowG = svg.append("g");
  const flowLines = flowG.selectAll("path")
    .data(flowPaths).enter().append("path")
    .attr("class", "flow-path")
    .attr("stroke", d => d.color)
    .attr("stroke-dasharray", "8,4")
    .attr("filter", "url(#glow)");

  function animateFlows() {
    flowLines.attr("stroke-dashoffset", 0)
      .transition().duration(2000).ease(d3.easeLinear)
      .attr("stroke-dashoffset", -24)
      .on("end", animateFlows);
  }

  // Particles
  const particleG = svg.append("g");
  function createParticles() {
    flowPaths.forEach(fp => {
      const nodeMap = new Map(nodes.map(n => [n.id, n]));
      const points = fp.path.map(id => nodeMap.get(id)).filter(Boolean);
      if (points.length < 2) return;

      const particle = particleG.append("circle")
        .attr("class", "flow-particle")
        .attr("r", 4).attr("fill", fp.color).attr("filter", "url(#glow)");

      function animateParticle() {
        let chain = particle
          .attr("cx", points[0].x || points[0].fx)
          .attr("cy", points[0].y || points[0].fy)
          .attr("opacity", 0.9);

        for (let i = 1; i < points.length; i++) {
          chain = chain.transition().duration(600).ease(d3.easeLinear)
            .attr("cx", points[i].x || points[i].fx)
            .attr("cy", points[i].y || points[i].fy);
        }
        chain.transition().duration(200).attr("opacity", 0)
          .transition().delay(500 + Math.random() * 1000).on("end", animateParticle);
      }
      setTimeout(animateParticle, Math.random() * 2000);
    });
  }

  // Draw nodes
  const nodeG = svg.append("g");
  const node = nodeG.selectAll("g")
    .data(nodes).enter().append("g")
    .attr("class", "topo-node")
    .call(d3.drag()
      .on("start", (e, d) => { if (!e.active) simulation.alphaTarget(0.3).restart(); d.fx = d.x; d.fy = d.y; })
      .on("drag", (e, d) => { d.fx = e.x; d.fy = e.y; })
      .on("end", (e, d) => { if (!e.active) simulation.alphaTarget(0); })
    );

  node.append("circle").attr("r", 30).attr("fill", "none")
    .attr("stroke", d => d.stroke).attr("stroke-width", 1).attr("opacity", 0.15);
  node.append("circle").attr("r", 22).attr("fill", d => d.color)
    .attr("stroke", d => d.stroke).attr("stroke-width", 2);
  node.append("text").attr("class", "topo-label").attr("dy", -1).text(d => d.label);
  node.append("text").attr("class", "topo-type").attr("dy", 12).text(d => d.shortLabel);

  simulation.on("tick", () => {
    linkLine
      .attr("x1", d => d.source.x).attr("y1", d => d.source.y)
      .attr("x2", d => d.target.x).attr("y2", d => d.target.y);
    node.attr("transform", d => `translate(${d.x},${d.y})`);
    flowLines.attr("d", fp => {
      const nodeMap = new Map(nodes.map(n => [n.id, n]));
      const points = fp.path.map(id => nodeMap.get(id)).filter(Boolean);
      if (points.length < 2) return "";
      const lineGen = d3.line().x(d => d.x).y(d => d.y).curve(d3.curveCatmullRom.alpha(0.5));
      const offset = flowPaths.indexOf(fp) * 3 - 3;
      return lineGen(points.map(p => ({ x: p.x + offset, y: p.y + offset })));
    });
  });

  setTimeout(() => { animateFlows(); createParticles(); }, 500);
}

/* ═══════════════════════════════════════════════
   RENDER: GCL TIMELINE (Gantt)
   ═══════════════════════════════════════════════ */
export function renderGCL(model, result, opts = {}) {
  const containerId = opts.containerId || "gclContainer";
  const legendId = opts.legendId || "gclLegend";
  const colorFn = opts.flowColorFn || flowColor;
  const beColor = opts.beColor || '#cbd5e1';
  const beBorder = opts.beBorder || 'rgba(100,116,139,.2)';

  const container = document.getElementById(containerId);
  if (!container) return;
  container.innerHTML = "";

  // Legend
  const legendEl = document.getElementById(legendId);
  if (legendEl) {
    const types = [...new Set(model.flows.map(f => f.traffic_type))];
    legendEl.innerHTML = types.map(t => {
      const rep = model.flows.find(f => f.traffic_type === t);
      const c = rep ? colorFn(rep.id) : (FLOW_COLORS_HEX[t] || "#7b61ff");
      return `<div class="legend-item"><div class="legend-dot" style="background:${c}"></div>${t} (P${rep?.priority ?? '?'})</div>`;
    }).join("") +
      `<div class="legend-item"><div class="legend-dot" style="background:#f9a825;background-image:repeating-linear-gradient(45deg,transparent,transparent 2px,rgba(0,0,0,0.15) 2px,rgba(0,0,0,0.15) 4px)"></div>Guard Band</div>` +
      `<div class="legend-item"><div class="legend-dot" style="background:${beColor}"></div>Best-Effort</div>`;
  }

  // Filter to links with TSN entries
  const activeLinks = model.links.filter(l => {
    const entries = result.gcl.links[l.id]?.entries || [];
    return entries.some(e => !e.note.includes("best-effort"));
  });

  // Auto-zoom: if TSN traffic uses <30% of cycle, zoom into active region
  let maxActiveEnd = 0;
  activeLinks.forEach(link => {
    const entries = result.gcl.links[link.id]?.entries || [];
    entries.forEach(e => {
      if (!e.note.includes("best-effort")) maxActiveEnd = Math.max(maxActiveEnd, e.end_us);
    });
  });
  const zoomThreshold = model.cycle_time_us * 0.3;
  const isZoomed = maxActiveEnd > 0 && maxActiveEnd < zoomThreshold;
  const xMax = isZoomed ? Math.max(maxActiveEnd * 1.5, maxActiveEnd + 20) : model.cycle_time_us;

  const margin = { top: 30, right: 30, bottom: isZoomed ? 50 : 40, left: 140 };
  const rowH = 48;
  const W = container.clientWidth;
  const H = margin.top + activeLinks.length * rowH + margin.bottom;

  const svg = d3.select(container).append("svg")
    .attr("viewBox", `0 0 ${W} ${H}`)
    .attr("preserveAspectRatio", "xMidYMid meet");

  // Defs: hatched pattern for guard bands + drop shadow
  const defs = svg.append("defs");
  const guardPat = defs.append("pattern")
    .attr("id", "guardHatch").attr("patternUnits", "userSpaceOnUse")
    .attr("width", 6).attr("height", 6).attr("patternTransform", "rotate(45)");
  guardPat.append("rect").attr("width", 6).attr("height", 6).attr("fill", "#f9a825");
  guardPat.append("line").attr("x1", 0).attr("y1", 0).attr("x2", 0).attr("y2", 6)
    .attr("stroke", "rgba(0,0,0,0.2)").attr("stroke-width", 2);

  const dropShadow = defs.append("filter").attr("id", "barShadow")
    .attr("x", "-5%").attr("y", "-5%").attr("width", "110%").attr("height", "120%");
  dropShadow.append("feDropShadow")
    .attr("dx", 0).attr("dy", 1).attr("stdDeviation", 1.5)
    .attr("flood-color", "rgba(0,0,0,0.12)");

  const g = svg.append("g").attr("transform", `translate(${margin.left},${margin.top})`);
  const innerW = W - margin.left - margin.right;
  const innerH = activeLinks.length * rowH;

  const x = d3.scaleLinear().domain([0, xMax]).range([0, innerW]);
  const y = d3.scaleBand().domain(activeLinks.map(l => l.id)).range([0, innerH]).padding(0.2);

  // Grid
  g.append("g").attr("class", "gcl-grid")
    .selectAll("line").data(x.ticks(10)).enter().append("line")
    .attr("x1", d => x(d)).attr("x2", d => x(d))
    .attr("y1", 0).attr("y2", innerH);

  // X axis
  g.append("g").attr("class", "gcl-axis")
    .attr("transform", `translate(0,${innerH})`)
    .call(d3.axisBottom(x).ticks(10).tickFormat(d => d + " \u00b5s"))
    .selectAll("text").attr("fill", "var(--text3)");

  // Y axis
  g.append("g").attr("class", "gcl-axis")
    .call(d3.axisLeft(y).tickSize(0).tickPadding(8))
    .selectAll("text")
    .attr("fill", "var(--text2)")
    .attr("font-size", "10px")
    .text(d => {
      const link = model.links.find(l => l.id === d);
      return link ? `${link.from} \u2192 ${link.to}` : d;
    });

  // Row backgrounds
  g.selectAll(".row-bg")
    .data(activeLinks).enter().append("rect")
    .attr("x", 0).attr("y", d => y(d.id))
    .attr("width", innerW).attr("height", y.bandwidth())
    .attr("fill", (d, i) => i % 2 === 0 ? "rgba(59,130,246,0.03)" : "transparent").attr("rx", 4);

  // Collect all flow bars for cross-flow highlighting
  const allFlowBars = [];

  // GCL bars
  activeLinks.forEach(link => {
    const entries = result.gcl.links[link.id]?.entries || [];
    const barG = g.append("g");

    // When zoomed, only show entries within visible range
    const visibleEntries = isZoomed ? entries.filter(e => e.start_us < xMax) : entries;

    barG.selectAll("rect.gcl-bar")
      .data(visibleEntries).enter().append("rect")
      .attr("class", d => {
        const fid = d.note.includes("guard") || d.note.includes("best-effort") ? '' : d.note.split("#")[0];
        return `gcl-bar ${fid ? 'flow-bar' : ''}`
      })
      .attr("data-flow", d => {
        if (d.note.includes("guard") || d.note.includes("best-effort")) return '';
        return d.note.split("#")[0];
      })
      .attr("x", d => x(d.start_us))
      .attr("y", y(link.id))
      .attr("width", 0)
      .attr("height", y.bandwidth())
      .attr("fill", d => {
        if (d.note.includes("guard")) return "url(#guardHatch)";
        if (d.note.includes("best-effort")) return beColor;
        return colorFn(d.note);
      })
      .attr("stroke", d => {
        if (d.note.includes("guard")) return "#e8a317";
        if (d.note.includes("best-effort")) return beBorder;
        return "rgba(255,255,255,0.4)";
      })
      .attr("stroke-width", d => d.note.includes("best-effort") ? 0.5 : 0.8)
      .attr("opacity", d => d.note.includes("best-effort") ? 0.3 : 0.9)
      .attr("rx", 3)
      .attr("filter", d => d.note.includes("best-effort") || d.note.includes("guard") ? "none" : "url(#barShadow)")
      .on("mouseover", function(evt, d) {
        const fid = d.note.split("#")[0];
        // Cross-flow highlight: dim all, brighten same flow
        if (!d.note.includes("best-effort") && !d.note.includes("guard")) {
          g.selectAll(".flow-bar").attr("opacity", function() {
            return d3.select(this).attr("data-flow") === fid ? 1.0 : 0.25;
          });
        }
        const flowId = d.note.split("#")[0];
        const pktId = d.note;
        const pktRow = result.packetRows.find(p => p.packet_id === pktId);
        showTip(evt, `
          <div class="tt-title">${pktId}</div>
          <div class="tt-row"><span class="tt-k">Flow</span><span class="tt-v">${flowId.replace("f_","")}</span></div>
          <div class="tt-row"><span class="tt-k">Gate</span><span class="tt-v">${d.gate_mask}</span></div>
          <div class="tt-row"><span class="tt-k">Start</span><span class="tt-v">${d.start_us} \u00b5s</span></div>
          <div class="tt-row"><span class="tt-k">End</span><span class="tt-v">${d.end_us} \u00b5s</span></div>
          <div class="tt-row"><span class="tt-k">Duration</span><span class="tt-v">${d.duration_us} \u00b5s</span></div>
          ${pktRow ? `<div class="tt-row"><span class="tt-k">E2E Delay</span><span class="tt-v">${pktRow.e2e_delay_us} \u00b5s</span></div>` : ''}
        `);
      })
      .on("mousemove", (evt) => {
        tooltipEl.style.left = (evt.clientX + 14) + "px";
        tooltipEl.style.top = (evt.clientY - 10) + "px";
      })
      .on("mouseout", function() {
        g.selectAll(".flow-bar").attr("opacity", 0.9);
        hideTip();
      })
      .transition().duration(800).delay((d, i) => i * 30)
      .attr("width", d => {
        const endClamped = Math.min(d.start_us + d.duration_us, xMax);
        const w = x(endClamped) - x(d.start_us);
        return d.note.includes("best-effort") ? Math.max(w, 0) : Math.max(w, 3);
      });

    // Gate transition markers — small ticks at flow/guard boundaries
    const transitions = visibleEntries.filter(e => !e.note.includes("best-effort"));
    barG.selectAll("line.gate-tick")
      .data(transitions).enter().append("line")
      .attr("class", "gate-tick")
      .attr("x1", d => x(d.start_us)).attr("x2", d => x(d.start_us))
      .attr("y1", y(link.id) - 2).attr("y2", y(link.id) + 4)
      .attr("stroke", d => d.note.includes("guard") ? "#e8a317" : "rgba(0,0,0,0.3)")
      .attr("stroke-width", 1);

    // Labels on flow bars
    barG.selectAll("text")
      .data(visibleEntries.filter(e => !e.note.includes("best-effort") && !e.note.includes("guard")))
      .enter().append("text")
      .attr("class", "gcl-label")
      .attr("x", d => {
        const endClamped = Math.min(d.start_us + d.duration_us, xMax);
        return x(d.start_us) + (x(endClamped) - x(d.start_us)) / 2;
      })
      .attr("y", d => y(link.id) + y.bandwidth() / 2)
      .attr("text-anchor", "middle").attr("dominant-baseline", "central")
      .attr("opacity", 0)
      .text(d => d.note.split("#")[0].replace("f_", ""))
      .transition().delay(800).duration(300).attr("opacity", 1);
  });

  // Cycle info + zoom indicator
  svg.append("text")
    .attr("x", margin.left).attr("y", margin.top - 10)
    .attr("fill", "var(--text3)").attr("font-size", "10px")
    .text(`Cycle: ${model.cycle_time_us} \u00b5s | Guard: ${model.guard_band_us} \u00b5s | Active: ${activeLinks.length}/${model.links.length} links${isZoomed ? ` | Zoom: 0\u2013${Math.round(xMax)} \u00b5s` : ''}`);

  // Zoom indicator bar
  if (isZoomed) {
    const zoomBarY = H - 14;
    const zoomRatio = xMax / model.cycle_time_us;
    svg.append("rect")
      .attr("x", margin.left).attr("y", zoomBarY)
      .attr("width", innerW).attr("height", 6)
      .attr("fill", beBorder).attr("opacity", 0.3).attr("rx", 3);
    svg.append("rect")
      .attr("x", margin.left).attr("y", zoomBarY)
      .attr("width", innerW * zoomRatio).attr("height", 6)
      .attr("fill", "#3B82F6").attr("opacity", 0.5).attr("rx", 3);
    svg.append("text")
      .attr("x", margin.left + innerW * zoomRatio + 6).attr("y", zoomBarY + 5)
      .attr("fill", "#64748b").attr("font-size", "8px")
      .text(`${(zoomRatio * 100).toFixed(0)}% of cycle`);
  }
}

/* ═══════════════════════════════════════════════
   RENDER: PACKET DELAY CHART
   ═══════════════════════════════════════════════ */
export function renderDelayChart(model, result, optsOrId = "delayContainer") {
  const _opts = typeof optsOrId === 'string' ? { containerId: optsOrId } : (optsOrId || {});
  const containerId = _opts.containerId || "delayContainer";
  const colorFn = _opts.flowColorFn || flowColor;
  const container = document.getElementById(containerId);
  if (!container) return;
  container.innerHTML = "";

  const margin = { top: 20, right: 20, bottom: 60, left: 55 };
  const W = container.clientWidth;
  const H = 380;

  const svg = d3.select(container).append("svg")
    .attr("viewBox", `0 0 ${W} ${H}`)
    .attr("preserveAspectRatio", "xMidYMid meet");

  const g = svg.append("g").attr("transform", `translate(${margin.left},${margin.top})`);
  const innerW = W - margin.left - margin.right;
  const innerH = H - margin.top - margin.bottom;

  const pkts = result.packetRows;

  const x = d3.scaleBand().domain(pkts.map(p => p.packet_id)).range([0, innerW]).padding(0.25);
  const maxDelay = Math.max(...pkts.map(p => Math.max(p.e2e_delay_us, p.deadline_abs_us || 0)));
  const y = d3.scaleLinear().domain([0, maxDelay * 1.15]).range([innerH, 0]);

  // Grid
  g.append("g").attr("class", "gcl-grid")
    .selectAll("line").data(y.ticks(6)).enter().append("line")
    .attr("x1", 0).attr("x2", innerW)
    .attr("y1", d => y(d)).attr("y2", d => y(d));

  // Deadline lines
  const deadlineGroups = {};
  pkts.forEach(p => {
    if (p.deadline_abs_us != null) {
      const dlRel = p.deadline_abs_us - p.release_us;
      const key = p.flow_id;
      if (!deadlineGroups[key]) deadlineGroups[key] = { flow_id: key, deadline: dlRel };
    }
  });

  Object.values(deadlineGroups).forEach(dg => {
    g.append("line").attr("class", "deadline-line")
      .attr("x1", 0).attr("x2", innerW)
      .attr("y1", y(dg.deadline)).attr("y2", y(dg.deadline));
    g.append("text")
      .attr("x", innerW - 4).attr("y", y(dg.deadline) - 5)
      .attr("text-anchor", "end").attr("fill", "var(--red)").attr("font-size", "9px")
      .text(`Deadline: ${dg.flow_id.replace("f_", "")} (${dg.deadline} us)`);
  });

  // Bars
  g.selectAll(".delay-bar")
    .data(pkts).enter().append("rect")
    .attr("class", "delay-bar")
    .attr("x", d => x(d.packet_id)).attr("y", innerH)
    .attr("width", x.bandwidth()).attr("height", 0)
    .attr("fill", d => colorFn(d.flow_id)).attr("opacity", 0.85)
    .on("mouseover", (evt, d) => {
      showTip(evt, `
        <div class="tt-title">${d.packet_id}</div>
        <div class="tt-row"><span class="tt-k">E2E Delay</span><span class="tt-v">${d.e2e_delay_us} us</span></div>
        <div class="tt-row"><span class="tt-k">Release</span><span class="tt-v">${d.release_us} us</span></div>
        <div class="tt-row"><span class="tt-k">Finish</span><span class="tt-v">${d.end_us} us</span></div>
        ${d.deadline_abs_us != null ? `<div class="tt-row"><span class="tt-k">Slack</span><span class="tt-v" style="color:${d.slack_us > 0 ? 'var(--green)' : 'var(--red)'}">${d.slack_us} us</span></div>` : ''}
        <div class="tt-row"><span class="tt-k">Status</span><span class="tt-v">${d.status}</span></div>
      `);
    })
    .on("mousemove", (evt) => {
      tooltipEl.style.left = (evt.clientX + 14) + "px";
      tooltipEl.style.top = (evt.clientY - 10) + "px";
    })
    .on("mouseout", hideTip)
    .transition().duration(800).delay((d, i) => i * 80).ease(d3.easeCubicOut)
    .attr("y", d => y(d.e2e_delay_us))
    .attr("height", d => innerH - y(d.e2e_delay_us));

  // Value labels
  g.selectAll(".bar-val")
    .data(pkts).enter().append("text")
    .attr("x", d => x(d.packet_id) + x.bandwidth() / 2)
    .attr("y", d => y(d.e2e_delay_us) - 6)
    .attr("text-anchor", "middle").attr("fill", "var(--text2)")
    .attr("font-size", "9px").attr("font-weight", "600").attr("opacity", 0)
    .text(d => d.e2e_delay_us.toFixed(1))
    .transition().delay(800).duration(300).attr("opacity", 1);

  // Axes
  g.append("g").attr("class", "delay-axis")
    .attr("transform", `translate(0,${innerH})`)
    .call(d3.axisBottom(x).tickSize(0).tickPadding(8))
    .selectAll("text")
    .attr("transform", "rotate(-35)").attr("text-anchor", "end").attr("font-size", "8px")
    .text(d => d.replace("f_", "").replace(/_/g, " ").substring(0, 20));

  g.append("g").attr("class", "delay-axis")
    .call(d3.axisLeft(y).ticks(6).tickFormat(d => d + " us"));

  svg.append("text")
    .attr("transform", "rotate(-90)")
    .attr("x", -H / 2).attr("y", 14)
    .attr("text-anchor", "middle").attr("fill", "var(--text3)").attr("font-size", "10px")
    .text("E2E Delay (us)");
}

/* ═══════════════════════════════════════════════
   RENDER: LINK UTILIZATION (Donuts)
   ═══════════════════════════════════════════════ */
export function renderUtilization(model, result, optsOrId = "utilContainer") {
  const _opts = typeof optsOrId === 'string' ? { containerId: optsOrId } : (optsOrId || {});
  const containerId = _opts.containerId || "utilContainer";
  const beColor = _opts.beColor || "#0d1a30";
  const container = document.getElementById(containerId);
  if (!container) return;
  container.innerHTML = "";

  const linkData = model.links.map(l => {
    const entries = result.gcl.links[l.id]?.entries || [];
    let flowTime = 0, guardTime = 0, beTime = 0;
    entries.forEach(e => {
      if (e.note.includes("best-effort")) beTime += e.duration_us;
      else if (e.note.includes("guard")) guardTime += e.duration_us;
      else flowTime += e.duration_us;
    });
    return {
      id: l.id, from: l.from, to: l.to,
      flow: flowTime, guard: guardTime, be: beTime,
      util: ((flowTime + guardTime) / model.cycle_time_us * 100).toFixed(1)
    };
  }).filter(l => l.flow > 0 || l.guard > 0);

  const donutSize = 90;
  const cols = Math.min(linkData.length, Math.floor(container.clientWidth / (donutSize + 30)));
  const rows = Math.ceil(linkData.length / cols);
  const W = container.clientWidth;
  const H = rows * (donutSize + 40) + 20;

  const svg = d3.select(container).append("svg")
    .attr("viewBox", `0 0 ${W} ${H}`)
    .attr("preserveAspectRatio", "xMidYMid meet");

  const pie = d3.pie().value(d => d.value).sort(null);
  const arc = d3.arc().innerRadius(28).outerRadius(40);

  linkData.forEach((link, i) => {
    const col = i % cols;
    const row = Math.floor(i / cols);
    const cx = (W / cols) * (col + 0.5);
    const cy = 50 + row * (donutSize + 40);
    const g = svg.append("g").attr("transform", `translate(${cx},${cy})`);

    const data = [
      { label: "Flow", value: link.flow, color: "#4895ef" },
      { label: "Guard", value: link.guard, color: "#f9a825" },
      { label: "BE", value: link.be, color: beColor }
    ];

    g.selectAll(".util-arc")
      .data(pie(data)).enter().append("path")
      .attr("class", "util-arc")
      .attr("fill", d => d.data.color)
      .attr("stroke", "var(--bg)").attr("stroke-width", 1.5)
      .attr("opacity", d => d.data.label === "BE" ? 0.3 : 0.85)
      .on("mouseover", (evt, d) => {
        showTip(evt, `
          <div class="tt-title">${link.from} \u2192 ${link.to}</div>
          <div class="tt-row"><span class="tt-k">${d.data.label}</span><span class="tt-v">${d.data.value.toFixed(1)} us</span></div>
        `);
      })
      .on("mouseout", hideTip)
      .transition().duration(800).delay(i * 100)
      .attrTween("d", function(d) {
        const interp = d3.interpolate({ startAngle: 0, endAngle: 0 }, d);
        return t => arc(interp(t));
      });

    g.append("text").attr("class", "util-label").attr("dy", 1)
      .text(link.util + "%").attr("opacity", 0)
      .transition().delay(800 + i * 100).duration(300).attr("opacity", 1);

    g.append("text").attr("class", "util-name").attr("y", 52)
      .text(`${link.from} \u2192 ${link.to}`);
  });
}

/* ═══════════════════════════════════════════════
   RENDER: PACKET TABLE
   ═══════════════════════════════════════════════ */
export function renderTable(result, optsOrId = "pktTableBody") {
  const _opts = typeof optsOrId === 'string' ? { containerId: optsOrId } : (optsOrId || {});
  const containerId = _opts.containerId || "pktTableBody";
  const colorFn = _opts.flowColorFn || flowColor;
  const tbody = document.getElementById(containerId);
  if (!tbody) return;
  tbody.innerHTML = result.packetRows.map(p => {
    const ft = flowType(p.flow_id);
    const badgeColor = colorFn(p.flow_id);
    const statusCls = p.status === "OK" ? "ok" : p.status === "MISS" ? "miss" : "be";
    return `<tr>
      <td>${p.packet_id.replace("f_", "")}</td>
      <td><span class="flow-badge" style="background:${badgeColor}20;color:${badgeColor};border:1px solid ${badgeColor}40">${ft}</span></td>
      <td>R${p.selected_route}</td>
      <td>${p.release_us}</td>
      <td>${p.end_us}</td>
      <td>${p.e2e_delay_us}</td>
      <td>${p.deadline_abs_us ?? "-"}</td>
      <td>${p.slack_us != null ? p.slack_us : "-"}</td>
      <td class="${statusCls}">${p.status}</td>
    </tr>`;
  }).join("");
}

/* ═══════════════════════════════════════════════
   RENDER: PER-SWITCH GCL DETAIL CARDS
   Shows GCL entries per egress port of each switch
   ═══════════════════════════════════════════════ */
export function renderSwitchGCL(model, result, opts = {}) {
  const containerId = opts.containerId || "switchGclArea";
  const switches = opts.switches || [];
  const colorFn = opts.flowColorFn || flowColor;

  const area = document.getElementById(containerId);
  if (!area || !switches.length) return;
  area.innerHTML = "";

  // Compute auto-zoom range across all switches
  let globalMaxActive = 0;
  switches.forEach(sw => {
    model.links.filter(l => l.from === sw.id).forEach(l => {
      const entries = result.gcl.links[l.id]?.entries || [];
      entries.forEach(e => {
        if (!e.note.includes("best-effort")) globalMaxActive = Math.max(globalMaxActive, e.end_us);
      });
    });
  });
  const zoomThreshold = model.cycle_time_us * 0.3;
  const isZoomed = globalMaxActive > 0 && globalMaxActive < zoomThreshold;
  const xMax = isZoomed ? Math.max(globalMaxActive * 1.5, globalMaxActive + 20) : model.cycle_time_us;

  switches.forEach(sw => {
    const egressLinks = model.links.filter(l => l.from === sw.id);
    const activeEgress = egressLinks.filter(l => {
      const entries = result.gcl.links[l.id]?.entries || [];
      return entries.some(e => !e.note.includes("best-effort"));
    });
    if (activeEgress.length === 0) return;

    let totalFlow = 0, totalGuard = 0, tsnCount = 0;
    activeEgress.forEach(l => {
      const entries = result.gcl.links[l.id]?.entries || [];
      entries.forEach(e => {
        if (e.note.includes("guard")) totalGuard += e.duration_us;
        else if (!e.note.includes("best-effort")) { totalFlow += e.duration_us; tsnCount++; }
      });
    });
    const utilPct = ((totalFlow + totalGuard) / model.cycle_time_us / activeEgress.length * 100).toFixed(1);

    const card = document.createElement("div");
    card.className = "gcl-switch-card";

    card.innerHTML = `
      <div class="sw-header">
        <div class="sw-dot" style="background:${sw.color};"></div>
        <span class="sw-name">${sw.label}</span>
        <span class="sw-chip">${sw.chip}</span>
        <span class="sw-stats">${activeEgress.length} port${activeEgress.length > 1 ? 's' : ''} \u00b7 ${tsnCount} entries \u00b7 ${utilPct}% util</span>
      </div>
    `;

    // Mini SVG Gantt per egress port
    activeEgress.forEach(link => {
      const entries = result.gcl.links[link.id]?.entries || [];
      const linkDiv = document.createElement("div");
      linkDiv.style.marginBottom = "14px";

      // Label
      const labelDiv = document.createElement("div");
      labelDiv.className = "link-label-dir";
      labelDiv.innerHTML = `${link.from} \u2192 ${link.to}`;
      linkDiv.appendChild(labelDiv);

      // Mini Gantt SVG
      const ganttH = 28;
      const ganttW = 100; // percentage-based via viewBox
      const svgEl = document.createElementNS("http://www.w3.org/2000/svg", "svg");
      svgEl.setAttribute("viewBox", `0 0 400 ${ganttH}`);
      svgEl.setAttribute("preserveAspectRatio", "xMidYMid meet");
      svgEl.style.width = "100%";
      svgEl.style.height = ganttH + "px";
      svgEl.style.borderRadius = "6px";
      svgEl.style.overflow = "hidden";

      // Hatched pattern for guard bands (inline in each mini SVG)
      const defsEl = document.createElementNS("http://www.w3.org/2000/svg", "defs");
      const patEl = document.createElementNS("http://www.w3.org/2000/svg", "pattern");
      patEl.setAttribute("id", `gh_${sw.id}_${link.id.replace(/[^a-zA-Z0-9]/g,'_')}`);
      patEl.setAttribute("patternUnits", "userSpaceOnUse");
      patEl.setAttribute("width", "5"); patEl.setAttribute("height", "5");
      patEl.setAttribute("patternTransform", "rotate(45)");
      const patBg = document.createElementNS("http://www.w3.org/2000/svg", "rect");
      patBg.setAttribute("width", "5"); patBg.setAttribute("height", "5"); patBg.setAttribute("fill", "#f9a825");
      patEl.appendChild(patBg);
      const patLine = document.createElementNS("http://www.w3.org/2000/svg", "line");
      patLine.setAttribute("x1", "0"); patLine.setAttribute("y1", "0");
      patLine.setAttribute("x2", "0"); patLine.setAttribute("y2", "5");
      patLine.setAttribute("stroke", "rgba(0,0,0,0.2)"); patLine.setAttribute("stroke-width", "1.5");
      patEl.appendChild(patLine);
      defsEl.appendChild(patEl);
      svgEl.appendChild(defsEl);

      // Background
      const bgRect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
      bgRect.setAttribute("width", "400"); bgRect.setAttribute("height", String(ganttH));
      bgRect.setAttribute("fill", "rgba(0,0,0,0.03)"); bgRect.setAttribute("rx", "4");
      svgEl.appendChild(bgRect);

      const visibleEntries = isZoomed ? entries.filter(e => e.start_us < xMax) : entries;
      const patId = `gh_${sw.id}_${link.id.replace(/[^a-zA-Z0-9]/g,'_')}`;

      visibleEntries.forEach(e => {
        const isBE = e.note.includes("best-effort");
        const isGuard = e.note.includes("guard");
        const sx = (e.start_us / xMax) * 400;
        const ew = Math.max(((Math.min(e.end_us, xMax) - e.start_us) / xMax) * 400, isBE ? 0 : 2);

        const rect = document.createElementNS("http://www.w3.org/2000/svg", "rect");
        rect.setAttribute("x", String(sx));
        rect.setAttribute("y", "0");
        rect.setAttribute("width", String(ew));
        rect.setAttribute("height", String(ganttH));
        rect.setAttribute("rx", "2");

        if (isGuard) {
          rect.setAttribute("fill", `url(#${patId})`);
          rect.setAttribute("opacity", "0.85");
        } else if (isBE) {
          rect.setAttribute("fill", "rgba(0,0,0,0.04)");
        } else {
          rect.setAttribute("fill", colorFn(e.note));
          rect.setAttribute("opacity", "0.85");
        }
        svgEl.appendChild(rect);

        // Flow label inside bar if wide enough
        if (!isBE && !isGuard && ew > 30) {
          const txt = document.createElementNS("http://www.w3.org/2000/svg", "text");
          txt.setAttribute("x", String(sx + ew / 2));
          txt.setAttribute("y", String(ganttH / 2 + 1));
          txt.setAttribute("text-anchor", "middle");
          txt.setAttribute("dominant-baseline", "central");
          txt.setAttribute("font-size", "9");
          txt.setAttribute("fill", "#fff");
          txt.setAttribute("font-weight", "600");
          txt.textContent = e.note.split("#")[0].replace("f_", "");
          svgEl.appendChild(txt);
        }
      });

      linkDiv.appendChild(svgEl);

      // Gate mask row — visual 8-queue blocks
      const gateDiv = document.createElement("div");
      gateDiv.style.cssText = "display:flex;gap:3px;margin-top:4px;flex-wrap:wrap;";

      // Show unique gate states (excluding BE)
      const uniqueGates = [];
      const seenMasks = new Set();
      visibleEntries.forEach(e => {
        if (e.note.includes("best-effort")) return;
        if (!seenMasks.has(e.gate_mask)) {
          seenMasks.add(e.gate_mask);
          uniqueGates.push(e);
        }
      });

      uniqueGates.forEach(e => {
        const isGuard = e.note.includes("guard");
        const gateEl = document.createElement("div");
        gateEl.style.cssText = "display:flex;align-items:center;gap:2px;padding:2px 6px;border-radius:4px;background:rgba(0,0,0,0.03);font-size:0.7rem;";

        // 8 queue blocks
        for (let q = 0; q < 8; q++) {
          const qOpen = e.gate_mask[q] === '1';
          const block = document.createElement("span");
          block.style.cssText = `display:inline-block;width:8px;height:8px;border-radius:2px;border:1px solid ${isGuard ? '#e8a317' : qOpen ? colorFn(e.note) : 'rgba(0,0,0,0.15)'};background:${isGuard ? '#f9a825' : qOpen ? colorFn(e.note) : 'transparent'};opacity:${qOpen || isGuard ? '0.85' : '0.3'};`;
          block.title = `TC${7 - q}: ${qOpen ? 'OPEN' : 'CLOSED'}`;
          gateEl.appendChild(block);
        }

        const lbl = document.createElement("span");
        lbl.style.cssText = `margin-left:4px;font-weight:600;color:${isGuard ? '#e8a317' : 'var(--text2)'};`;
        lbl.textContent = isGuard ? 'guard' : e.note.split("#")[0].replace("f_","");
        gateEl.appendChild(lbl);
        gateDiv.appendChild(gateEl);
      });

      linkDiv.appendChild(gateDiv);
      card.appendChild(linkDiv);
    });

    area.appendChild(card);
  });
}

/* BFS shortest path (returns link IDs) */
function bfsPath(adj, src, dst) {
  const visited = new Set([src]);
  const queue = [[src, []]];
  while (queue.length > 0) {
    const [node, path] = queue.shift();
    for (const edge of (adj.get(node) || [])) {
      if (visited.has(edge.to)) continue;
      const newPath = [...path, edge.lid];
      if (edge.to === dst) return newPath;
      visited.add(edge.to);
      queue.push([edge.to, newPath]);
    }
  }
  return [];
}
