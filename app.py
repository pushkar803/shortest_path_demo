
from __future__ import annotations

import heapq
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any

from flask import Flask, request, jsonify, render_template_string


# ============================================================
# 1) MAP CONFIG (predefined locations + predefined paths)
#    - Edit this section to match your demo layout.
# ============================================================

NODES = [
    {"id": "A", "name": "Entry",    "x": 60,  "y": 80},
    {"id": "B", "name": "Desk",     "x": 180, "y": 80},
    {"id": "C", "name": "Door",     "x": 300, "y": 80},
    {"id": "D", "name": "Charging", "x": 60,  "y": 200},
    {"id": "E", "name": "Shelf",    "x": 180, "y": 200},
    {"id": "F", "name": "Meeting",  "x": 300, "y": 200},
    {"id": "G", "name": "Window",   "x": 420, "y": 200},
    {"id": "H", "name": "Corner",   "x": 180, "y": 320},
]

NODE_POS = {n["id"]: (n["x"], n["y"]) for n in NODES}

# Undirected weighted edges (cost can be "time", "distance", etc.)
EDGES = [
    ("A", "B", 1),
    ("A", "D", 1),
    ("B", "C", 1),
    ("B", "E", 1),
    ("C", "F", 1),
    ("E", "F", 1),
    ("E", "H", 1),
    ("F", "G", 1),
    ("D", "B", 1),
    ("D", "E", 1),
    # ("D", "G", 3),
    ("H", "G", 1.7),
]

DEFAULT_START_NODE = "A"  # Start from A by default

# Optional: discourage "up/down" oscillation
REVERSAL_PENALTY = 2.0
# (Reversal = immediately going X->Y then Y->X in the expanded route)


import math

NODE_POS = {n["id"]: (n["x"], n["y"]) for n in NODES}

def turn_angle_deg(a: str, b: str, c: str) -> float:
    ax, ay = NODE_POS[a]
    bx, by = NODE_POS[b]
    cx, cy = NODE_POS[c]

    v1x, v1y = bx - ax, by - ay
    v2x, v2y = cx - bx, cy - by

    n1 = math.hypot(v1x, v1y)
    n2 = math.hypot(v2x, v2y)
    if n1 == 0 or n2 == 0:
        return 0.0

    dot = v1x * v2x + v1y * v2y
    cosang = max(-1.0, min(1.0, dot / (n1 * n2)))
    return math.degrees(math.acos(cosang))


UTURN_HARD_LIMIT_DEG = 165   # set None to disable hard blocking
UTURN_PENALTY = 80.0         # used when hard limit is None or for softer discouragement

def held_karp_open_path_no_uturn(dist_mat, idx_to_node):
    """
    dist_mat: (n+1)x(n+1), index 0=start, 1..n=selected nodes
    idx_to_node: {idx: node_id}
    Returns: best_cost, index_path [0, ...]
    """
    n = len(dist_mat) - 1
    if n <= 0:
        return 0.0, [0]

    INF = 1e18
    dp = {}       # (mask, prev, cur) -> cost
    parent = {}   # (mask, prev, cur) -> previous state

    # init: start(0)->j
    for j in range(1, n + 1):
        mask = 1 << (j - 1)
        dp[(mask, 0, j)] = dist_mat[0][j]
        parent[(mask, 0, j)] = None

    for mask in range(1, 1 << n):
        for prev in range(0, n + 1):
            for cur in range(1, n + 1):
                state = (mask, prev, cur)
                if state not in dp:
                    continue

                for nxt in range(1, n + 1):
                    if mask & (1 << (nxt - 1)):
                        continue

                    # 1) hard-block immediate backtrack: prev->cur->prev
                    if prev != 0 and nxt == prev:
                        continue

                    turn_cost = 0.0
                    if prev != 0:
                        a = idx_to_node[prev]
                        b = idx_to_node[cur]
                        c = idx_to_node[nxt]
                        ang = turn_angle_deg(a, b, c)

                        # 2) block near-180 turns (optional)
                        if UTURN_HARD_LIMIT_DEG is not None and ang >= UTURN_HARD_LIMIT_DEG:
                            continue

                        # 3) or/and penalize "big U-turn-ish" angles
                        if ang >= 150:
                            turn_cost += UTURN_PENALTY

                    new_mask = mask | (1 << (nxt - 1))
                    new_state = (new_mask, cur, nxt)
                    new_cost = dp[state] + dist_mat[cur][nxt] + turn_cost

                    if new_cost < dp.get(new_state, INF):
                        dp[new_state] = new_cost
                        parent[new_state] = state

    full = (1 << n) - 1

    # open path: pick best end (prev, cur)
    best_state = None
    best_cost = INF
    for prev in range(0, n + 1):
        for cur in range(1, n + 1):
            st = (full, prev, cur)
            if st in dp and dp[st] < best_cost:
                best_cost = dp[st]
                best_state = st

    # reconstruct indices
    path = []
    st = best_state
    while st is not None:
        mask, prev, cur = st
        path.append(cur)
        st = parent[st]
    path.reverse()

    return best_cost, [0] + path

# ============================================================
# 2) GRAPH + SHORTEST PATH (Dijkstra)
# ============================================================

def build_graph(edges: List[Tuple[str, str, float]]) -> Dict[str, List[Tuple[str, float]]]:
    g: Dict[str, List[Tuple[str, float]]] = {}
    for u, v, w in edges:
        g.setdefault(u, []).append((v, w))
        g.setdefault(v, []).append((u, w))
    return g

GRAPH = build_graph(EDGES)
NODE_NAME = {n["id"]: n["name"] for n in NODES}

def dijkstra_with_prev(graph: Dict[str, List[Tuple[str, float]]], start: str) -> Tuple[Dict[str, float], Dict[str, Optional[str]]]:
    dist = {start: 0.0}
    prev: Dict[str, Optional[str]] = {start: None}
    pq = [(0.0, start)]

    while pq:
        d, u = heapq.heappop(pq)
        if d != dist.get(u, float("inf")):
            continue
        for v, w in graph.get(u, []):
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return dist, prev

def reconstruct_path(prev: Dict[str, Optional[str]], start: str, end: str) -> List[str]:
    if end not in prev:
        return []
    path = []
    cur: Optional[str] = end
    while cur is not None:
        path.append(cur)
        cur = prev.get(cur)
    path.reverse()
    if path and path[0] == start:
        return path
    return []


# ============================================================
# 3) MULTI-STOP ROUTE (Held–Karp for OPEN PATH)
#    - Visits all selected nodes, stops at last node (no return).
# ============================================================

@dataclass
class PlanResult:
    start: str
    selected: List[str]
    visit_order: List[str]          # order in selected-space (nodes)
    expanded_route: List[str]       # full node-by-node route
    total_cost: float
    legs: List[Dict[str, Any]]      # per-hop details

def held_karp_open_path(dist_mat: List[List[float]]) -> Tuple[float, List[int]]:
    """
    dist_mat is (n+1)x(n+1) where index 0 is start S, and 1..n are selected nodes.
    Returns: best_cost, best_path_indices including start index 0 and then selected indices.
    """
    n = len(dist_mat) - 1
    if n <= 0:
        return 0.0, [0]

    # dp[mask][j] = min cost to start at 0, visit subset mask (over 1..n), and end at j (1..n)
    # mask bit i corresponds to node (i+1)
    dp: Dict[Tuple[int, int], float] = {}
    parent: Dict[Tuple[int, int], int] = {}

    # init: visit one node j
    for j in range(1, n + 1):
        mask = 1 << (j - 1)
        dp[(mask, j)] = dist_mat[0][j]
        parent[(mask, j)] = 0

    # iterate masks
    for mask in range(1, 1 << n):
        for j in range(1, n + 1):
            if not (mask & (1 << (j - 1))):
                continue
            prev_mask = mask ^ (1 << (j - 1))
            if prev_mask == 0:
                continue
            best = float("inf")
            best_k = -1
            for k in range(1, n + 1):
                if not (prev_mask & (1 << (k - 1))):
                    continue
                cand = dp[(prev_mask, k)] + dist_mat[k][j]
                if cand < best:
                    best = cand
                    best_k = k
            dp[(mask, j)] = best
            parent[(mask, j)] = best_k

    full = (1 << n) - 1
    # open path: choose best endpoint j
    end_j = min(range(1, n + 1), key=lambda j: dp[(full, j)])
    best_cost = dp[(full, end_j)]

    # reconstruct order (indices)
    order = [end_j]
    mask = full
    cur = end_j
    while True:
        p = parent[(mask, cur)]
        if p == 0:
            break
        order.append(p)
        mask ^= 1 << (cur - 1)
        cur = p
    order.reverse()

    return best_cost, [0] + order

def apply_reversal_penalty(expanded_route: List[str], base_cost: float, penalty: float) -> float:
    if penalty <= 0:
        return base_cost
    extra = 0.0
    # reversal pattern: X->Y then Y->X (i-2 == i)
    for i in range(2, len(expanded_route)):
        if expanded_route[i] == expanded_route[i - 2] and expanded_route[i - 1] != expanded_route[i]:
            extra += penalty
    return base_cost + extra

def plan_route(start: str, selected: List[str]) -> PlanResult:
    selected = [s for s in selected if s != start]
    selected = list(dict.fromkeys(selected))  # de-dup preserving order

    # If nothing selected, route is just start.
    if not selected:
        return PlanResult(
            start=start,
            selected=[],
            visit_order=[],
            expanded_route=[start],
            total_cost=0.0,
            legs=[]
        )

    # Precompute shortest paths between start + selected nodes
    points = [start] + selected  # index 0 = start
    pair_dist: Dict[Tuple[str, str], float] = {}
    pair_path: Dict[Tuple[str, str], List[str]] = {}

    # run dijkstra from each point
    dijkstra_cache = {}
    for p in points:
        dist, prev = dijkstra_with_prev(GRAPH, p)
        dijkstra_cache[p] = (dist, prev)

    # build distance matrix
    n = len(selected)
    dist_mat = [[float("inf")] * (n + 1) for _ in range(n + 1)]
    for i, a in enumerate(points):
        dist_a, prev_a = dijkstra_cache[a]
        for j, b in enumerate(points):
            if i == j:
                dist_mat[i][j] = 0.0
                pair_path[(a, b)] = [a]
                pair_dist[(a, b)] = 0.0
            else:
                if b in dist_a:
                    dist_mat[i][j] = dist_a[b]
                    pair_dist[(a, b)] = dist_a[b]
                    pair_path[(a, b)] = reconstruct_path(prev_a, a, b)
                else:
                    # unreachable
                    dist_mat[i][j] = float("inf")
                    pair_dist[(a, b)] = float("inf")
                    pair_path[(a, b)] = []

    # Check reachability
    for s in selected:
        if pair_dist[(start, s)] == float("inf"):
            raise ValueError(f"Selected node {s} is unreachable from start {start}")

    idx_to_node = {0: start}
    for k in range(1, n + 1):
        idx_to_node[k] = selected[k - 1]

    best_cost, best_idx_path = held_karp_open_path_no_uturn(dist_mat, idx_to_node)

    # Convert index route to node order
    idx_to_node = {0: start}
    for k in range(1, n + 1):
        idx_to_node[k] = selected[k - 1]

    visit_order = [idx_to_node[i] for i in best_idx_path[1:]]

    # Expand into full path using stored shortest paths
    expanded: List[str] = [start]
    legs: List[Dict[str, Any]] = []

    cur = start
    base_total = 0.0
    for nxt in visit_order:
        p = pair_path[(cur, nxt)]
        if not p:
            raise ValueError(f"No path found between {cur} and {nxt}")
        # append but avoid duplicating cur
        expanded += p[1:]
        leg_cost = pair_dist[(cur, nxt)]
        base_total += leg_cost
        legs.append({
            "from": cur,
            "to": nxt,
            "cost": leg_cost,
            "path": p
        })
        cur = nxt

    # Optional “no up/down” feel via reversal penalty (purely for scoring/display)
    total_cost = apply_reversal_penalty(expanded, base_total, REVERSAL_PENALTY)

    return PlanResult(
        start=start,
        selected=selected,
        visit_order=visit_order,
        expanded_route=expanded,
        total_cost=total_cost,
        legs=legs
    )


# ============================================================
# 4) ROBOT EXECUTOR (STUBS you will replace later)
# ============================================================

def robot_go_to(next_node: str) -> None:
    # TODO: replace with real MechDog traversal logic
    print(f"[ROBOT] Moving to {next_node} ({NODE_NAME.get(next_node, next_node)})")
    time.sleep(0.5)

def robot_take_photo(at_node: str) -> str:
    # TODO: replace with real camera capture
    # For demo: just create a placeholder file name
    ts = time.strftime("%Y%m%d-%H%M%S")
    filename = f"photo_{at_node}_{ts}.jpg"
    # (You could actually generate a dummy file if you want)
    print(f"[ROBOT] Taking photo at {at_node} -> {filename}")
    return filename


# ============================================================
# 5) FLASK UI + API
# ============================================================

app = Flask(__name__)

INDEX_HTML = """
<!doctype html>
<html>
<head>
  <meta charset="utf-8"/>
  <title>MechDog Route Planner Demo</title>
  <style>
    body { font-family: Arial, sans-serif; margin: 24px; }
    .row { display: flex; gap: 24px; }
    .card { border: 1px solid #ddd; border-radius: 10px; padding: 16px; width: 420px; }
    /* Wider card variant to fit larger SVGs without overflow */
    .card.wide { width: 520px; }
    .small { color: #666; font-size: 13px; }
    button { padding: 10px 14px; cursor: pointer; border-radius: 8px; border: 1px solid #333; background: #fff; }
    button.primary { background: #111; color: #fff; border-color: #111; }
    pre { background: #f6f6f6; padding: 12px; border-radius: 8px; overflow: auto; }
    .pill { display:inline-block; padding:2px 8px; border-radius:999px; background:#eee; margin-right:6px; }
    svg { background: #fafafa; border-radius: 10px; border: 1px solid #ddd; }
    .node { fill: #fff; stroke: #111; stroke-width: 2; }
    .nodeText { font-size: 12px; fill: #111; }
    .edge { stroke: #bbb; stroke-width: 3; }
    .route { stroke: #111; stroke-width: 4; fill: none; stroke-linecap: round; stroke-linejoin: round; }
    .dog { fill: #e11d48; stroke: #111; stroke-width: 2; }
    /* edge weight labels */
    .edgeLabel {
      font-size: 12px;
      fill: #111;
      paint-order: stroke;
      stroke: #fff;
      stroke-width: 6px; /* white halo for readability */
      pointer-events: none;
    }
  </style>
</head>
<body>
  <h2>MechDog Route Planner Demo</h2>
  <div class="small">Select locations → compute shortest open route (stop at last) → execute (demo stubs) with photo at each selected location.</div>

  <div class="row" style="margin-top:16px;">
    <div class="card">
      <h3>Inputs</h3>

      <div style="margin-bottom:10px;">
        <label><b>Start node:</b></label>
        <select id="start">
          {# Render all nodes but keep the default start node as the last option #}
          {% for n in nodes %}
            {% if n.id != default_start %}
              <option value="{{n.id}}">{{n.id}} — {{n.name}}</option>
            {% endif %}
          {% endfor %}
          {# Default start shown last and selected #}
          {% for n in nodes %}
            {% if n.id == default_start %}
              <option value="{{n.id}}" selected>{{n.id}} — {{n.name}}</option>
            {% endif %}
          {% endfor %}
        </select>
      </div>

      <div><b>Select locations to visit:</b></div>
      <div class="small">Dog will take a photo at each selected location.</div>

      <div style="margin-top:10px;">
        {% for n in nodes %}
          <div>
            <label>
              <input type="checkbox" class="loc" value="{{n.id}}">
              <span class="pill">{{n.id}}</span> {{n.name}}
            </label>
          </div>
        {% endfor %}
      </div>

      <div style="margin-top:14px; display:flex; gap:10px;">
        <button class="primary" onclick="plan()">Plan Route</button>
        <button onclick="executePlan()">Execute (Demo)</button>
      </div>

      <div class="small" style="margin-top:10px;">
        Note: “Execute” is currently simulated. Replace robot stubs with MechDog motion + real camera capture.
      </div>
    </div>

    <div class="card">
      <h3>Plan Output</h3>
      <div id="summary" class="small">No plan yet.</div>
      <pre id="output"></pre>
    </div>

    <div class="card wide">
        <div style="margin:10px 0; width: 100%; display: flex; flex-direction: column; gap: 10px;">
            <b>Visualization</b>
            <div class="small">Route line + animated marker</div>
            <svg id="map" width="520" height="380" viewBox="0 0 520 380"></svg>
        </div>
    </div>
</div>

</div>

<script>
let lastPlan = null;

// server-rendered data
const NODES = {{ nodes|tojson }};
const EDGES = {{ edges|tojson }};

const pos = {};
NODES.forEach(n => pos[n.id] = {x: n.x, y: n.y});

const svg = document.getElementById("map");

// ---------- SVG helpers ----------
function el(name, attrs={}) {
  const e = document.createElementNS("http://www.w3.org/2000/svg", name);
  Object.entries(attrs).forEach(([k,v]) => e.setAttribute(k, v));
  return e;
}

function clearSVG() {
  while (svg.firstChild) svg.removeChild(svg.firstChild);
}

function drawBaseMap() {
  clearSVG();

  // edges first (with weight labels)
  EDGES.forEach(([u,v,w]) => {
    if (!pos[u] || !pos[v]) return;
    const x1 = pos[u].x, y1 = pos[u].y, x2 = pos[v].x, y2 = pos[v].y;
    svg.appendChild(el("line", {
      x1, y1, x2, y2, class: "edge"
    }));

    // weight label at midpoint; small offset perpendicular for readability
    const mx = (x1 + x2) / 2;
    const my = (y1 + y2) / 2;
    // perpendicular offset (scaled)
    const dx = x2 - x1;
    const dy = y2 - y1;
    const len = Math.hypot(dx, dy) || 1;
    const ox = -dy / len * 8; // 8px offset
    const oy = dx / len * 8;

    const lbl = el("text", {
      x: mx + ox,
      y: my + oy,
      class: "edgeLabel",
      "text-anchor": "middle",
      dy: "4"
    });
    lbl.textContent = String(w);
    svg.appendChild(lbl);
  });

  // nodes on top
  NODES.forEach(n => {
    svg.appendChild(el("circle", {cx: n.x, cy: n.y, r: 14, class: "node"}));
    svg.appendChild(el("text", {
      x: n.x + 18, y: n.y + 4, class: "nodeText"
    })).textContent = `${n.id} — ${n.name}`;
  });
}

drawBaseMap();

// ---------- UI helpers ----------
function getSelected() {
  return Array.from(document.querySelectorAll(".loc"))
    .filter(cb => cb.checked)
    .map(cb => cb.value);
}

// ---------- Route drawing + animation ----------
let routePathEl = null;
let dogEl = null;
let animReq = null;

function stopAnim() {
  if (animReq) cancelAnimationFrame(animReq);
  animReq = null;
}

function drawRoute(expandedRoute) {
  // Remove old route/dog
  if (routePathEl) routePathEl.remove();
  if (dogEl) dogEl.remove();
  stopAnim();

  // Convert expanded route nodes -> points
  const pts = expandedRoute
    .map(id => pos[id])
    .filter(p => !!p);

  if (pts.length < 2) return;

  const d = smoothPathFromPoints(pts, 0.35); // try 0.25..0.5

  routePathEl = el("path", { d, class: "route" });
  svg.appendChild(routePathEl);

  dogEl = el("circle", { r: 10, class: "dog" });
  svg.appendChild(dogEl);

  // Start dog at beginning
  dogEl.setAttribute("cx", pts[0].x);
  dogEl.setAttribute("cy", pts[0].y);
}

function animateDog(durationMs = 4000) {
  if (!routePathEl || !dogEl) return;

  const total = routePathEl.getTotalLength();
  const start = performance.now();

  function step(t) {
    const elapsed = t - start;
    const k = Math.min(elapsed / durationMs, 1);
    const len = total * k;
    const p = routePathEl.getPointAtLength(len);
    dogEl.setAttribute("cx", p.x);
    dogEl.setAttribute("cy", p.y);

    if (k < 1) {
      animReq = requestAnimationFrame(step);
    }
  }
  animReq = requestAnimationFrame(step);
}

// Helper: update the Start select so newStart appears last and is selected
function updateStartSelect(newStart) {
  if (!newStart) return;
  const startSel = document.getElementById("start");
  // Rebuild options so that the new start appears as the last option (and is selected).
  startSel.innerHTML = "";
  NODES.forEach(n => {
    if (n.id === newStart) return;
    const o = document.createElement("option");
    o.value = n.id;
    o.text = `${n.id} — ${n.name}`;
    startSel.add(o);
  });
  const ns = NODES.find(x => x.id === newStart);
  const o2 = document.createElement("option");
  o2.value = newStart;
  o2.text = `${newStart} — ${ns ? ns.name : newStart}`;
  o2.selected = true;
  startSel.add(o2);
  startSel.value = newStart;
  if (lastPlan) lastPlan.start = newStart;
}

// ---------- API calls ----------
async function plan() {
  const start = document.getElementById("start").value;
  const selected = getSelected().filter(x => x !== start);

  const res = await fetch("/api/plan", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({start, selected})
  });

  const data = await res.json();
  lastPlan = data;

  if (!res.ok) {
    document.getElementById("summary").innerText = "Error planning route.";
    document.getElementById("output").innerText = JSON.stringify(data, null, 2);
    drawBaseMap();
    return;
  }

  document.getElementById("summary").innerText =
    `Visit order: ${data.visit_order.join(" → ")} | Total cost: ${data.total_cost}`;

  document.getElementById("output").innerText = JSON.stringify(data, null, 2);

  // Visualization
  drawBaseMap();
  drawRoute(data.expanded_route);
  animateDog(4500); // adjust speed
  // If plan produced a visit order, set its last node as the new start in the selector
  if (data.visit_order && data.visit_order.length > 0) {
    const newStart = data.visit_order[data.visit_order.length - 1];
    console.log("plan() setting new start to", newStart);
    updateStartSelect(newStart);
  }
}

async function executePlan() {
  if (!lastPlan || !lastPlan.visit_order) {
    alert("Plan a route first.");
    return;
  }

  const res = await fetch("/api/execute", {
    method: "POST",
    headers: {"Content-Type": "application/json"},
    body: JSON.stringify({plan: lastPlan})
  });

  const data = await res.json();

  if (!res.ok) {
    document.getElementById("summary").innerText = "Error executing route.";
    document.getElementById("output").innerText = JSON.stringify(data, null, 2);
    return;
  }

  document.getElementById("summary").innerText =
    `Execution complete. Photos taken: ${data.photos.length}`;

  document.getElementById("output").innerText = JSON.stringify(data, null, 2);
  // If server returned a new start node (last destination), update the start selector.
  if (data.new_start) {
    console.log("executePlan response:", data);
    updateStartSelect(data.new_start);
  }
}
function smoothPathFromPoints(points, tension = 0.35) {
  // points: [{x,y}, ...]
  if (points.length < 2) return "";

  const p = points;

  // Helper: control points for cubic bezier between p1 -> p2
  function controlPoints(p0, p1, p2, p3) {
    // Using a Catmull-Rom like approach
    const d01 = Math.hypot(p1.x - p0.x, p1.y - p0.y);
    const d12 = Math.hypot(p2.x - p1.x, p2.y - p1.y);
    const d23 = Math.hypot(p3.x - p2.x, p3.y - p2.y);

    const fa = tension * d01 / (d01 + d12 || 1);
    const fb = tension * d23 / (d12 + d23 || 1);

    const c1x = p1.x + fa * (p2.x - p0.x);
    const c1y = p1.y + fa * (p2.y - p0.y);

    const c2x = p2.x - fb * (p3.x - p1.x);
    const c2y = p2.y - fb * (p3.y - p1.y);

    return [{x: c1x, y: c1y}, {x: c2x, y: c2y}];
  }

  let d = `M ${p[0].x} ${p[0].y}`;

  for (let i = 0; i < p.length - 1; i++) {
    const p0 = p[Math.max(0, i - 1)];
    const p1 = p[i];
    const p2 = p[i + 1];
    const p3 = p[Math.min(p.length - 1, i + 2)];

    const [c1, c2] = controlPoints(p0, p1, p2, p3);
    d += ` C ${c1.x} ${c1.y}, ${c2.x} ${c2.y}, ${p2.x} ${p2.y}`;
  }

  return d;
}

</script>
</body>
</html>
"""


@app.get("/")
def index():
    return render_template_string(
        INDEX_HTML,
        nodes=NODES,
        default_start=DEFAULT_START_NODE,
        edges=EDGES
    )

@app.post("/api/plan")
def api_plan():
    payload = request.get_json(force=True) or {}
    start = payload.get("start") or DEFAULT_START_NODE
    selected = payload.get("selected") or []

    # validate
    node_ids = set(NODE_NAME.keys())
    if start not in node_ids:
        return jsonify({"error": f"Invalid start node: {start}"}), 400
    for s in selected:
        if s not in node_ids:
            return jsonify({"error": f"Invalid selected node: {s}"}), 400

    try:
        plan = plan_route(start, selected)
    except Exception as e:
        return jsonify({"error": str(e)}), 400

    return jsonify({
        "start": plan.start,
        "selected": plan.selected,
        "visit_order": plan.visit_order,
        "expanded_route": plan.expanded_route,
        "total_cost": round(plan.total_cost, 3),
        "legs": plan.legs,
    })

@app.post("/api/execute")
def api_execute():
    payload = request.get_json(force=True) or {}
    plan = payload.get("plan")
    if not plan or "visit_order" not in plan:
        return jsonify({"error": "Missing plan"}), 400

    start = plan.get("start")
    visit_order = plan.get("visit_order", [])
    if not isinstance(visit_order, list):
        return jsonify({"error": "visit_order must be a list"}), 400

    # Demo execution:
    # - move through visit_order
    # - take photo at each visited location
    photos = []
    try:
        cur = start
        for loc in visit_order:
            robot_go_to(loc)
            photos.append({"location": loc, "file": robot_take_photo(loc)})
            cur = loc
    except Exception as e:
        return jsonify({"error": f"Execution failed: {e}"}), 500
    # 'cur' now holds the last visited location (or start if none)
    return jsonify({
        "status": "ok",
        "photos": photos,
        "new_start": cur
    })

if __name__ == "__main__":
    # For local demo
    app.run(host="0.0.0.0", port=int(os.environ.get("PORT", 5000)), debug=True)