
import heapq
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional, Any

from flask import Flask, request, jsonify, render_template

# ============================================================
# 1) MAP CONFIG (predefined locations + predefined paths)
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
    ("H", "G", 2.3),
]

DEFAULT_START_NODE = "A"
REVERSAL_PENALTY = 2.0

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

UTURN_HARD_LIMIT_DEG = 165
UTURN_PENALTY = 80.0

def held_karp_open_path_no_uturn(dist_mat, idx_to_node):
    n = len(dist_mat) - 1
    if n <= 0:
        return 0.0, [0]
    INF = 1e18
    dp = {}
    parent = {}
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
                    if prev != 0 and nxt == prev:
                        continue
                    turn_cost = 0.0
                    if prev != 0:
                        a = idx_to_node[prev]
                        b = idx_to_node[cur]
                        c = idx_to_node[nxt]
                        ang = turn_angle_deg(a, b, c)
                        if UTURN_HARD_LIMIT_DEG is not None and ang >= UTURN_HARD_LIMIT_DEG:
                            continue
                        if ang >= 150:
                            turn_cost += UTURN_PENALTY
                    new_mask = mask | (1 << (nxt - 1))
                    new_state = (new_mask, cur, nxt)
                    new_cost = dp[state] + dist_mat[cur][nxt] + turn_cost
                    if new_cost < dp.get(new_state, INF):
                        dp[new_state] = new_cost
                        parent[new_state] = state
    full = (1 << n) - 1
    best_state = None
    best_cost = INF
    for prev in range(0, n + 1):
        for cur in range(1, n + 1):
            st = (full, prev, cur)
            if st in dp and dp[st] < best_cost:
                best_cost = dp[st]
                best_state = st
    path = []
    st = best_state
    while st is not None:
        mask, prev, cur = st
        path.append(cur)
        st = parent[st]
    path.reverse()
    return best_cost, [0] + path

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

@dataclass
class PlanResult:
    start: str
    selected: List[str]
    visit_order: List[str]
    expanded_route: List[str]
    total_cost: float
    legs: List[Dict[str, Any]]

def held_karp_open_path(dist_mat: List[List[float]]) -> Tuple[float, List[int]]:
    n = len(dist_mat) - 1
    if n <= 0:
        return 0.0, [0]
    dp: Dict[Tuple[int, int], float] = {}
    parent: Dict[Tuple[int, int], int] = {}
    for j in range(1, n + 1):
        mask = 1 << (j - 1)
        dp[(mask, j)] = dist_mat[0][j]
        parent[(mask, j)] = 0
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
    end_j = min(range(1, n + 1), key=lambda j: dp[(full, j)])
    best_cost = dp[(full, end_j)]
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
    for i in range(2, len(expanded_route)):
        if expanded_route[i] == expanded_route[i - 2] and expanded_route[i - 1] != expanded_route[i]:
            extra += penalty
    return base_cost + extra

def plan_route(start: str, selected: List[str]) -> PlanResult:
    selected = [s for s in selected if s != start]
    selected = list(dict.fromkeys(selected))
    if not selected:
        return PlanResult(start=start, selected=[], visit_order=[], expanded_route=[start], total_cost=0.0, legs=[])
    points = [start] + selected
    pair_dist: Dict[Tuple[str, str], float] = {}
    pair_path: Dict[Tuple[str, str], List[str]] = {}
    dijkstra_cache = {}
    for p in points:
        dist, prev = dijkstra_with_prev(GRAPH, p)
        dijkstra_cache[p] = (dist, prev)
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
                    dist_mat[i][j] = float("inf")
                    pair_dist[(a, b)] = float("inf")
                    pair_path[(a, b)] = []
    for s in selected:
        if pair_dist[(start, s)] == float("inf"):
            raise ValueError(f"Selected node {s} is unreachable from start {start}")
    idx_to_node = {0: start}
    for k in range(1, n + 1):
        idx_to_node[k] = selected[k - 1]
    best_cost, best_idx_path = held_karp_open_path_no_uturn(dist_mat, idx_to_node)
    idx_to_node = {0: start}
    for k in range(1, n + 1):
        idx_to_node[k] = selected[k - 1]
    visit_order = [idx_to_node[i] for i in best_idx_path[1:]]
    expanded: List[str] = [start]
    legs: List[Dict[str, Any]] = []
    cur = start
    base_total = 0.0
    for nxt in visit_order:
        p = pair_path[(cur, nxt)]
        if not p:
            raise ValueError(f"No path found between {cur} and {nxt}")
        expanded += p[1:]
        leg_cost = pair_dist[(cur, nxt)]
        base_total += leg_cost
        legs.append({"from": cur, "to": nxt, "cost": leg_cost, "path": p})
        cur = nxt
    total_cost = apply_reversal_penalty(expanded, base_total, REVERSAL_PENALTY)
    return PlanResult(start=start, selected=selected, visit_order=visit_order, expanded_route=expanded, total_cost=total_cost, legs=legs)

def robot_go_to(next_node: str) -> None:
    print(f"[ROBOT] Moving to {next_node} ({NODE_NAME.get(next_node, next_node)})")
    time.sleep(0.5)

def robot_take_photo(at_node: str) -> str:
    ts = time.strftime("%Y%m%d-%H%M%S")
    filename = f"photo_{at_node}_{ts}.jpg"
    print(f"[ROBOT] Taking photo at {at_node} -> {filename}")
    return filename

app = Flask(__name__)

@app.get("/")
def index():
    return render_template("index.html", nodes=NODES, default_start=DEFAULT_START_NODE, edges=EDGES)

@app.post("/api/plan")
def api_plan():
    payload = request.get_json(force=True) or {}
    start = payload.get("start") or DEFAULT_START_NODE
    selected = payload.get("selected") or []
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
    return jsonify({"start": plan.start, "selected": plan.selected, "visit_order": plan.visit_order, "expanded_route": plan.expanded_route, "total_cost": round(plan.total_cost, 3), "legs": plan.legs})

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
    photos = []
    try:
        cur = start
        for loc in visit_order:
            robot_go_to(loc)
            photos.append({"location": loc, "file": robot_take_photo(loc)})
            cur = loc
    except Exception as e:
        return jsonify({"error": f"Execution failed: {e}"}), 500
    return jsonify({"status": "ok", "photos": photos, "new_start": cur})

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=int(os.environ.get("PORT", 5000)), debug=True)

