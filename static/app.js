let lastPlan = null;

// NODES and EDGES are provided by a small inline JSON blob in the template.
// If global variables aren't present, parse the JSON blob with id="server-data".
(function() {
  if (typeof NODES === 'undefined') {
    const sd = document.getElementById('server-data');
    if (sd) {
      try {
        const obj = JSON.parse(sd.textContent || sd.innerText || '{}');
        window.NODES = obj.nodes || [];
        window.EDGES = obj.edges || [];
      } catch (e) {
        window.NODES = [];
        window.EDGES = [];
        console.error('Failed to parse server-data', e);
      }
    } else {
      window.NODES = [];
      window.EDGES = [];
    }
  }
})();

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
    const dx = x2 - x1;
    const dy = y2 - y1;
    const len = Math.hypot(dx, dy) || 1;
    const ox = -dy / len * 8;
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
  if (routePathEl) routePathEl.remove();
  if (dogEl) dogEl.remove();
  stopAnim();

  const pts = expandedRoute
    .map(id => pos[id])
    .filter(p => !!p);

  if (pts.length < 2) return;

  const d = smoothPathFromPoints(pts, 0.35);

  routePathEl = el("path", { d, class: "route" });
  svg.appendChild(routePathEl);

  dogEl = el("circle", { r: 10, class: "dog" });
  svg.appendChild(dogEl);
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
  drawBaseMap();
  drawRoute(data.expanded_route);
  animateDog(4500);
  // Close Inputs modal if it's open (small-screen flow).
  const inputsModalEl = document.getElementById('inputsModal');
  if (inputsModalEl && inputsModalEl.classList.contains('show') && window.bootstrap && bootstrap.Modal) {
    try {
      bootstrap.Modal.getOrCreateInstance(inputsModalEl).hide();
    } catch (e) {
      console.warn('Failed to hide inputs modal', e);
    }
  }

  // Do not auto-open the Plan Output modal after planning on small screens;
  // the plan card is updated in-place. Users can open Output explicitly.
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
  // On small screens (below md breakpoint), open the Plan Output modal so the user sees results.
  if (window.innerWidth < 768) {
    const outModalEl = document.getElementById('outputModal');
    if (outModalEl && window.bootstrap && bootstrap.Modal) {
      bootstrap.Modal.getOrCreateInstance(outModalEl).show();
    }
  }
  if (data.new_start) {
    console.log("executePlan response:", data);
    updateStartSelect(data.new_start);
  }
}

function smoothPathFromPoints(points, tension = 0.35) {
  if (points.length < 2) return "";
  const p = points;
  function controlPoints(p0, p1, p2, p3) {
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

// Move inputs card into modal on small screens to avoid duplicating form IDs.
// When modal opens we append the actual card into the modal body; on close we restore it.
document.addEventListener('DOMContentLoaded', () => {
  // Inputs card <-> inputsModal
  const inputsCard = document.getElementById('inputsCard');
  const inputsModal = document.getElementById('inputsModal');
  if (inputsCard && inputsModal) {
    const inputsOriginalParent = inputsCard.parentNode;
    const inputsNextSibling = inputsCard.nextSibling;
    const inputsModalBody = inputsModal.querySelector('.modal-body');

    inputsModal.addEventListener('show.bs.modal', () => {
      inputsCard.classList.remove('d-none', 'd-md-block');
      inputsCard.classList.add('d-block');
      inputsModalBody.appendChild(inputsCard);
    });

    inputsModal.addEventListener('hide.bs.modal', () => {
      inputsCard.classList.remove('d-block');
      inputsCard.classList.add('d-none', 'd-md-block');
      if (inputsNextSibling) inputsOriginalParent.insertBefore(inputsCard, inputsNextSibling);
      else inputsOriginalParent.appendChild(inputsCard);
    });
  }

  // Plan Output card <-> outputModal
  const planCard = document.getElementById('planCard');
  const outputModal = document.getElementById('outputModal');
  if (planCard && outputModal) {
    const planOriginalParent = planCard.parentNode;
    const planNextSibling = planCard.nextSibling;
    const outputModalBody = outputModal.querySelector('.modal-body');

    outputModal.addEventListener('show.bs.modal', () => {
      planCard.classList.remove('d-none', 'd-md-block');
      planCard.classList.add('d-block');
      outputModalBody.appendChild(planCard);
    });

    outputModal.addEventListener('hide.bs.modal', () => {
      planCard.classList.remove('d-block');
      planCard.classList.add('d-none', 'd-md-block');
      if (planNextSibling) planOriginalParent.insertBefore(planCard, planNextSibling);
      else planOriginalParent.appendChild(planCard);
    });
  }
});

