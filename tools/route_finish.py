"""Finisher: heal the remaining net splits on the CURRENT board.

Loads Radar1.kicad_pcb as-is (no regeneration) and reuses the machinery
from route_signals.py (exec'd up to its routing phase). Strategy:
 1. remove optional GND stitch vias/stubs (frees corridors; re-added at end)
 2. per split net: reconnect clusters with A* at 0.4/0.2/0.15, rip-up allowed
    (crossed rippable nets are deleted whole and re-healed the same way)
 3. plane rails: try stitching disconnected clusters straight into In2 first
 4. re-run GND stitching, remove dangling signal tracks, refill zones, save
Run inside the KiCad snap shell. DRC afterwards is the ground truth.
"""
import math, pcbnew
from pcbnew import ToMM

SRC = "/home/christian-thomas-hearn/Desktop/X-BAND FMCW RADAR/Radar1/tools/route_signals.py"
src = open(SRC).read().split("# ---------- escape stubs")[0]
G = {}
exec(compile(src, "route_signals_core", "exec"), G)
for _k in ("b", "F", "B", "maps", "hardmaps", "softmaps", "holes", "cell",
           "xy", "mm", "build_maps", "add_track", "add_via", "hole_ok",
           "via_ok", "astar", "emit", "pad_goal_cells", "net_tree",
           "pads_by_net", "net_order", "RIPPABLE", "PLANES", "RAILS",
           "pad_rect", "NX", "NY", "STEP", "hwclass"):
    globals()[_k] = G[_k]
b = G["b"]

RF_NETS = ("RF_TX", "ANT_TX", "ANT_RX")
gcode = b.FindNet("GND").GetNetCode()

# keep python wrappers of everything we detach alive, or later b.Tracks()
# iterations hand back raw SwigPyObjects (dangling) and crash
_detached = []
def remove(item):
    _detached.append(item)
    b.Remove(item)

def plane_ok(rail, x, y):
    if rail == "GND": return True
    r = PLANES[rail]
    if r: return r[0]+0.8 < x < r[2]-0.8 and r[1]+0.8 < y < r[3]-0.8
    return not any(q[0]-0.8 < x < q[2]+0.8 and q[1]-0.8 < y < q[3]+0.8
                   for q in PLANES.values() if q)

# ---------- 1. drop optional GND stitching ----------
rf_segs = []
for t in b.Tracks():
    if isinstance(t, pcbnew.PCB_VIA): continue
    if t.GetNetname() in RF_NETS:
        s, e = t.GetStart(), t.GetEnd()
        rf_segs.append((ToMM(s.x), ToMM(s.y), ToMM(e.x), ToMM(e.y)))

def _segd(px, py, s):
    x0, y0, x1, y1 = s
    dx, dy = x1-x0, y1-y0
    L2 = dx*dx + dy*dy
    tt = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px-x0)*dx+(py-y0)*dy)/L2))
    return math.hypot(px-(x0+tt*dx), py-(y0+tt*dy))

all_pad_rects = []
for fp in b.GetFootprints():
    for p_ in fp.Pads():
        all_pad_rects.append(pad_rect(p_))

removed_gnd = 0
for t in list(b.Tracks()):
    if t.GetNetCode() != gcode: continue
    if isinstance(t, pcbnew.PCB_VIA):
        pos = t.GetPosition(); px, py = ToMM(pos.x), ToMM(pos.y)
        if any(_segd(px, py, s) < 1.0 for s in rf_segs): continue  # fence
        if any(r[0]-0.05 <= px <= r[2]+0.05 and r[1]-0.05 <= py <= r[3]+0.05
               for r in all_pad_rects): continue                   # EP vias
        remove(t); removed_gnd += 1
    elif abs(ToMM(t.GetWidth()) - 0.3) < 0.01:                     # stitch stub
        remove(t); removed_gnd += 1
print(f"optional GND stitching removed: {removed_gnd} items")

build_maps()

# ---------- cluster machinery ----------
def clusters_of(n, code):
    items = [("pad", q) for q in pads_by_net.get(n, [])]
    for tt in b.Tracks():
        if tt.GetNetCode() != code: continue
        if isinstance(tt, pcbnew.PCB_VIA):
            pos = tt.GetPosition()
            items.append(("via", (ToMM(pos.x), ToMM(pos.y),
                                  ToMM(tt.GetWidth()) / 2)))
        else:
            s, e = tt.GetStart(), tt.GetEnd()
            lay = 0 if tt.GetLayer() == F else (1 if tt.GetLayer() == B else 2)
            items.append(("trk", (ToMM(s.x), ToMM(s.y), ToMM(e.x), ToMM(e.y),
                                  ToMM(tt.GetWidth()) / 2, lay)))
    def touch(a, b_):
        (ka, ga), (kb, gb) = a, b_
        if ka == "pad" and kb == "pad": return False
        if kb == "pad": return touch(b_, a)
        if kb == "via": pts = [(gb[0], gb[1])]; hw = gb[2]; layb = None
        else:
            k = max(int(math.hypot(gb[2]-gb[0], gb[3]-gb[1]) / 0.1), 1)
            pts = [(gb[0]+(gb[2]-gb[0])*i/k, gb[1]+(gb[3]-gb[1])*i/k)
                   for i in range(k+1)]
            hw = gb[4]; layb = gb[5]
        if ka == "pad":
            r = ga[2]
            if not ga[4] and layb is not None and layb != ga[3]: return False
            return any(r[0]-hw-.01 <= x <= r[2]+hw+.01 and
                       r[1]-hw-.01 <= y <= r[3]+hw+.01 for x, y in pts)
        if ka == "via":
            return any(math.hypot(x-ga[0], y-ga[1]) <= ga[2]+hw+.01
                       for x, y in pts)
        if layb is not None and ga[5] != 2 and layb != 2 and ga[5] != layb:
            return False
        seg = (ga[0], ga[1], ga[2], ga[3])
        return any(_segd(x, y, seg) <= ga[4]+hw+.01 for x, y in pts)
    parent = list(range(len(items) + 1))
    def find(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            if find(i) != find(j) and touch(items[i], items[j]):
                parent[find(i)] = find(j)
    if n in PLANES or n == "GND":
        for i, (k, g) in enumerate(items):
            if k == "via" and plane_ok(n, g[0], g[1]):
                parent[find(i)] = find(len(items))
    groups = {}
    for i, it in enumerate(items):
        groups.setdefault(find(i), []).append(it)
    return groups, find(len(items))

def cluster_cells(members, code):
    out = set()
    for kind, g in members:
        if kind == "pad":
            out |= set(pad_goal_cells(g[0], g[1], g[2], g[3], code, 0.1, g[4]))
        elif kind == "via":
            ix, iy = cell(g[0], g[1])
            if 0 <= ix < NX and 0 <= iy < NY:
                out |= {(0, ix, iy), (1, ix, iy)}
        else:
            if g[5] == 2: continue
            k = max(int(math.hypot(g[2]-g[0], g[3]-g[1]) / STEP), 1)
            for i in range(k + 1):
                ix, iy = cell(g[0]+(g[2]-g[0])*i/k, g[1]+(g[3]-g[1])*i/k)
                if 0 <= ix < NX and 0 <= iy < NY: out.add((g[5], ix, iy))
    return out

def stitch_pad(px, py, rect, code, rail):
    rx = max(rect[2]-rect[0], rect[3]-rect[1]) / 2
    for rad in (rx+0.55, rx+0.75, rx+1.0, rx+1.4, rx+1.9, rx+2.5):
        for a in range(16):
            vx = px + rad*math.cos(a*math.pi/8)
            vy = py + rad*math.sin(a*math.pi/8)
            ix, iy = cell(vx, vy)
            if not (1 <= ix < NX-1 and 1 <= iy < NY-1): continue
            if not via_ok(code, 0.1, ix, iy): continue
            vx, vy = xy(ix, iy)
            if not plane_ok(rail, vx, vy): continue
            # stub corridor check on the power-class map
            m = G["maps"][(0, 0.2)]
            k = max(int(math.hypot(vx-px, vy-py) / 0.1), 1)
            ok = True
            for i in range(k+1):
                jx, jy = cell(px+(vx-px)*i/k, py+(vy-py)*i/k)
                if m[jx*NY+jy] not in (0, code): ok = False; break
            if not ok: continue
            add_track(px, py, vx, vy, 0.4, 0, code)
            add_via(vx, vy, code)
            return True
    return False

code2name = {b.FindNet(n).GetNetCode(): n for n in pads_by_net if b.FindNet(n)}

def rip_near_path(victims, path):
    """remove only victim tracks crossing the healing path's corridor."""
    pathcells = {}
    for (lay, ix, iy) in path:
        pathcells.setdefault(lay, set()).add((ix, iy))
    k = 0
    vcodes = {b.FindNet(v).GetNetCode() for v in victims}
    for tt in list(b.Tracks()):
        if tt.GetNetCode() not in vcodes: continue
        if isinstance(tt, pcbnew.PCB_VIA): continue
        lay = 0 if tt.GetLayer() == F else (1 if tt.GetLayer() == B else None)
        if lay is None or lay not in pathcells: continue
        s, e = tt.GetStart(), tt.GetEnd()
        x0, y0, x1, y1 = ToMM(s.x), ToMM(s.y), ToMM(e.x), ToMM(e.y)
        n_ = max(int(math.hypot(x1-x0, y1-y0) / STEP), 1)
        hit = False
        for i in range(n_ + 1):
            ix, iy = cell(x0+(x1-x0)*i/n_, y0+(y1-y0)*i/n_)
            for dx in (-2, -1, 0, 1, 2):
                for dy in (-2, -1, 0, 1, 2):
                    if (ix+dx, iy+dy) in pathcells[lay]: hit = True; break
                if hit: break
            if hit: break
        if hit:
            remove(tt); k += 1
    return k

# ---------- 2./3. heal queue with rip ----------
def heal_net(n, allow_rip):
    """returns (fully_healed, victims_ripped)"""
    code = b.FindNet(n).GetNetCode()
    groups, planeroot = clusters_of(n, code)
    padded = [(root, m) for root, m in groups.items()
              if any(k == "pad" for k, _ in m)]
    if len(padded) <= 1: return True, set()
    padded.sort(key=lambda t: (t[0] != planeroot,
                               -sum(1 for k, _ in t[1] if k == "pad")))
    # plane rails: try stitching stray clusters straight into the plane
    if n in PLANES:
        rest = []
        for root, m in padded[1:]:
            done = False
            for kind, g in m:
                if kind == "pad" and not g[4] and \
                   stitch_pad(g[0], g[1], g[2], code, n):
                    done = True; break
            if not done: rest.append((root, m))
        if not rest: return True, set()
        padded = padded[:1] + rest
    main_cells = cluster_cells(padded[0][1], code)
    w = 0.4 if n in RAILS else 0.2
    widths = (w, 0.2, 0.15) if w == 0.4 else (0.2, 0.15)
    for root, members in padded[1:]:
        goals = cluster_cells(members, code)
        srcs = main_cells - goals
        done = False
        for ww in widths:
            path, ripped = astar(code, ww/2, srcs, goals)
            if path and not ripped:
                emit(path, ww, code, None)
                main_cells |= set(path) | goals
                done = True; break
        if done: continue
        if allow_rip:
            for ww in widths:
                path, ripped = astar(code, ww/2, srcs, goals, rip=True)
                if path and ripped:
                    victims = {code2name[c] for c in ripped
                               if c in code2name} - {n}
                    if victims:
                        return False, (victims, path)
        return False, None
    return True, None

splits = []
for n in sorted(set(list(RIPPABLE) + list(PLANES))):
    groups, _ = clusters_of(n, b.FindNet(n).GetNetCode())
    padded = [m for m in groups.values() if any(k == "pad" for k, _ in m)]
    if len(padded) > 1: splits.append(n)
print(f"split nets at start: {splits}")

queue = list(splits)
tries = {}
rip_budget = 40
unresolved = []
while queue:
    n = queue.pop(0)
    tries[n] = tries.get(n, 0) + 1
    ok, ripinfo = heal_net(n, allow_rip=False)
    if ok:
        print(f"healed {n}")
        continue
    if ripinfo:
        victims, path = ripinfo
        rip_budget -= 1
        rm = rip_near_path(victims, path)
        print(f"rip for {n}: cut {sorted(victims)} ({rm} segs, "
              f"budget {rip_budget})")
        build_maps()
        queue.insert(0, n)
        for v in victims:
            if v not in queue: queue.append(v)
    elif tries[n] <= 3:
        queue.append(n)          # try again after later heals free space
    else:
        unresolved.append(n)
print(f"unresolved nets: {unresolved}")

# ---------- diagnostics for unresolved ----------
for n in unresolved:
    code = b.FindNet(n).GetNetCode()
    groups, planeroot = clusters_of(n, code)
    padded = [(root, m) for root, m in groups.items()
              if any(k == "pad" for k, _ in m)]
    print(f"-- {n}: {len(padded)} pad-clusters")
    for root, m in padded:
        pads = [g[:2] for k, g in m if k == "pad"]
        print(f"   cluster {pads}")

# ---------- 4. re-stitch GND, cleanup, fill, save ----------
gnd_done = 0
for (px, py, rect, lay, tht) in pads_by_net.get("GND", []):
    if tht or lay != 0: continue
    if any((px-a)**2 + (py-c)**2 < 1.2**2 for a, c in G["holes"]): continue
    rx = max(rect[2]-rect[0], rect[3]-rect[1]) / 2
    placed = False
    for rad in (rx+0.55, rx+0.75, rx+1.0):
        for a in range(8):
            vx, vy = px + rad*math.cos(a*math.pi/4), py + rad*math.sin(a*math.pi/4)
            ix, iy = cell(vx, vy)
            if not (1 <= ix < NX-1 and 1 <= iy < NY-1): continue
            if not via_ok(gcode, 0.1, ix, iy): continue
            vx, vy = xy(ix, iy)
            m = G["maps"][(0, 0.1)]
            k = max(int(math.hypot(vx-px, vy-py)/0.1), 1)
            if any(m[cell(px+(vx-px)*i/k, py+(vy-py)*i/k)[0]*NY +
                     cell(px+(vx-px)*i/k, py+(vy-py)*i/k)[1]]
                   not in (0, gcode) for i in range(k+1)): continue
            add_track(px, py, vx, vy, 0.3, 0, gcode)
            add_via(vx, vy, gcode)
            gnd_done += 1; placed = True; break
        if placed: break
print(f"GND re-stitched: {gnd_done}")

# dangling cleanup for signal nets (no zones involved)
def endpoint_touches(tt, ex, ey):
    code = tt.GetNetCode()
    uid = tt.m_Uuid.AsString()
    lay = tt.GetLayer()
    for other in b.Tracks():
        if other.GetNetCode() != code: continue
        if other.m_Uuid.AsString() == uid: continue
        if isinstance(other, pcbnew.PCB_VIA):
            pos = other.GetPosition()
            if math.hypot(ToMM(pos.x)-ex, ToMM(pos.y)-ey) <= \
               ToMM(other.GetWidth())/2 + ToMM(tt.GetWidth())/2: return True
        else:
            if other.GetLayer() != lay: continue
            s, e = other.GetStart(), other.GetEnd()
            if _segd(ex, ey, (ToMM(s.x), ToMM(s.y), ToMM(e.x), ToMM(e.y))) <= \
               ToMM(other.GetWidth())/2 + ToMM(tt.GetWidth())/2: return True
    for fp in b.GetFootprints():
        for p_ in fp.Pads():
            if p_.GetNetCode() != code: continue
            r = pad_rect(p_)
            if r[0]-.05 <= ex <= r[2]+.05 and r[1]-.05 <= ey <= r[3]+.05:
                return True
    return False

removed_tail = 1
total_tail = 0
while removed_tail:
    removed_tail = 0
    for tt in list(b.Tracks()):
        if isinstance(tt, pcbnew.PCB_VIA): continue
        nname = code2name.get(tt.GetNetCode())
        if nname is None or nname in PLANES or nname == "GND": continue
        if nname in RF_NETS: continue
        s, e = tt.GetStart(), tt.GetEnd()
        for (ex, ey) in ((ToMM(s.x), ToMM(s.y)), (ToMM(e.x), ToMM(e.y))):
            if not endpoint_touches(tt, ex, ey):
                remove(tt); removed_tail += 1; total_tail += 1
                break
    total_tail += 0
print(f"dangling tails removed: {total_tail}")

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(G["BOARD"], b)
print("saved")
