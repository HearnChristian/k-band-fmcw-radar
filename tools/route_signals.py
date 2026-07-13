"""Phase C copper for Radar1.kicad_pcb: route all remaining signal/power nets.

Grid A* (0.2 mm) over F.Cu/B.Cu with through-vias, escape stubs for fine-
pitch ICs, In2 split-plane stitching for the big rails (see PLANES / the
zones in route_pcb.py), GND stitching to In1, retry rounds, and finally
rip-up & reroute: a sealed leg may cross foreign *tracks* (never pads/vias
of other nets) at high cost; the crossed nets are ripped and rerouted.
Run AFTER tools/route_pcb.py (inside the KiCad snap shell).
"""
import heapq, math
import pcbnew
from pcbnew import VECTOR2I, FromMM, ToMM

BOARD = "/home/christian-thomas-hearn/Desktop/X-BAND FMCW RADAR/Radar1/Radar1.kicad_pcb"
OX, OY, BW, BH = 60.0, 40.0, 66.0, 46.0
STEP = 0.2
NX, NY = int(BW / STEP) + 1, int(BH / STEP) + 1
EDGE = 0.5                      # copper keep-in from board edge
CLR = 0.15
VIA_D, VIA_DRL = 0.6, 0.3
VIA_COST, TURN_COST, RIP_COST = 12, 2, 300

RAILS = {"+1V8", "+3V0_PA", "+3V3_DIG", "+3V3_RF", "+VIN", "-VGG", "VBUS",
         "VCORE", "VREF", "VGG_CP", "VIN_RAW", "+5V"}
# rails carried by In2 plane regions (keep in sync with zones in route_pcb.py)
PLANES = {
    "+3V3_RF":  (59.0, 41.0,  88.0, 78.0),
    "+1V8":     (88.5, 41.0, 103.0, 78.0),
    "+3V3_DIG": (103.5, 41.0, 127.0, 78.0),
    "+5V":      None,        # base fill: anywhere the sub-zones aren't
}
SKIP = {"GND", "RF_TX", "ANT_TX", "ANT_RX", ""} | set(PLANES)

b = pcbnew.LoadBoard(BOARD)
F, B = pcbnew.F_Cu, pcbnew.B_Cu

def cell(x, y):  return (round((x - OX) / STEP), round((y - OY) / STEP))
def xy(ix, iy):  return (OX + ix * STEP, OY + iy * STEP)
def mm(x, y):    return VECTOR2I(FromMM(x), FromMM(y))

def pad_rect(p):
    bb = p.GetBoundingBox()
    return (ToMM(bb.GetLeft()), ToMM(bb.GetTop()),
            ToMM(bb.GetRight()), ToMM(bb.GetBottom()))

# ---------- net inventory ----------
pads_by_net = {}
for fp in b.GetFootprints():
    for p in fp.Pads():
        n = p.GetNetname()
        if not n or n.startswith("unconnected"): continue
        pos = p.GetPosition()
        tht = p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH, pcbnew.PAD_ATTRIB_NPTH)
        pads_by_net.setdefault(n, []).append(
            (ToMM(pos.x), ToMM(pos.y), pad_rect(p),
             0 if (p.IsOnLayer(F) or tht) else 1, tht))

# escape-limited nets (pads on fine-pitch ICs) route first, before the
# open area around those pin columns is consumed by other nets
FINE_RECTS = []
for fp in b.GetFootprints():
    if fp.GetReference() in ("U1", "U2", "U7", "U8", "U12", "J1"):
        bb = fp.GetBoundingBox()
        FINE_RECTS.append((ToMM(bb.GetLeft()), ToMM(bb.GetTop()),
                           ToMM(bb.GetRight()), ToMM(bb.GetBottom())))
def escape_limited(pl):
    return any(any(r[0] <= q[0] <= r[2] and r[1] <= q[1] <= r[3]
                   for r in FINE_RECTS) for q in pl)

net_order = []
for n, pl in pads_by_net.items():
    if n in SKIP or len(pl) < 2: continue
    xs = [q[0] for q in pl]; ys = [q[1] for q in pl]
    net_order.append((0 if escape_limited(pl) else 1,
                      max(xs) - min(xs) + max(ys) - min(ys), n))
net_order.sort()
net_order = [(sz, n) for _, sz, n in net_order]
RIPPABLE = {n for _, n in net_order}

# ---------- blocking maps ----------
# hardmaps: pads, vias, board edge, non-rippable tracks  (0/net/255)
# softmaps: tracks of rippable nets                      (0/net/255)
# maps:     union of both, what A* normally sees
HWS = (0.075, 0.1, 0.2)         # halfwidths: heal-thin / signal / power
maps = hardmaps = softmaps = None
holes = []                      # (x, y) of every drill on the board
STUB_IDS = set()                # escape-stub track objects (never ripped)

def _mark(m, k, nc):
    cur = m[k]
    m[k] = nc if cur in (0, nc) else 255

def paint(dsts, lay, hw, x0, y0, x1, y1, geomhw, net):
    r = geomhw + CLR + hw + 0.05
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    ax0, ay0 = cell(min(x0, x1) - r, min(y0, y1) - r)
    ax1, ay1 = cell(max(x0, x1) + r, max(y0, y1) + r)
    nc = net if 0 < net < 255 else 255
    for ix in range(max(ax0, 0), min(ax1, NX - 1) + 1):
        for iy in range(max(ay0, 0), min(ay1, NY - 1) + 1):
            px, py = xy(ix, iy)
            t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px-x0)*dx + (py-y0)*dy) / L2))
            if math.hypot(px - (x0 + t*dx), py - (y0 + t*dy)) < r:
                for d in dsts: _mark(d[(lay, hw)], ix * NY + iy, nc)

def paint_rect(dsts, lay, hw, bx0, by0, bx1, by1, net):
    r = CLR + hw + 0.05
    ax0, ay0 = cell(bx0 - r, by0 - r)
    ax1, ay1 = cell(bx1 + r, by1 + r)
    nc = net if 0 < net < 255 else 255
    for ix in range(max(ax0, 0), min(ax1, NX - 1) + 1):
        for iy in range(max(ay0, 0), min(ay1, NY - 1) + 1):
            px, py = xy(ix, iy)
            ddx = max(bx0 - px, 0, px - bx1)
            ddy = max(by0 - py, 0, py - by1)
            if math.hypot(ddx, ddy) < r:
                for d in dsts: _mark(d[(lay, hw)], ix * NY + iy, nc)

VIA_ENV = 0.3 + CLR + 0.02       # via copper envelope vs foreign outline

def _via_rect(bx0, by0, bx1, by1, net):
    r = VIA_ENV
    ax0, ay0 = cell(bx0 - r, by0 - r); ax1, ay1 = cell(bx1 + r, by1 + r)
    nc = net if 0 < net < 255 else 255
    for ix in range(max(ax0, 0), min(ax1, NX - 1) + 1):
        for iy in range(max(ay0, 0), min(ay1, NY - 1) + 1):
            px, py = xy(ix, iy)
            ddx = max(bx0 - px, 0, px - bx1); ddy = max(by0 - py, 0, py - by1)
            if math.hypot(ddx, ddy) < r: _mark(viamap, ix * NY + iy, nc)

def _via_seg(x0, y0, x1, y1, geomhw, net):
    r = VIA_ENV + geomhw
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    ax0, ay0 = cell(min(x0, x1) - r, min(y0, y1) - r)
    ax1, ay1 = cell(max(x0, x1) + r, max(y0, y1) + r)
    nc = net if 0 < net < 255 else 255
    for ix in range(max(ax0, 0), min(ax1, NX - 1) + 1):
        for iy in range(max(ay0, 0), min(ay1, NY - 1) + 1):
            px, py = xy(ix, iy)
            tt = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px-x0)*dx + (py-y0)*dy) / L2))
            if math.hypot(px - (x0 + tt*dx), py - (y0 + tt*dy)) < r:
                _mark(viamap, ix * NY + iy, nc)

def build_maps():
    global maps, hardmaps, softmaps, holes, viamap
    maps = {(l, h): bytearray(NX * NY) for l in (0, 1) for h in HWS}
    hardmaps = {(l, h): bytearray(NX * NY) for l in (0, 1) for h in HWS}
    softmaps = {(l, h): bytearray(NX * NY) for l in (0, 1) for h in HWS}
    viamap = bytearray(NX * NY)
    holes = []
    code2name = {b.FindNet(n).GetNetCode(): n
                 for n in list(pads_by_net) if b.FindNet(n)}
    for fp in b.GetFootprints():
        for p in fp.Pads():
            n = p.GetNetname(); code = p.GetNetCode()
            r = pad_rect(p)
            tht = p.GetAttribute() in (pcbnew.PAD_ATTRIB_PTH,
                                       pcbnew.PAD_ATTRIB_NPTH)
            if tht:
                pos = p.GetPosition()
                holes.append((ToMM(pos.x), ToMM(pos.y)))
            net = code if n and not n.startswith("unconnected") else 255
            _via_rect(r[0], r[1], r[2], r[3], net)
            for lay, on in ((0, p.IsOnLayer(F) or tht), (1, p.IsOnLayer(B) or tht)):
                if not on: continue
                for hw in HWS:
                    paint_rect((maps, hardmaps), lay, hw,
                               r[0], r[1], r[2], r[3], net)
    for t in b.Tracks():
        code = t.GetNetCode()
        soft = (code2name.get(code) in RIPPABLE
                and t.m_Uuid.AsString() not in STUB_IDS)
        dst = (maps, softmaps) if soft else (maps, hardmaps)
        if isinstance(t, pcbnew.PCB_VIA):
            pos = t.GetPosition()
            x, y = ToMM(pos.x), ToMM(pos.y)
            holes.append((x, y))
            _via_seg(x, y, x, y, VIA_D / 2, code)
            for lay in (0, 1):
                for hw in HWS:
                    paint(dst, lay, hw, x, y, x, y, VIA_D / 2, code)
        else:
            s, e = t.GetStart(), t.GetEnd()
            lay = 0 if t.GetLayer() == F else (1 if t.GetLayer() == B else None)
            if lay is None: continue
            _via_seg(ToMM(s.x), ToMM(s.y), ToMM(e.x), ToMM(e.y),
                     ToMM(t.GetWidth()) / 2, code)
            for hw in HWS:
                paint(dst, lay, hw, ToMM(s.x), ToMM(s.y), ToMM(e.x), ToMM(e.y),
                      ToMM(t.GetWidth()) / 2, code)
    for hw in HWS:                       # board edge margin
        lim = EDGE + hw
        for lay in (0, 1):
            m, hm = maps[(lay, hw)], hardmaps[(lay, hw)]
            for ix in range(NX):
                for iy in range(NY):
                    px, py = xy(ix, iy)
                    if (px - OX < lim or OX + BW - px < lim or
                            py - OY < lim or OY + BH - py < lim):
                        m[ix * NY + iy] = 255; hm[ix * NY + iy] = 255
                    if (px - OX < 0.55 or OX + BW - px < 0.55 or
                            py - OY < 0.55 or OY + BH - py < 0.55):
                        viamap[ix * NY + iy] = 255

# ---------- emit helpers (with per-net bookkeeping for rip-up) ----------
emitted = {}                    # net name -> [board objects]

def add_track(x0, y0, x1, y1, w, lay, code, book=None, stub=False):
    t = pcbnew.PCB_TRACK(b)
    t.SetStart(mm(x0, y0)); t.SetEnd(mm(x1, y1))
    t.SetWidth(FromMM(w)); t.SetLayer(F if lay == 0 else B); t.SetNetCode(code)
    b.Add(t)
    if stub:
        STUB_IDS.add(t.m_Uuid.AsString())
        STUB_LAST[0] = t
    dst = (maps, softmaps) if (book and not stub) else (maps, hardmaps)
    if book is not None and not stub: emitted.setdefault(book, []).append(t)
    _via_seg(x0, y0, x1, y1, w / 2, code)
    for hw in HWS:
        paint(dst, lay, hw, x0, y0, x1, y1, w / 2, code)

def add_via(x, y, code, book=None):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(mm(x, y)); v.SetDrill(FromMM(VIA_DRL)); v.SetWidth(FromMM(VIA_D))
    v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(F, B)
    v.SetNetCode(code); b.Add(v)
    holes.append((x, y))
    dst = (maps, softmaps) if book else (maps, hardmaps)
    if book is not None: emitted.setdefault(book, []).append(v)
    _via_seg(x, y, x, y, VIA_D / 2, code)
    for lay in (0, 1):
        for hw in HWS:
            paint(dst, lay, hw, x, y, x, y, VIA_D / 2, code)

def hole_ok(x, y):
    return all((x-a)**2 + (y-c)**2 >= 0.56**2 for a, c in holes)

def hwclass(hw):
    if hw <= 0.075: return 0.075
    return 0.1 if hw <= 0.1 else 0.2

def via_ok(code, hw, ix, iy):
    # exact-geometry feasibility map + drill spacing
    v = viamap[ix * NY + iy]
    if v != 0 and v != code: return False
    return hole_ok(*xy(ix, iy))

# ---------- A* ----------
def astar(code, hw, sources, goals, rip=False, extra=frozenset(),
          forbid=frozenset()):
    """returns (node path, set of ripped net codes) or (None, None).
    extra: cells pre-validated by exact clearance that A* may traverse
    even where the conservative blocking map says no."""
    mF, mB = maps[(0, hw)], maps[(1, hw)]
    hF, hB = hardmaps[(0, hw)], hardmaps[(1, hw)]
    sF, sB = softmaps[(0, hw)], softmaps[(1, hw)]
    def state(lay, ix, iy):
        if (lay, ix, iy) in gset or (lay, ix, iy) in extra: return 0
        k = ix * NY + iy
        v = (mF if lay == 0 else mB)[k]
        if v == 0 or v == code: return 0                     # free
        if not rip: return -1
        hv = (hF if lay == 0 else hB)[k]
        if hv not in (0, code): return -1                    # pad/via: hard
        sv = (sF if lay == 0 else sB)[k]
        if sv in (0, 255, code) or sv in forbid: return -1
        return sv                                            # rippable net
    gset = goals
    gxy = [xy(ix, iy) for (_, ix, iy) in goals]
    def h(lay, ix, iy):
        px, py = xy(ix, iy)
        best = 1e9
        for gx, gy in gxy:
            dxa, dya = abs(px-gx), abs(py-gy)
            v = max(dxa, dya) + 0.415 * min(dxa, dya)
            if v < best: best = v
        return best / STEP
    openq = [(h(*s), 0, s, None) for s in sources if state(*s) == 0]
    heapq.heapify(openq)
    came, gcost = {}, {}
    for _, g, s, _ in openq: gcost[s] = 0
    while openq:
        f, g, node, prev = heapq.heappop(openq)
        if node in came: continue
        came[node] = prev
        if node in gset:
            path = [node]
            while came[path[-1]] is not None: path.append(came[path[-1]])
            path = path[::-1]
            ripped = set()
            if rip:
                for (lay, ix, iy) in path:
                    st = state(lay, ix, iy)
                    if st > 0: ripped.add(st)
            return path, ripped
        lay, ix, iy = node
        pdir = None
        if prev and prev[0] == lay:
            pdir = (ix - prev[1], iy - prev[2])
        for dx, dy, sc in ((1,0,1), (-1,0,1), (0,1,1), (0,-1,1),
                           (1,1,1.42), (1,-1,1.42), (-1,1,1.42), (-1,-1,1.42)):
            jx, jy = ix + dx, iy + dy
            nb = (lay, jx, jy)
            if not (0 <= jx < NX and 0 <= jy < NY) or nb in came: continue
            st = state(lay, jx, jy)
            if st < 0: continue
            if dx and dy:
                if state(lay, jx, iy) < 0 or state(lay, ix, jy) < 0:
                    continue                     # no corner cutting
            ng = g + sc + (TURN_COST if pdir and (dx, dy) != pdir else 0) \
                   + (RIP_COST if st > 0 else 0)
            if ng < gcost.get(nb, 1e9):
                gcost[nb] = ng
                heapq.heappush(openq, (ng + h(lay, jx, jy), ng, nb, node))
        nb = (1 - lay, ix, iy)
        if nb not in came and via_ok(code, hw, ix, iy):
            ng = g + VIA_COST
            if ng < gcost.get(nb, 1e9):
                gcost[nb] = ng
                heapq.heappush(openq, (ng + h(*nb), ng, nb, node))
    return None, None

def emit(path, w, code, book):
    i = 0
    while i < len(path) - 1:
        if path[i][0] != path[i+1][0]:            # via
            add_via(*xy(path[i][1], path[i][2]), code, book=book)
            i += 1; continue
        j = i + 1
        dx, dy = path[j][1] - path[i][1], path[j][2] - path[i][2]
        while (j + 1 < len(path) and path[j+1][0] == path[i][0] and
               (path[j+1][1] - path[j][1], path[j+1][2] - path[j][2]) == (dx, dy)):
            j += 1
        add_track(*xy(path[i][1], path[i][2]), *xy(path[j][1], path[j][2]),
                  w, path[i][0], code, book=book)
        i = j

PADSH = 0.09     # roundrect/oval corner radius: bbox corners have no copper

def pad_goal_cells(px, py, rect, lay, code, hw, tht):
    out = []
    m = maps[(lay, hwclass(hw))]
    sx = min(PADSH, (rect[2] - rect[0]) * 0.25)
    sy = min(PADSH, (rect[3] - rect[1]) * 0.25)
    ax0, ay0 = cell(rect[0], rect[1]); ax1, ay1 = cell(rect[2], rect[3])
    for ix in range(max(ax0, 0), min(ax1, NX - 1) + 1):
        for iy in range(max(ay0, 0), min(ay1, NY - 1) + 1):
            qx, qy = xy(ix, iy)
            # inside the shrunk rect OR on the pad's centre axes: real copper
            on_ax = (abs(qx - px) <= 0.03 or abs(qy - py) <= 0.03)
            if (rect[0] + (0 if on_ax else sx) - 0.01 <= qx <=
                    rect[2] - (0 if on_ax else sx) + 0.01 and
                rect[1] + (0 if on_ax else sy) - 0.01 <= qy <=
                    rect[3] - (0 if on_ax else sy) + 0.01):
                if m[ix * NY + iy] in (0, code):
                    for L in ((0, 1) if tht else (lay,)): out.append((L, ix, iy))
    if not out:
        ix, iy = cell(px, py)
        if 0 <= ix < NX and 0 <= iy < NY and m[ix * NY + iy] in (0, code):
            out.append((lay, ix, iy))
    return out

def net_component(n, code, seed):
    """cells of the copper component geometrically connected to seed pad."""
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
        # sample the non-pad geometry, test against the other
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
        seg = (ga[0], ga[1], ga[2], ga[3], 0)
        return any(_seg_dd(x, y, seg) <= ga[4]+hw+.01 for x, y in pts)
    seedi = ("pad", seed)
    comp = {items.index(seedi)} if seedi in items else set()
    grew = True
    while grew:
        grew = False
        for i, it in enumerate(items):
            if i in comp: continue
            if any(touch(items[j], it) for j in comp):
                comp.add(i); grew = True
    tree, members = set(), []
    for i in comp:
        kind, g = items[i]
        members.append(items[i])
        if kind == "pad":
            tree |= set(pad_goal_cells(g[0], g[1], g[2], g[3], code, 0.1, g[4]))
        elif kind == "via":
            ix, iy = cell(g[0], g[1])
            if 0 <= ix < NX and 0 <= iy < NY:
                tree |= {(0, ix, iy), (1, ix, iy)}
        else:
            k = max(int(math.hypot(g[2]-g[0], g[3]-g[1]) / STEP), 1)
            for i2 in range(k + 1):
                ix, iy = cell(g[0]+(g[2]-g[0])*i2/k, g[1]+(g[3]-g[1])*i2/k)
                if 0 <= ix < NX and 0 <= iy < NY and g[5] != 2:
                    tree.add((g[5], ix, iy))
    pads_in = {g for kk, g in members if kk == "pad"}
    return tree, pads_in

def _seg_dd(px, py, seg):
    x0, y0, x1, y1, _ = seg
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    tt = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px-x0)*dx + (py-y0)*dy) / L2))
    return math.hypot(px - (x0 + tt*dx), py - (y0 + tt*dy))

def net_tree(n, code):
    """cells of every existing copper object of net n (pads+tracks+vias)."""
    tree = set()
    for (px, py, rect, lay, tht) in pads_by_net.get(n, []):
        tree |= set(pad_goal_cells(px, py, rect, lay, code, 0.1, tht))
    for t in b.Tracks():
        if t.GetNetCode() != code: continue
        if isinstance(t, pcbnew.PCB_VIA):
            pos = t.GetPosition()
            ix, iy = cell(ToMM(pos.x), ToMM(pos.y))
            if 0 <= ix < NX and 0 <= iy < NY:
                tree |= {(0, ix, iy), (1, ix, iy)}
        else:
            s, e = t.GetStart(), t.GetEnd()
            lay = 0 if t.GetLayer() == F else (1 if t.GetLayer() == B else None)
            if lay is None: continue
            x0, y0, x1, y1 = ToMM(s.x), ToMM(s.y), ToMM(e.x), ToMM(e.y)
            L = math.hypot(x1 - x0, y1 - y0)
            k = max(int(L / STEP), 1)
            for i in range(k + 1):
                ix, iy = cell(x0 + (x1-x0)*i/k, y0 + (y1-y0)*i/k)
                if 0 <= ix < NX and 0 <= iy < NY: tree.add((lay, ix, iy))
    return tree

build_maps()

# ---------- escape stubs for fine-pitch ICs ----------
def stub_clear0(px, py, vx, vy, code):
    m = maps[(0, 0.1)]
    L = math.hypot(vx - px, vy - py)
    k = max(int(L / 0.1), 1)
    for i in range(k + 1):
        qx, qy = px + (vx - px) * i / k, py + (vy - py) * i / k
        ix, iy = cell(qx, qy)
        if not (2 <= ix < NX - 2 and 2 <= iy < NY - 2): return False
        if m[ix * NY + iy] not in (0, code): return False
    return True

stub_cells = {}
stub_objs = []
STUB_LAST = [None]
FINE = {"U1","U2","U3","U4","U5","U6","U7","U8","U9","U10","U11","U12",
        "U13","U14","U15","U16","J1"}
LONG = {"U1", "U2", "U7", "J1"}          # big-halo ICs: stub through the lane
stubs = 0
for fp in b.GetFootprints():
    if fp.GetReference() not in FINE: continue
    c = fp.GetPosition(); fcx, fcy = ToMM(c.x), ToMM(c.y)
    for p in fp.Pads():
        n = p.GetNetname()
        if (not n or n == "GND" or n.startswith("unconnected")
                or n in ("RF_TX", "ANT_TX", "ANT_RX")):
            continue
        pos = p.GetPosition(); px, py = ToMM(pos.x), ToMM(pos.y)
        r_ = pad_rect(p)
        sx, sy = r_[2] - r_[0], r_[3] - r_[1]
        if abs(sx - sy) > 0.1:
            dx, dy = ((1.0, 0.0) if sx > sy else (0.0, 1.0))
        else:
            rx, ry = px - fcx, py - fcy
            dx, dy = ((1.0, 0.0) if abs(rx) > abs(ry) else (0.0, 1.0))
        if dx * (px - fcx) + dy * (py - fcy) < 0:
            dx, dy = -dx, -dy
        half = (sx if dx else sy) / 2
        L = 1.35 if fp.GetReference() in LONG else 0.85
        ex, ey = px + dx * (half + L), py + dy * (half + L)
        if stub_clear0(px, py, ex, ey, p.GetNetCode()):
            add_track(px, py, ex, ey, 0.2, 0, p.GetNetCode(), stub=True)
            cells = set()
            k = max(int(math.hypot(ex - px, ey - py) / 0.1), 1)
            for i in range(k + 1):
                ix, iy = cell(px + (ex-px)*i/k, py + (ey-py)*i/k)
                cells.add((0, ix, iy))
            stub_cells[(round(px, 2), round(py, 2))] = cells
            stub_objs.append((STUB_LAST[0], px, py, ex, ey, p.GetNetCode()))
            stubs += 1
print(f"escape stubs: {stubs}")

# ---------- route nets ----------
fails = []
trees = {}
def try_leg(n, code, w, tree, p, rip=False):
    for ww in ((w,) if w == 0.2 else (w, 0.2)):
        hw = ww / 2
        goals = set(pad_goal_cells(p[0], p[1], p[2], p[3], code, hw, p[4]))
        srcs = tree - goals           # a leg must attach to OTHER copper
        if not goals or not srcs: continue
        path, ripped = astar(code, hw, srcs, goals, rip=rip)
        if path and not ripped:
            emit(path, ww, code, n)
            tree |= set(path) | goals
            tree |= stub_cells.get((round(p[0], 2), round(p[1], 2)), set())
            return True, set()
        if path and rip:
            return False, ripped
    return False, set()

def route_net(n):
    pl = pads_by_net[n]
    code = b.FindNet(n).GetNetCode()
    w = 0.4 if n in RAILS else 0.2
    cx = sum(q[0] for q in pl) / len(pl); cy = sum(q[1] for q in pl) / len(pl)
    rest = sorted(pl, key=lambda q: (q[0]-cx)**2 + (q[1]-cy)**2)
    first, rest = rest[0], rest[1:]
    if emitted.get(n):
        # rerouting after rip: only copper actually connected to the seed pad
        tree, pads_in = net_component(n, code, first)
        rest = [q for q in rest if q not in pads_in]
        if not tree:
            tree = set(pad_goal_cells(first[0], first[1], first[2], first[3],
                                      code, 0.1 if w == 0.2 else 0.2, first[4]))
    else:
        tree = set(pad_goal_cells(first[0], first[1], first[2], first[3],
                                  code, 0.1 if w == 0.2 else 0.2, first[4]))
        tree |= stub_cells.get((round(first[0], 2), round(first[1], 2)), set())
    trees[n] = tree
    out = []
    while rest:
        rest.sort(key=lambda q: min(abs(q[0]-xy(ix,iy)[0]) + abs(q[1]-xy(ix,iy)[1])
                                    for (_, ix, iy) in tree) if tree else 0)
        p = rest.pop(0)
        ok, _ = try_leg(n, code, w, tree, p)
        if not ok:
            out.append((n, p))
    return out

for _, n in net_order:
    fails += route_net(n)

# ---------- plane-rail stitching to In2 regions ----------
gcode = b.FindNet("GND").GetNetCode()

def plane_ok(rail, x, y):
    if rail == "GND": return True
    r = PLANES[rail]
    if r: return r[0]+0.8 < x < r[2]-0.8 and r[1]+0.8 < y < r[3]-0.8
    return not any(q[0]-0.8 < x < q[2]+0.8 and q[1]-0.8 < y < q[3]+0.8
                   for q in PLANES.values() if q)

def stub_clear(px, py, vx, vy, code, hw):
    m = maps[(0, hwclass(hw))]
    L = math.hypot(vx - px, vy - py)
    k = max(int(L / 0.1), 1)
    for i in range(k + 1):
        qx, qy = px + (vx - px) * i / k, py + (vy - py) * i / k
        ix, iy = cell(qx, qy)
        if not (0 <= ix < NX and 0 <= iy < NY): return False
        if m[ix * NY + iy] not in (0, code): return False
    return True

ALL_PAD_RECTS = []
for fp in b.GetFootprints():
    for p_ in fp.Pads():
        ALL_PAD_RECTS.append((pad_rect(p_), p_.GetNetCode(),
                              p_.GetNetname() or "?"))

def stub_pad_clear(px, py, vx, vy, code, w):
    lim = 0.15 + w / 2 + 0.02
    for (r, c, _) in ALL_PAD_RECTS:
        if c == code: continue
        if min(px, vx) - lim > r[2] or max(px, vx) + lim < r[0]: continue
        if min(py, vy) - lim > r[3] or max(py, vy) + lim < r[1]: continue
        k = max(int(math.hypot(vx - px, vy - py) / 0.05), 1)
        for i in range(k + 1):
            qx, qy = px + (vx-px)*i/k, py + (vy-py)*i/k
            ddx = max(r[0] - qx, 0, qx - r[2])
            ddy = max(r[1] - qy, 0, qy - r[3])
            if math.hypot(ddx, ddy) < lim: return False
    return True

def stitch(px, py, rect, code, w, rail):
    rx = max(rect[2] - rect[0], rect[3] - rect[1]) / 2
    for rad in (rx + 0.55, rx + 0.75, rx + 1.0, rx + 1.4, rx + 1.9, rx + 2.5):
        for a in range(16):
            ang = a * math.pi / 8
            vx, vy = px + rad * math.cos(ang), py + rad * math.sin(ang)
            ix, iy = cell(vx, vy)
            if not (1 <= ix < NX - 1 and 1 <= iy < NY - 1): continue
            if not via_ok(code, w / 2, ix, iy): continue
            vx, vy = xy(ix, iy)
            if not plane_ok(rail, vx, vy): continue
            if not stub_clear(px, py, vx, vy, code, w / 2): continue
            if not stub_pad_clear(px, py, vx, vy, code, w): continue
            add_track(px, py, vx, vy, w, 0, code)
            add_via(vx, vy, code)
            return True
    return False

for rail in PLANES:
    code_r = b.FindNet(rail).GetNetCode()
    trees[rail] = set()
    for p in pads_by_net.get(rail, []):
        (px, py, rect, lay, tht) = p
        if tht: continue
        if stitch(px, py, rect, code_r, 0.4, rail):
            trees[rail] |= set(pad_goal_cells(px, py, rect, lay, code_r, 0.2, tht))
        else:
            fails.append((rail, p))

# ---------- GND stitching (In1 access) ----------
gnd_done = 0
for (px, py, rect, lay, tht) in pads_by_net.get("GND", []):
    if tht or lay != 0: continue
    if any((px-a)**2 + (py-c)**2 < 1.2**2 for a, c in holes):
        continue                    # a ground-ish hole is already close by
    if stitch(px, py, rect, gcode, 0.3, "GND"):
        gnd_done += 1

# ---------- power drop vias ----------
# rail pads that couldn't stitch get a free-standing via in their plane
# region within reach; A* then routes the pad to it
for n, p in fails:
    if n not in PLANES: continue
    code_r = b.FindNet(n).GetNetCode()
    px, py = p[0], p[1]
    best = None
    ix0, iy0 = cell(px, py)
    R = 30
    for dix in range(-R, R + 1):
        for diy in range(-R, R + 1):
            ix, iy = ix0 + dix, iy0 + diy
            if not (1 <= ix < NX - 1 and 1 <= iy < NY - 1): continue
            d2 = dix * dix + diy * diy
            if best and d2 >= best[0]: continue
            vx, vy = xy(ix, iy)
            if not plane_ok(n, vx, vy): continue
            if not via_ok(code_r, 0.1, ix, iy): continue
            best = (d2, ix, iy)
    if best:
        _, ix, iy = best
        vx, vy = xy(ix, iy)
        add_via(vx, vy, code_r)
        trees[n] |= {(0, ix, iy), (1, ix, iy)}

# ---------- retry rounds ----------
still = fails
for _round in (2, 3, 4):
    fails2 = []
    for n, p in still:
        code = b.FindNet(n).GetNetCode()
        w = 0.4 if n in RAILS else 0.2
        ok, _ = try_leg(n, code, w, trees.setdefault(n, net_tree(n, code)), p)
        if not ok:
            fails2.append((n, p))
    still = fails2

# ---------- rip-up & reroute ----------
rip_budget = 25
rip_tries = {}
queue = list(still)
still = []
code2name = {b.FindNet(n).GetNetCode(): n for n in pads_by_net if b.FindNet(n)}
while queue:
    n, p = queue.pop(0)
    code = b.FindNet(n).GetNetCode()
    w = 0.4 if n in RAILS else 0.2
    tree = trees.setdefault(n, net_tree(n, code))
    ok, _ = try_leg(n, code, w, tree, p)
    if ok: continue
    if rip_budget <= 0:
        still.append((n, p)); continue
    key = (n, round(p[0], 2), round(p[1], 2))
    if rip_tries.get(key, 0) >= 2:
        still.append((n, p)); continue
    rip_tries[key] = rip_tries.get(key, 0) + 1
    ok, ripped = try_leg(n, code, w, tree, p, rip=True)
    victims = {code2name[c] for c in ripped if c in code2name} - {n}
    if not victims:
        still.append((n, p)); continue
    rip_budget -= 1
    for vn in victims:
        for obj in emitted.get(vn, []):
            b.Remove(obj)
        emitted[vn] = []
    build_maps()
    trees.clear()
    tree = trees.setdefault(n, net_tree(n, code))
    ok, _ = try_leg(n, code, w, tree, p)   # rescued leg first
    if not ok:
        queue.append((n, p))
    for vn in victims:                     # then reroute the victims
        queue += route_net(vn)
    print(f"rip: {n} freed by ripping {sorted(victims)} "
          f"(budget {rip_budget}, rescued={ok})")

# ---------- trim unused escape stubs / unused drop vias ----------
def _net_copper(code, exclude):
    exid = exclude.m_Uuid.AsString() if exclude else None
    out = []
    for tt in b.Tracks():
        if tt.GetNetCode() != code or tt.m_Uuid.AsString() == exid: continue
        if isinstance(tt, pcbnew.PCB_VIA):
            pos = tt.GetPosition()
            out.append((ToMM(pos.x), ToMM(pos.y), ToMM(pos.x), ToMM(pos.y),
                        ToMM(tt.GetWidth()) / 2))
        else:
            s, e = tt.GetStart(), tt.GetEnd()
            out.append((ToMM(s.x), ToMM(s.y), ToMM(e.x), ToMM(e.y),
                        ToMM(tt.GetWidth()) / 2))
    return out

def _seg_d(px, py, seg):
    x0, y0, x1, y1, _ = seg
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    tt = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px-x0)*dx + (py-y0)*dy) / L2))
    return math.hypot(px - (x0 + tt*dx), py - (y0 + tt*dy))

trimmed = removed = 0
for (obj, px, py, ex, ey, code) in stub_objs:
    others = _net_copper(code, obj)
    keep_t = None
    for i in range(20, 0, -1):
        tt = i / 20.0
        qx, qy = px + (ex-px)*tt, py + (ey-py)*tt
        if any(_seg_d(qx, qy, s) <= 0.1 + s[4] + 0.02 for s in others):
            keep_t = tt; break
    if keep_t is None:
        b.Remove(obj); removed += 1
    elif keep_t < 0.99:
        obj.SetEnd(mm(px + (ex-px)*keep_t, py + (ey-py)*keep_t))
        trimmed += 1
print(f"stubs: removed {removed}, trimmed {trimmed}")

vias_rm = 0
for tt in list(b.Tracks()):
    if not isinstance(tt, pcbnew.PCB_VIA): continue
    if tt.GetNetname() not in PLANES and tt.GetNetname() != "GND": continue
    pos = tt.GetPosition()
    px, py = ToMM(pos.x), ToMM(pos.y)
    touch = any(_seg_d(px, py, s) <= 0.3 + s[4] + 0.02
                for s in _net_copper(tt.GetNetCode(), tt))
    near_pad = any(abs(px-q[0]) < 0.9 and abs(py-q[1]) < 0.9
                   for q in pads_by_net.get(tt.GetNetname(), []))
    if not touch and not near_pad and tt.GetNetname() != "GND":
        b.Remove(tt); vias_rm += 1
print(f"unused rail vias removed: {vias_rm}")

# ---------- heal pass: reconnect clusters from exact copper geometry ----------
def clusters_of(n, code):
    """geometric connected clusters of net n; In2 plane = virtual node -1."""
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
        seg = (ga[0], ga[1], ga[2], ga[3], 0)
        return any(_seg_dd(x, y, seg) <= ga[4]+hw+.01 for x, y in pts)
    parent = list(range(len(items) + 1))     # last index = plane node
    def find(i):
        while parent[i] != i: parent[i] = parent[parent[i]]; i = parent[i]
        return i
    def union(i, j): parent[find(i)] = find(j)
    for i in range(len(items)):
        for j in range(i + 1, len(items)):
            if find(i) != find(j) and touch(items[i], items[j]):
                union(i, j)
    if n in PLANES or n == "GND":
        for i, (k, g) in enumerate(items):
            if k == "via" and plane_ok(n, g[0], g[1]):
                union(i, len(items))
    groups = {}
    for i, it in enumerate(items):
        groups.setdefault(find(i), []).append(it)
    planeroot = find(len(items))
    return groups, planeroot

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

heal_fail = []
for _hround in (1, 2):
    heal_fail = []
    for n in sorted(set(list(RIPPABLE) + [r for r in PLANES])):
        code = b.FindNet(n).GetNetCode()
        groups, planeroot = clusters_of(n, code)
        padded = [(root, m) for root, m in groups.items()
                  if any(k == "pad" for k, _ in m)]
        if len(padded) <= 1: continue
        padded.sort(key=lambda t: (t[0] != planeroot,
                                   -sum(1 for k, _ in t[1] if k == "pad")))
        main_cells = cluster_cells(padded[0][1], code)
        w = 0.4 if n in RAILS else 0.2
        for root, members in padded[1:]:
            goals = cluster_cells(members, code)
            done = False
            for ww in ((w, 0.2, 0.15) if w == 0.4 else (0.2, 0.15)):
                srcs = main_cells - goals
                if not srcs or not goals: continue
                path, ripped = astar(code, ww / 2, srcs, goals)
                if path and not ripped:
                    emit(path, ww, code, n)
                    main_cells |= set(path) | goals
                    done = True
                    break
            if not done:
                main_cells |= goals      # try remaining clusters anyway
                heal_fail.append((n, members[0]))
    if not heal_fail: break
print(f"heal: {len(heal_fail)} clusters still split")
for n, m in heal_fail: print(f"  SPLIT {n} near {m[1][:2]}")

pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(BOARD, b)
print(f"routed {len(net_order)} nets; unresolved legs: {len(still)}")
for n, p in still: print(f"  FAIL {n} @({p[0]:.2f},{p[1]:.2f})")
print(f"GND stitch vias added: {gnd_done}")
