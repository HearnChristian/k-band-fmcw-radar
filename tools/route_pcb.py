"""Copper for Radar1.kicad_pcb via pcbnew API (run inside KiCad snap shell).

Phase A: zones (F.Cu GND / In1 GND / In2 +5V / B.Cu GND, solid connect),
         EP thermal via arrays.
Phase B: hand-crafted 24 GHz GCPW (0.5 mm, 0.25 gap) with GND via fences.
Run AFTER tools/board_gen.py regeneration. Fills zones and saves.
"""
import math
import pcbnew
from pcbnew import VECTOR2I, FromMM, ToMM

BOARD = "/home/christian-thomas-hearn/Desktop/X-BAND FMCW RADAR/Radar1/Radar1.kicad_pcb"
OX, OY, BW, BH = 60.0, 40.0, 66.0, 46.0

b = b_board = pcbnew.LoadBoard(BOARD)

def mm(x, y): return VECTOR2I(FromMM(x), FromMM(y))
def net(name):
    n = b.FindNet(name); assert n, "no net " + name
    return n
def pad_xy(ref, padnum):
    fp = b.FindFootprintByReference(ref); assert fp, ref
    for p in fp.Pads():
        if p.GetNumber() == padnum:
            pos = p.GetPosition(); return (ToMM(pos.x), ToMM(pos.y))
    raise KeyError(f"{ref}.{padnum}")

def add_zone(layer, netname, pts, prio, clearance):
    z = pcbnew.ZONE(b)
    z.SetLayer(layer); z.SetNetCode(net(netname).GetNetCode())
    z.SetAssignedPriority(prio)
    z.SetLocalClearance(FromMM(clearance)); z.SetMinThickness(FromMM(0.15))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    o = z.Outline(); o.NewOutline()
    for x, y in pts: o.Append(FromMM(x), FromMM(y))
    b.Add(z)

def add_via(x, y, netname="GND", drill=0.3, dia=0.6):
    v = pcbnew.PCB_VIA(b)
    v.SetPosition(mm(x, y)); v.SetDrill(FromMM(drill)); v.SetWidth(FromMM(dia))
    v.SetViaType(pcbnew.VIATYPE_THROUGH); v.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
    v.SetNetCode(net(netname).GetNetCode()); b.Add(v)

def add_path(pts, width, layer, netname):
    nc = net(netname).GetNetCode()
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(mm(x0, y0)); t.SetEnd(mm(x1, y1))
        t.SetWidth(FromMM(width)); t.SetLayer(layer); t.SetNetCode(nc)
        b.Add(t)

_via_pts = []
_obstacles = None
_rf_segs = []      # (x0,y0,x1,y1,halfwidth) of every RF segment — fence must clear these
def _build_obstacles():
    global _obstacles
    _obstacles = []
    for fp in b_board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() != "GND":
                s = p.GetSize()
                r = max(ToMM(s.x), ToMM(s.y)) / 2
                pos = p.GetPosition()
                _obstacles.append((ToMM(pos.x), ToMM(pos.y), r + 0.5))

def _seg_dist(px, py, x0, y0, x1, y1):
    dx, dy = x1 - x0, y1 - y0
    L2 = dx * dx + dy * dy
    t = 0.0 if L2 == 0 else max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / L2))
    return math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))

def guarded_via(x, y):
    if x < OX + 0.45 or x > OX + BW - 0.45 or y < OY + 0.45 or y > OY + BH - 0.45:
        return
    for (a, c, r) in _obstacles:
        if (x - a) ** 2 + (y - c) ** 2 < r * r: return
    for (a, c) in _via_pts:
        if (x - a) ** 2 + (y - c) ** 2 < 0.7 ** 2: return
    # via (r 0.3) vs RF copper: halfwidth + 0.3 + 0.15 clearance + margin
    for (x0, y0, x1, y1, hw) in _rf_segs:
        if _seg_dist(x, y, x0, y0, x1, y1) < hw + 0.47: return
    _via_pts.append((x, y))
    add_via(x, y)

def fence(pts, offset=0.85, pitch=0.8):
    if _obstacles is None: _build_obstacles()
    n = len(pts) - 1
    for k, ((x0, y0), (x1, y1)) in enumerate(zip(pts, pts[1:])):
        dx, dy = x1 - x0, y1 - y0
        L = math.hypot(dx, dy)
        m0 = 1.4 if k == 0 else 0.2
        m1 = 1.4 if k == n - 1 else 0.2
        if L < m0 + m1: continue
        ux, uy = dx / L, dy / L
        px, py = -uy, ux
        d = m0
        while d <= L - m1:
            cx, cy = x0 + ux * d, y0 + uy * d
            for s in (+1, -1):
                guarded_via(cx + s * px * offset, cy + s * py * offset)
            d += pitch

# ================= Phase A =================
rect = [(OX, OY), (OX + BW, OY), (OX + BW, OY + BH), (OX, OY + BH)]
add_zone(pcbnew.F_Cu,   "GND", rect, 0, 0.25)   # GCPW top ground, gap 0.25
add_zone(pcbnew.In1_Cu, "GND", rect, 0, 0.20)
add_zone(pcbnew.In2_Cu, "+5V", rect, 0, 0.30)
add_zone(pcbnew.B_Cu,   "GND", rect, 0, 0.20)

# In2 split plane: rail sub-zones (priority 1) over the +5V base fill.
# Keep in sync with PLANES in tools/route_signals.py.
def zrect(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
add_zone(pcbnew.In2_Cu, "+3V3_RF",  zrect(59.0, 41.0,  88.0, 78.0), 1, 0.30)
add_zone(pcbnew.In2_Cu, "+1V8",     zrect(88.5, 41.0, 103.0, 78.0), 1, 0.30)
add_zone(pcbnew.In2_Cu, "+3V3_DIG", zrect(103.5, 41.0, 127.0, 78.0), 1, 0.30)

# U12 excluded: Marki footprint already embeds 9 PTH thermal vias in the EP
EPGRID = {"U1": (2, 2, 0.9), "U2": (3, 3, 0.9),
          "U9": (2, 2, 0.9), "U6": (3, 3, 1.0), "U13": (3, 3, 1.0)}
for ref, (nx, ny, p) in EPGRID.items():
    fp = b.FindFootprintByReference(ref)
    c = fp.GetPosition(); cx, cy = ToMM(c.x), ToMM(c.y)
    for i in range(nx):
        for j in range(ny):
            vx, vy = cx + (i - (nx - 1) / 2) * p, cy + (j - (ny - 1) / 2) * p
            _via_pts.append((vx, vy)); add_via(vx, vy)

# ================= Phase B: 24 GHz GCPW =================
# 0.5 mm line is wider than the 0.5 mm-pitch QFN pads (U1 0.25 sq, U12
# 0.60x0.30) -> neck to pad width for the first NL mm so adjacent GND
# pads keep the 0.15 netclass clearance.
RFW = 0.5
NL = 0.6
tx   = pad_xy("U1", "11")    # BGT24 TX out       (net RF_TX)
rxin = pad_xy("U1", "3")     # BGT24 RX in        (net ANT_RX)
pain = pad_xy("U12", "2")    # AMM-8211 RF_IN     (net RF_TX)
paout= pad_xy("U12", "10")   # AMM-8211 RF_OUT    (net ANT_TX)
j3   = pad_xy("J3", "1")     # TX connector       (net ANT_TX)
j4   = pad_xy("J4", "1")     # RX connector       (net ANT_RX)
print("pads:", dict(tx=tx, rxin=rxin, pain=pain, paout=paout, j3=j3, j4=j4))

def rf_path(pts, netname, neck_w_start=None, neck_w_end=None):
    """add_path with pad-exit necks; registers all segments for the fence."""
    pts = list(pts)
    def split(p0, p1, L):          # point L mm from p0 toward p1
        dx, dy = p1[0] - p0[0], p1[1] - p0[1]
        d = math.hypot(dx, dy)
        return (p0[0] + dx / d * L, p0[1] + dy / d * L)
    runs = []                      # (ptslist, width)
    if neck_w_start:
        m = split(pts[0], pts[1], NL)
        runs.append(([pts[0], m], neck_w_start))
        pts = [m] + pts[1:]
    tail = None
    if neck_w_end:
        m = split(pts[-1], pts[-2], NL)
        tail = ([m, pts[-1]], neck_w_end)
        pts = pts[:-1] + [m]
    runs.append((pts, RFW))
    if tail: runs.append(tail)
    for seg_pts, w in runs:
        add_path(seg_pts, w, pcbnew.F_Cu, netname)
        for (x0, y0), (x1, y1) in zip(seg_pts, seg_pts[1:]):
            _rf_segs.append((x0, y0, x1, y1, w / 2))

# RF_TX: U1 TX (east col, x=79.05) -> corridor x=80.6 -> U12 RF_IN (east col)
XC = 80.6
p_rftx = [tx, (XC, tx[1]), (XC, pain[1]), pain]
rf_path(p_rftx, "RF_TX", neck_w_start=0.25, neck_w_end=0.30)

# ANT_TX: U12 RF_OUT (west col) -> straight west clear of the pad column
# (GND pads 9/11 sit 0.5 above/below pin 10) -> 45deg down -> J3 pin
p_anttx = [paout, (paout[0] - 1.0, paout[1]), (paout[0] - 1.5, j3[1]),
           (j3[0] + 0.3, j3[1]), j3]
rf_path(p_anttx, "ANT_TX", neck_w_start=0.30)

# ANT_RX: J4 -> east, 45 up, into U1 RX (west col, x=76.95)
mid_x = rxin[0] - 3.0
p_antrx = [j4, (mid_x - abs(rxin[1] - j4[1]), j4[1]), (mid_x, rxin[1]), rxin]
rf_path(p_antrx, "ANT_RX", neck_w_end=0.25)

fence(p_rftx)
fence(p_anttx)
fence(p_antrx)

# ---- fill & save ----
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(BOARD, b)
print("phase A+B done: zones filled, RF routed, saved")
