"""Generate Radar1.kicad_pcb from Radar1.net + floorplan (docs/PCB-PLAN.md).

Text-built board: 4-layer hybrid RO4350B/FR4 stackup, all components embedded
with pad nets bound from the netlist, ICs at floorplan coordinates, passives
auto-clustered around their owner ICs. No routing — placement starting point.
"""
import re, os, math, uuid, subprocess, sys

D   = "/home/christian-thomas-hearn/Desktop/X-BAND FMCW RADAR/Radar1"
STD = "/snap/kicad/current/usr/share/kicad/footprints"
NET = D + "/Radar1.net"
OUT = D + "/Radar1.kicad_pcb"

OX, OY = 60.0, 40.0          # page origin of board's top-left corner
BW, BH = 60.0, 42.0          # board size, mm

def nuid(): return str(uuid.uuid4())

# ---------- parse netlist ----------
ntxt = open(NET).read()
comps = {}
for ref, body in re.findall(r'\(comp \(ref "([^"]+)"\)(.*?)(?=\(comp \(ref|\(libparts)', ntxt, re.S):
    fp  = re.search(r'\(footprint "([^"]+)"\)', body)
    val = re.search(r'\(value "([^"]*)"\)', body)
    ts  = re.search(r'\(tstamps? "([^"]+)"\)', body)
    comps[ref] = {"fp": fp.group(1), "value": val.group(1) if val else "",
                  "ts": (ts.group(1).split()[0] if ts else nuid())}

netcode = {}          # name -> code
padnet  = {}          # (ref,pin) -> (code,name)
names   = []
for name, body in re.findall(r'\(net \(code "\d+"\) \(name "([^"]+)"\)(.*?)(?=\(net \(code|\Z)', ntxt, re.S):
    if name not in netcode:
        netcode[name] = len(netcode) + 1; names.append(name)
    for r, p in re.findall(r'\(node \(ref "([^"]+)"\) \(pin "([^"]+)"\)', body):
        padnet[(r, p)] = (netcode[name], name)

# ---------- fixed placement (board-local mm, from docs/PCB-PLAN.md) ----------
FIXED = {
 "J3": (0.8,  6.0, 0),   "J4": (0.8, 22.5, 0),
 "U12": (18.0, 6.0, 0),  "U1": (18.0, 17.5, 0),
 "U2": (24.0, 21.0, 0),  "Y1": (28.0, 27.5, 0),
 "U3": (26.0, 6.5, 0),   "U8": (35.0, 6.0, 0),
 "U16": (34.0, 12.0, 0), "U7": (45.0, 16.0, 0),
 "U11": (41.5, 27.0, 0), "U10": (48.3, 26.3, 0),
 "U15": (54.0, 22.5, 0), "J1": (57.5, 15.5, 90),
 "U4": (5.0, 37.0, 0),   "U13": (12.0, 37.0, 0),
 "U9": (19.5, 37.0, 0),  "U5": (26.5, 37.0, 0),
 "U14": (33.0, 37.0, 0), "U6": (38.5, 37.0, 0),
 "D1": (37.0, 31.0, 0),  "J2": (56.0, 34.0, 0),
 "C55": (42.5, 24.2, 0), "R22": (33.0, 22.0, 0),
}
# ring start radius by owner (courtyard half-extent + margin)
RSTART = {"U7": 8.5, "J1": 6.5, "U6": 5.0, "U13": 5.0, "U2": 4.0, "U9": 4.0,
          "U1": 3.5, "U12": 3.5, "J2": 8.0}
POWER_HINT = {  # rail name -> LDO to cluster rail-only passives near
 "+5V": "U6", "+3V3_RF": "U4", "+3V3_DIG": "U14", "+1V8": "U5",
 "+3V0_PA": "U13", "-VGG": "U9", "VGG_CP": "U9", "+VIN": "J2", "VIN_RAW": "J2",
 "VBUS": "J1", "VCORE": "U7", "VREF": "U16",
}

# ---------- owner assignment for passives ----------
net_members = {}
for (r, p), (c, n) in padnet.items():
    net_members.setdefault(n, []).append((r, p))
ICS = set(FIXED)
def owner(ref):
    score = {}
    rails = []
    for (r, p), (c, n) in padnet.items():
        if r != ref: continue
        mem = net_members[n]
        if n == "GND" or n.startswith("unconnected"): continue
        if len(mem) > 6:
            rails.append(n); continue
        for r2, _ in mem:
            if r2 != ref and r2 in ICS:
                score[r2] = score.get(r2, 0) + 1
    if score:
        return max(score, key=lambda k: (score[k], -int(re.sub(r'\D','',k) or 0)))
    for n in rails:
        if n in POWER_HINT: return POWER_HINT[n]
    return "U6"

# ---------- size-aware ring slot allocator ----------
def _fp_bbox(path):
    t = open(path).read()
    xs, ys = [], []
    for m in re.finditer(r'\(pad\s+"[^"]*"[^(]*\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+[-\d.]+)?\)\s*\(size\s+([-\d.]+)\s+([-\d.]+)\)', t):
        x, y, sx, sy = map(float, m.groups())
        xs += [x-sx/2, x+sx/2]; ys += [y-sy/2, y+sy/2]
    for m in re.finditer(r'\((?:start|end|xy)\s+([-\d.]+)\s+([-\d.]+)\)', t):
        xs.append(float(m.group(1))); ys.append(float(m.group(2)))
    if not xs: return (-1, -1, 1, 1)
    return (min(xs), min(ys), max(xs), max(ys))

_bbcache = {}
def bbox_of(ref):
    fpid = comps[ref]["fp"]
    if fpid not in _bbcache:
        _bbcache[fpid] = _fp_bbox(fp_path(fpid))
    return _bbcache[fpid]

def radius_of(ref):
    x0, y0, x1, y1 = bbox_of(ref)
    return max(math.hypot(a, b) for a in (x0, x1) for b in (y0, y1)) + 0.25

RECTS = []      # bounding rects of everything placed (fixed + passives)

def _rot_bbox(bb, rot):
    x0, y0, x1, y1 = bb
    if rot % 360 == 90:  return (y0, -x1, y1, -x0)
    if rot % 360 == 180: return (-x1, -y1, -x0, -y0)
    if rot % 360 == 270: return (-y1, x0, -y0, x1)
    return bb

def free_rect(x, y, bb, gap=0.3):
    x0, y0, x1, y1 = x+bb[0]-gap, y+bb[1]-gap, x+bb[2]+gap, y+bb[3]+gap
    if x0 < OX+0.8 or y0 < OY+0.8 or x1 > OX+BW-0.8 or y1 > OY+BH-2.6:
        return False
    for a0, b0, a1, b1 in RECTS:
        if x0 < a1 and x1 > a0 and y0 < b1 and y1 > b0:
            return False
    return True

def slot_near(oref, ref):
    bb = bbox_of(ref)
    bx, by, _ = FIXED[oref]
    cx, cy = OX+bx, OY+by
    cands = sorted((( (i*0.55)**2 + (j*0.55)**2, cx+i*0.55, cy+j*0.55)
                    for i in range(-60, 61) for j in range(-60, 61)))
    for _, x, y in cands:
        if free_rect(x, y, bb):
            RECTS.append((x+bb[0], y+bb[1], x+bb[2], y+bb[3]))
            return x, y
    raise RuntimeError("no slot near " + oref)

# ---------- footprint embedding ----------
def fp_path(fpid):
    lib, name = fpid.split(":", 1)
    for base in (D, STD):
        p = f"{base}/{lib}.pretty/{name}.kicad_mod"
        if os.path.exists(p): return p
    raise FileNotFoundError(fpid)

def spans(txt, opener):
    out, i = [], 0
    while True:
        i = txt.find(opener, i)
        if i < 0: break
        d, j = 0, i
        while True:
            if txt[j] == '(': d += 1
            elif txt[j] == ')':
                d -= 1
                if d == 0: break
            j += 1
        out.append((i, j+1)); i = j
    return out

def embed(ref):
    c = comps[ref]
    fpid = c["fp"]; lib, name = fpid.split(":", 1)
    t = open(fp_path(fpid)).read()
    if ref in FIXED:
        bx, by, rot = FIXED[ref]; x, y = OX+bx, OY+by
    else:
        x, y = slot_near(owner(ref), ref); rot = 0
    t = t.replace(f'(footprint "{name}"', f'(footprint "{fpid}"', 1)
    t = re.sub(r'\t\(version \d+\)\n|\t\(generator "[^"]*"\)\n|\t\(generator_version "[^"]*"\)\n', '', t)
    t = t.replace('(layer "F.Cu")',
                  f'(layer "F.Cu")\n\t(uuid "{nuid()}")\n\t(at {x:.3f} {y:.3f} {rot})\n\t(path "/{c["ts"]}")', 1)
    # rotation: child coords stay local; angle fields become absolute
    if rot:
        def bump(m):
            return f'(at {m.group(1)} {m.group(2)} {float(m.group(3) or 0)+rot:g})' if m.group(3) is not None \
                   else f'(at {m.group(1)} {m.group(2)} {rot})'
        body_start = t.find('(path')
        head, body = t[:body_start], t[body_start:]
        body = re.sub(r'\(at\s+([-\d.]+)\s+([-\d.]+)(?:\s+([-\d.]+))?\)', bump, body)
        t = head + body
    # reference / value texts
    t = re.sub(r'\(property "Reference" "[^"]*"', f'(property "Reference" "{ref}"', t, count=1)
    t = re.sub(r'\(property "Value" "[^"]*"', f'(property "Value" "{c["value"]}"', t, count=1)
    t = t.replace('"REF**"', f'"{ref}"')
    # pad nets
    out, last = [], 0
    for s, e in spans(t, '(pad'):
        blk = t[s:e]
        m = re.match(r'\(pad\s+"([^"]*)"', blk)
        if m and (ref, m.group(1)) in padnet:
            code, nname = padnet[(ref, m.group(1))]
            nname = nname.replace('"', '\\"')
            blk = blk[:-1].rstrip() + f'\n\t\t(net {code} "{nname}")\n\t)'
        out.append(t[last:s]); out.append(blk); last = e
    out.append(t[last:])
    t = "".join(out)
    t = re.sub(r'\(uuid "[0-9a-f-]+"\)', lambda m: f'(uuid "{nuid()}")', t)
    return "\t" + t.replace("\n", "\n\t").rstrip("\t")

# ---------- board skeleton ----------
def header():
    nets = '\n'.join(f'\t(net {i+1} "{n}")' for i, n in enumerate(names))
    return f'''(kicad_pcb
\t(version 20241229)
\t(generator "pcbnew")
\t(generator_version "9.0")
\t(general
\t\t(thickness 1.545)
\t\t(legacy_teardrops no)
\t)
\t(paper "A4")
\t(title_block
\t\t(title "24 GHz FMCW Radar")
\t\t(date "2026-07-09")
\t\t(rev "D")
\t\t(comment 1 "Hybrid stackup: RO4350B 0.254mm over FR4 core")
\t)
\t(layers
\t\t(0 "F.Cu" signal)
\t\t(1 "In1.Cu" signal "GND")
\t\t(2 "In2.Cu" signal "PWR")
\t\t(3 "B.Cu" signal)
\t\t(11 "F.Paste" user)
\t\t(13 "F.SilkS" user "F.Silkscreen")
\t\t(15 "F.Mask" user)
\t\t(10 "B.Paste" user)
\t\t(12 "B.SilkS" user "B.Silkscreen")
\t\t(14 "B.Mask" user)
\t\t(5 "Cmts.User" user "User.Comments")
\t\t(17 "Dwgs.User" user "User.Drawings")
\t\t(25 "Edge.Cuts" user)
\t\t(27 "Margin" user)
\t\t(29 "F.CrtYd" user "F.Courtyard")
\t\t(28 "B.CrtYd" user "B.Courtyard")
\t\t(31 "F.Fab" user)
\t\t(30 "B.Fab" user)
\t)
\t(setup
\t\t(stackup
\t\t\t(layer "F.SilkS" (type "Top Silk Screen"))
\t\t\t(layer "F.Paste" (type "Top Solder Paste"))
\t\t\t(layer "F.Mask" (type "Top Solder Mask") (thickness 0.01))
\t\t\t(layer "F.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "dielectric 1" (type "core") (thickness 0.254) (material "RO4350B") (epsilon_r 3.48) (loss_tangent 0.0037))
\t\t\t(layer "In1.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "dielectric 2" (type "core") (thickness 0.9) (material "FR4") (epsilon_r 4.5) (loss_tangent 0.02))
\t\t\t(layer "In2.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "dielectric 3" (type "prepreg") (thickness 0.2) (material "FR4") (epsilon_r 4.4) (loss_tangent 0.02))
\t\t\t(layer "B.Cu" (type "copper") (thickness 0.035))
\t\t\t(layer "B.Mask" (type "Bottom Solder Mask") (thickness 0.01))
\t\t\t(layer "B.Paste" (type "Bottom Solder Paste"))
\t\t\t(layer "B.SilkS" (type "Bottom Silk Screen"))
\t\t\t(copper_finish "ENIG")
\t\t\t(dielectric_constraints no)
\t\t)
\t\t(pad_to_mask_clearance 0)
\t\t(allow_soldermask_bridges_in_footprints yes)
\t)
\t(net 0 "")
{nets}
'''

def edges():
    r = OX+BW; b = OY+BH
    g = []
    g.append(f'\t(gr_rect (start {OX} {OY}) (end {r} {b}) (stroke (width 0.1) (type solid)) (fill no) (layer "Edge.Cuts") (uuid "{nuid()}"))')
    g.append(f'\t(gr_text "24 GHz FMCW RADAR  REV D" (at {OX+30} {OY+BH-1.2} 0) (layer "F.SilkS") (uuid "{nuid()}") (effects (font (size 1 1) (thickness 0.15))))')
    return "\n".join(g)

for _r, (_x, _y, _rot) in FIXED.items():
    x0, y0, x1, y1 = _rot_bbox(bbox_of(_r), _rot)
    RECTS.append((OX+_x+x0, OY+_y+y0, OX+_x+x1, OY+_y+y1))

# RF corridor keepouts (absolute mm) — GCPW lines + via fences from
# tools/route_pcb.py must stay clear of auto-slotted passives
RF_KEEPOUT = [
    (78.8, 44.5, 82.1, 58.8),   # RF_TX: U1.11 -> corridor x=80.6 -> U12.2
    (60.9, 44.7, 77.0, 47.6),   # ANT_TX: U12.10 -> west to J3
    (60.9, 56.2, 77.3, 63.9),   # ANT_RX: J4 -> 45deg -> U1.3
]
RECTS.extend(RF_KEEPOUT)
order = list(FIXED) + sorted([r for r in comps if r not in FIXED],
                             key=lambda r: (r[0], int(re.sub(r'\D','',r) or 0)))
blocks = [embed(r) for r in order]
open(OUT, "w").write(header() + "\n".join(blocks) + "\n" + edges() + "\n)\n")
print(f"wrote {OUT}: {len(blocks)} footprints, {len(names)} nets")
