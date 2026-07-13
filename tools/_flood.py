import math, pcbnew
from pcbnew import ToMM
SRC = "/home/christian-thomas-hearn/Desktop/X-BAND FMCW RADAR/Radar1/tools/route_signals.py"
src = open(SRC).read().split("# ---------- escape stubs")[0]
G = {}
exec(compile(src, "core", "exec"), G)
b, cell, xy, NX, NY = G["b"], G["cell"], G["xy"], G["NX"], G["NY"]
G["build_maps"]()
maps = G["maps"]

def flood(code, hw, seeds):
    m0, m1 = maps[(0, hw)], maps[(1, hw)]
    def ok(lay, ix, iy):
        v = (m0 if lay == 0 else m1)[ix*NY+iy]
        return v == 0 or v == code
    seen = set(s for s in seeds if ok(*s))
    stack = list(seen)
    while stack:
        lay, ix, iy = stack.pop()
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            jx, jy = ix+dx, iy+dy
            nb = (lay, jx, jy)
            if 0 <= jx < NX and 0 <= jy < NY and nb not in seen and ok(lay, jx, jy):
                seen.add(nb); stack.append(nb)
        nb = (1-lay, ix, iy)
        # NOTE: no via_ok here — pure layer-change potential
        if nb not in seen and ok(*nb):
            seen.add(nb); stack.append(nb)
    return seen

for netname, seedxy in [("-VGG", (78.25, 47.4)), ("DIVO", (78.5, 58.55)),
                        ("ADF_MUXOUT", (85.95, 60.25))]:
    code = b.FindNet(netname).GetNetCode()
    ix, iy = cell(*seedxy)
    seeds = [(0, ix+dx, iy+dy) for dx in (-1,0,1) for dy in (-1,0,1)]
    r = flood(code, 0.075, seeds)
    if not r:
        print(f"{netname}: seed cells all blocked"); continue
    xs = [xy(ix, iy)[0] for (_, ix, iy) in r]
    ys = [xy(ix, iy)[1] for (_, ix, iy) in r]
    print(f"{netname} from {seedxy}: {len(r)} cells reachable (free layer-hop), "
          f"x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}]")
