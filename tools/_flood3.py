import math, pcbnew
from pcbnew import ToMM
SRC = "/home/christian-thomas-hearn/Desktop/X-BAND FMCW RADAR/Radar1/tools/route_signals.py"
src = open(SRC).read().split("# ---------- escape stubs")[0]
G = {}
exec(compile(src, "core", "exec"), G)
b, cell, xy, NX, NY = G["b"], G["cell"], G["xy"], G["NX"], G["NY"]
G["build_maps"]()
maps, via_ok = G["maps"], G["via_ok"]

code = b.FindNet("-VGG").GetNetCode()
hw = 0.075
m0, m1 = maps[(0, hw)], maps[(1, hw)]
def ok(lay, ix, iy):
    v = (m0 if lay == 0 else m1)[ix*NY+iy]
    return v == 0 or v == code

def flood(seeds, viahop):
    seen = set(s for s in seeds if 0 <= s[1] < NX and 0 <= s[2] < NY and ok(*s))
    stack = list(seen)
    nvia = 0
    while stack:
        lay, ix, iy = stack.pop()
        for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
            jx, jy = ix+dx, iy+dy
            nb = (lay, jx, jy)
            if 0 <= jx < NX and 0 <= jy < NY and nb not in seen and ok(lay, jx, jy):
                seen.add(nb); stack.append(nb)
        if viahop:
            nb = (1-lay, ix, iy)
            if nb not in seen and ok(*nb) and via_ok(code, hw, ix, iy):
                nvia += 1; seen.add(nb); stack.append(nb)
    return seen, nvia

six, siy = cell(78.25, 47.4)   # stray: U12.6 pocket pad
mix, miy = cell(80.18, 71.65)  # a main-cluster pad near U9
stray, sv = flood([(0, six+dx, siy+dy) for dx in range(-2,3) for dy in range(-2,3)], True)
main, mv = flood([(0, mix+dx, miy+dy) for dx in range(-2,3) for dy in range(-2,3)], True)
for name, r, nv in (("stray", stray, sv), ("main", main, mv)):
    xs = [xy(ix,iy)[0] for (_,ix,iy) in r]; ys = [xy(ix,iy)[1] for (_,ix,iy) in r]
    nB = sum(1 for (l,_,_) in r if l == 1)
    print(f"{name}: {len(r)} cells ({nB} on B, {nv} via-hops) "
          f"x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}]")
print("intersect:", len(stray & main))
