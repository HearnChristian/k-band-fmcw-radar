import math, pcbnew
from pcbnew import ToMM
SRC = "/home/christian-thomas-hearn/Desktop/X-BAND FMCW RADAR/Radar1/tools/route_signals.py"
src = open(SRC).read().split("# ---------- escape stubs")[0]
G = {}
exec(compile(src, "core", "exec"), G)
b, cell, xy, NX, NY = G["b"], G["cell"], G["xy"], G["NX"], G["NY"]
G["build_maps"]()
maps, viamap = G["maps"], G["viamap"]
via_ok, pad_goal_cells = G["via_ok"], G["pad_goal_cells"]
pads_by_net = G["pads_by_net"]

STUCK = {"-VGG": (78.25, 47.4), "DIVO": (78.5, 58.55), "+3V3_RF": (79.05, 58.5),
         "ADF_MUXOUT": (85.95, 60.25), "SPI_MOSI": (85.95, 61.25),
         "USB_DM": (120.08, 55.25), "CC1_N": (118.83, 47.8),
         "RFINA_N": (82.05, 61.75), "CS_ADC": (97.92, 46.97),
         "VGG_CP": (78.55, 50.0)}
for netname, (sx, sy) in STUCK.items():
    net = b.FindNet(netname)
    if not net: print(netname, "??"); continue
    code = net.GetNetCode()
    six, siy = cell(sx, sy)
    for hw in (0.075, 0.1):
        m0, m1 = maps[(0, hw)], maps[(1, hw)]
        def ok(lay, ix, iy):
            v = (m0 if lay == 0 else m1)[ix*NY+iy]
            return v == 0 or v == code
        seeds = [(0, six+dx, siy+dy) for dx in range(-2,3) for dy in range(-2,3)]
        seen = set(s for s in seeds if 0 <= s[1] < NX and 0 <= s[2] < NY and ok(*s))
        stack = list(seen)
        while stack:                        # SAME-LAYER flood only
            lay, ix, iy = stack.pop()
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                jx, jy = ix+dx, iy+dy
                nb = (lay, jx, jy)
                if 0 <= jx < NX and 0 <= jy < NY and nb not in seen and ok(lay, jx, jy):
                    seen.add(nb); stack.append(nb)
        vias = sum(1 for (lay, ix, iy) in seen if lay == 0 and via_ok(code, hw, ix, iy)
                   and ok(1, ix, iy))
        if seen:
            xs = [xy(ix,iy)[0] for (_,ix,iy) in seen]; ys = [xy(ix,iy)[1] for (_,ix,iy) in seen]
            print(f"{netname:11} hw={hw}: F-region {len(seen):5} cells "
                  f"x[{min(xs):.1f},{max(xs):.1f}] y[{min(ys):.1f},{max(ys):.1f}] "
                  f"legal-via-cells(with B free): {vias}")
        else:
            print(f"{netname:11} hw={hw}: seeds all blocked")
