src = open("/home/christian-thomas-hearn/Desktop/X-BAND FMCW RADAR/Radar1/tools/route_signals.py").read()
src = src.split("# ---------- escape stubs")[0]
g = {}
exec(compile(src, "rs", "exec"), g)
g["build_maps"]()
maps, NX, NY, cell, xy, b = g["maps"], g["NX"], g["NY"], g["cell"], g["xy"], g["b"]
import pcbnew
def dump(lay, xa, xb, ya, yb, netname, hw=0.075):
    code = b.FindNet(netname).GetNetCode()
    m = maps[(lay, hw)]
    print(f"{'F' if lay==0 else 'B'}.Cu hw={hw} net={netname}({code}) x[{xa},{xb}] y[{ya},{yb}]")
    for iy in range(cell(xa,ya)[1], cell(xa,yb)[1]+1):
        row = ""
        for ix in range(cell(xa,ya)[0], cell(xb,ya)[0]+1):
            v = m[ix*NY+iy]
            row += "." if v==0 else ("N" if v==code else ("*" if v==255 else "o"))
        print(f" {40+iy*0.2:5.1f} {row}")
dump(0, 84.5, 91.0, 59.0, 63.0, "ADF_MUXOUT")
dump(1, 84.5, 91.0, 59.0, 63.0, "ADF_MUXOUT")
