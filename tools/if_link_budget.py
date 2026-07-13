"""IF gain vs link budget for the 24 GHz FMCW chain (Rev-F check).

Chain: BGT24LTR11 mixer (Gc 20 dB, NF 10 dB SSB, IP1dB -28 dBm, Zif 1k)
-> AC coupling (C36/C37 into 100k bias @ VREF) -> OPA1656 non-inverting
gain -> 49.9R/10nF anti-alias (319 kHz) -> ADS8353 pseudo-diff vs 2.5 V
(FS = +/-2.5 V about VREF in 2xVREF mode).
Chirp: 250 MHz / 1 ms -> beat 1.667 kHz/m; FFT bin = 1 kHz (1 ms dwell).
"""
import math

PT, GANT, LCBL = 6.0, 10.0, 1.0        # dBm TX, dBi each antenna, dB each cable
LAM, C = 3e8/24.125e9, 3e8
SLOPE = 250e6/1e-3                     # Hz/s
GC, NF, IP1 = 20.0, 10.0, -28.0        # mixer volt-gain dB, NF dB, in P1dB dBm
BIN = 1e3                              # Hz per FFT bin (1 ms chirp)
FS_PK = 2.5                            # ADC full scale, volts peak about VREF
AA = 319e3                             # anti-alias corner

def pr_dbm(r_m, sigma):
    return (PT + 2*(GANT - LCBL)
            + 20*math.log10(LAM) + 10*math.log10(sigma)
            - 30*math.log10(4*math.pi) - 40*math.log10(r_m))

def vpk_adc(pr, gain):
    vin = math.sqrt(10**(pr/10)*1e-3*50)          # rms at RX port (50R)
    return vin*10**(GC/20)*gain*math.sqrt(2)      # pk at ADC

def fbeat(r_m): return 2*SLOPE*r_m/C

# gain so ADC FS is reached exactly when the mixer hits compression
vmix_p1_pk = math.sqrt(10**(IP1/10)*1e-3*50)*10**(GC/20)*math.sqrt(2)
g_opt = FS_PK/vmix_p1_pk
print(f"mixer out at IP1dB: {vmix_p1_pk*1e3:.0f} mVpk -> gain to match ADC FS: x{g_opt:.1f}")

for gain, tag in ((11, "old  x11 (10k/1k)"), (19, "new  x19 (18k/1k)")):
    print(f"--- {tag}: {20*math.log10(gain):.1f} dB ---")
    for r, sig, what in ((2,1,"person 2m"), (10,1,"person 10m"), (30,1,"person 30m"),
                         (50,10,"car 50m"), (100,10,"car 100m"), (150,10,"car 150m")):
        pr = pr_dbm(r, sig)
        v = vpk_adc(pr, gain)
        # SNR per bin: input-referred
        snr = pr - (-174 + NF + 10*math.log10(BIN))
        print(f"  {what:10}: beat {fbeat(r)/1e3:6.1f} kHz  Pr {pr:6.1f} dBm  "
              f"ADC {v*1e6:9.1f} uVpk  SNR/bin {snr:5.1f} dB")
    # noise at ADC: thermal through chain vs ADC noise
    vn = math.sqrt(4e-21*50)*10**((NF)/20)*10**(GC/20)*gain   # V/rtHz at ADC
    vn_tot = vn*math.sqrt(AA*1.57)
    print(f"  chain noise at ADC: {vn*1e9:.0f} nV/rtHz, {vn_tot*1e6:.0f} uVrms "
          f"(ADS8353 ~44 uVrms) -> thermal-limited: {vn_tot > 44e-6}")

# detection range, 13 dB per bin, sigma 1 and 10
for sig in (1, 10):
    lo, hi = 1, 2000
    for _ in range(60):
        mid = math.sqrt(lo*hi)
        if pr_dbm(mid, sig) - (-174 + NF + 10*math.log10(BIN)) > 13: lo = mid
        else: hi = mid
    print(f"13 dB/bin detection range, sigma={sig} m2: {lo:.0f} m "
          f"(beat {fbeat(lo)/1e3:.0f} kHz, AA corner {AA/1e3:.0f} kHz)")
print(f"AA-corner range limit: {AA*C/(2*SLOPE):.0f} m")
print(f"HPF: 100nF@100k = {1/(2*math.pi*1e5*1e-7):.0f} Hz; "
      f"560pF@100k = {1/(2*math.pi*1e5*560e-12)/1e3:.2f} kHz "
      f"(= {1/(2*math.pi*1e5*560e-12)/fbeat(1):.1f} m min range)")
