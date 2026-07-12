# PCB Plan — Stackup Trade & Floorplan (pre-layout)

Base: schematic `21be641` (electrically identical to Rev-C `4aef776`, netlist-verified).
Date: 2026-07-09.

## Substrate decision

Only the three 24 GHz runs (TX chain to J3, RX feed from J4) and the 1.5 GHz
divided LO care about the substrate; everything else is DC/SPI/IF.

| Option | Dk | tan δ | ~Loss @24 GHz | Z₀ tol | Verdict |
|---|---|---|---|---|---|
| All FR4 | 4.2–4.6 (uncontrolled) | ~0.020 | 0.5–0.8 dB/cm | ±8–10 % | Reject — loss + Dk uncertainty eat the PA gain budget |
| **Hybrid RO4350B (10 mil) over FR4** | 3.48 ± 0.05 | 0.0037 | 0.10–0.15 dB/cm | ±2–3 % | **Pick** — all RF is on L1/L2; FR4 below carries power/digital |
| Full RO4350B | 3.48 ± 0.05 | 0.0037 | 0.10–0.15 dB/cm | ±2–3 % | Overkill — inner layers are DC/digital |

**Stackup (≈1.5 mm, ENIG):** L1 RF/signal — RO4350B 0.254 mm — L2 solid GND —
FR4 core ≈0.9 mm — L3 power — prepreg — L4 signal/GND.
50 Ω GCPW starting point: w ≈ 0.50 mm, gap ≈ 0.25 mm (field-solve against fab stackup).

## Floorplan (board ≈ 66 × 46 mm)

- **West edge:** J3 (TX) / J4 (RX) end-launch, ≥26.5 GHz rated (2.92 mm preferred).
  U12 PA driver between U1 and J3; U1 BGT24 center-west. 24 GHz GCPW with via
  fences (pitch ≤ 0.8 mm) both sides; stitching ring around RF zone.
- **PLL:** U2 ADF4159 east of U1; loop filter tight, guarded, nothing switching
  within 5 mm; Y1 50 MHz TCXO short/direct into REFIN.
- **Center:** IF chain U1 → U3 (OPA1656 ×3) → U8 ADS8353; U15 REF5025 adjacent.
- **East edge:** U7 FT2232H, U11 12 MHz, U10 EEPROM, J1 USB-C.
- **South strip:** J2 barrel (SE) → U6 5 V LDO (+ thermal pour) → LDO row.
  U9 ADP5600 (−VGG charge pump, the only switcher) far corner from RX, LC
  post-filter to PA gate. +3V3_RF / +3V3_DIG copper split mirrors the Rev-C rail split.
- Thermal: EP via arrays under U1, U12, U6, U13.

## Blockers before placement

1. **CRITICAL — ADF4159 SPI overvoltage.** FT2232H drives CLK/DATA/LE/CE/TXDATA
   at 3.3 V; ADF4159 digital abs-max = DVDD + 0.3 V = 2.1 V (datasheet Table 4).
   MUXOUT readback (1.8 V logic → 3.3 V-bank input) is marginal in reverse.
   Fix (no new parts): power FT2232H ADBUS-bank VCCIO from +1V8 (VCCIO spec
   1.62–3.63 V), and move ADS8353 DVDD from +3V3_RF to +1V8 (also removes ADC
   digital noise from the RF rail).
2. Footprint pass: ~12 symbols unassigned; 5 libs missing from `fp-lib-table`.
3. Sync embedded symbols with libs (USB4215-03-A, AMM-8211PSM mismatch).
4. Spec barrel input 6 V nominal (≤6.5 V) or add buck before U6 — LDO drops
   (VIN−5 V) × ~0.45 A.
5. Optional: route U1 V_PTAT to a spare ADC channel for temperature telemetry.

## Verified correct in this review

PA sequencing (ADP5600 PGOOD gates +3V0_PA enable), PA bias (Vd = 3 V/175 mA =
datasheet nominal), TPS7A4700 straps (5.0 V / 3.0 V exact), FT2232 VCORE loop,
ADF4159 rail split (AVDD 3.3 / DVDD 1.8). ERC "errors" are symbol pin-type
pedantry; netlist has no floating nets.
