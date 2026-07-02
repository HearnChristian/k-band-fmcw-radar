# Changelog

## Rev C — 2026-07-02

Functional-gap pass: every block needed for a working board (except antenna
feeds) is now present and driven.

### Added
- **U13 (TPS7A4700)** — 3.0 V PA drain LDO for the AMM-8211 (`+3V0_PA` was an
  undriven net). Output set by grounding the 1P6V ANY-OUT pin (1.4 + 1.6 V).
  Fed from +5 V; **EN driven by U9 (ADP5600) PGOOD** so the −0.5 V gate bias is
  established before drain power: gate-before-drain sequencing for the GaAs
  HEMT. AMM-8211 draws 175 mA typ (datasheet EC table, Vd = 3 V); the LDO is
  good for 1 A. C46/C47 input/output caps, C48 noise-reduction cap.
- **U16 (REF5025)** — buffered precision 2.5 V driving `VREF`. Replaces the
  R9/R10 1 kΩ divider: ADS8353 AINM inputs must sit within VREF ± 0.1 V and are
  sampled, so they need a low-impedance source, not a 500 Ω divider. C54 supply
  bypass; existing C34/C35 serve as output caps (REF50xx prefers 1–1.5 Ω ESR —
  verify, or add ~1 Ω in series at layout).
- **R23/C49, R24/C50** — anti-alias / charge-reservoir RC (49.9 Ω + 10 nF C0G,
  fc ≈ 320 kHz) between the OPA1656 outputs and ADS8353 AINP_A/B
  (new nets `ADC_A`/`ADC_B`).
- **U14 (TPS7A2033)** — second 3.3 V LDO creating `+3V3_DIG` for the FT2232
  (VCCIO ×4, VPHY, VPLL, VREGIN), EEPROM, and the MUXOUT pull-up, with
  C13–C15/C30–C32 moved over. `+3V3_RF` now feeds only ADF4159, BGT24LTR11 and
  the TCXO. Previously one 300 mA LDO carried ~230–280 mA of combined load and
  put USB switching noise on the PLL/VCO rail.
- **C51 (100 pF)** — AC coupling between the TCXO output and ADF4159 REFIN
  (new net `ADF_REFIN`), per ADF4159 reference-input practice.
- **D1 (SS34)** — reverse-polarity Schottky after the barrel jack (new net
  `VIN_RAW`). Recommended supply ~6 V: at 12 V input, U6 would dissipate ~4 W.
- **U15 (TPD2E009)** — USB ESD protection array on USB_DP/USB_DM.
- **J3/J4** — board-side RF ports for `ANT_TX`/`ANT_RX`. At 24 GHz use 2.92 mm
  (K) end-launch connectors; the assigned stock SMA edge-mount footprint is a
  **placeholder** to be replaced during layout. No series DC blocks needed —
  the AMM-8211 RF ports are internally DC-blocked (Marki datasheet).

### Changed
- **U9 (ADP5600) VIN/EN moved from raw VIN to +5 V.** Absolute max input is
  5.5 V while the TPS7A4700 needs > 5.3 V input to regulate — no valid supply
  voltage satisfied both. Input bypass C10 moved with it.
- **R6 (BGT24LTR11 R_TUNE) 10 k → 16 k** — datasheet spec condition for the
  24.05–24.25 GHz VCO range.
- Footprints assigned to **all** passives (0603; 10 µF/3.3 µF in 0805) and J2.
- Sheet A4 → A3 (content overflowed the A4 border).
- New symbols live in the project library `RadarExtra.kicad_sym`.

### Firmware notes
- ADS8353: CFR.B7 = 1, CFR.B9 = 1 (pseudo-differential, 2×VREF range,
  AINM = 2.5 V, AINP swings 0–5 V); single-SDO 32-CLK mode.
- ADF4159: configure MUXOUT as open-drain (R22 pulls up to 3.3 V digital rail).

### Still open
- Antenna feed line design + real 2.92 mm end-launch footprints (layout).
- PLL loop filter values are placeholders — recompute from Kvco/Icp/N.
- IF gain ×11 is a placeholder pending link-budget analysis.
- REF5025 output-cap ESR check (see above).

## Rev B — 2026-07-01

Datasheet-verification pass over the fully wired schematic (ERC 50 → 11).

- ADF4159 VP 5 V → 3.3 V (4 V abs max — was over-stressed); SDVDD tied to +1V8
  (was floating).
- U4/U5 (6 V-max LDOs) re-fed from +5 V instead of raw VIN.
- AMM-8211 Vg moved from raw charge-pump output to the ADP5600 regulated
  −0.505 V LDO output (SEL1 = SEL2 = GND fixed mode, R17 → 0 R); Vd1–3 moved to
  new `+3V0_PA` net (LDO added in Rev C).
- FT2232: C1–C3 rewired as VCORE decouplers (100 nF), C16 3.3 nF → 3.3 µF,
  VBUS sense divider (R2/R20) into RESET# for self-powered configuration,
  R21 EEPROM DO isolation, R22 MUXOUT pull-up.
- Bulk caps C43–C45; VREF divider stiffened (superseded by REF5025 in Rev C);
  29 no-connect flags; GND moved onto J1 B1/A12 (was 3.81 mm off-pin).

## Rev A — 2026-06-26

Initial full wiring: 8 → 58 connected nets, 43 components added. Regulators,
TCXO reference, PLL loop filter + DIV_OUT feedback, TX → AMM-8211, IF stage,
SPI/USB/EEPROM, ADP5600 → −VGG gate bias.
