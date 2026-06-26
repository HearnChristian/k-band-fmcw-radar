# K-Band FMCW Radar

A frequency-modulated continuous-wave (FMCW) radar front-end designed in KiCad —
a single-board transceiver intended for **ground-based ranging and weather /
precipitation sensing**. This repository holds the schematic capture, the
project's custom KiCad libraries, and the generated netlist. Work in progress.

## Architecture

FMCW radar works by transmitting a continuous tone whose frequency is swept
(chirped) linearly in time, then mixing the received echo against the
transmitted signal. The frequency of the resulting beat tone is proportional to
target range, so the whole signal chain is built around generating a clean,
linear chirp and digitizing the low-frequency beat note.

```
  Reference        Chirp synth        RF transceiver        Driver amp
  ┌────────┐      ┌──────────┐       ┌──────────────┐      ┌──────────┐
  │  TCXO  │─────▶│ ADF4159  │──────▶│ BGT24LTR11N16│─────▶│AMM-8211PSM│──▶ TX
  │ ref clk│      │ PLL/ramp │       │ 24 GHz Tx/Rx │      │ driver PA │
  └────────┘      └──────────┘       └──────┬───────┘      └──────────┘
                                            │ IF (beat)
                                            ▼
                       ┌──────────┐   ┌──────────┐   ┌──────────┐
                  Rx ──│ OPA1656  │──▶│ ADS8353  │──▶│ FT2232H  │──▶ USB host
                       │ IF amp   │   │ 16b ADC  │   │ USB 2.0  │
                       └──────────┘   └──────────┘   └──────────┘
```

## Key components

| Block | Part | Function |
|-------|------|----------|
| Chirp synthesizer | **ADF4159** | Fractional-N PLL with onboard ramp generator — produces the linear FMCW sweep |
| RF transceiver | **BGT24LTR11N16** | 24 GHz SiGe transmit/receive front-end with integrated VCO and downconversion |
| Driver amplifier | **AMM-8211PSM** | 22–57 GHz GaAs MMIC driver amp boosting the transmit chain |
| IF / baseband | **OPA1656** | Low-noise, low-distortion op-amp conditioning the beat-note signal |
| Data converter | **ADS8353** | Dual 16-bit SAR ADC sampling the IF |
| Host interface | **FT2232HL** | USB 2.0 bridge for control + data streaming |
| Reference clock | **ECS-TXO-2520MV** | TCXO frequency reference |
| Power | **TPS7A4700 / TPS7A2018 / TPS7A2033** | Ultra-low-noise LDOs for clean RF/analog rails |
| Config | **93LC56B** | EEPROM for FT2232 descriptor |

## Custom library work

The **AMM-8211PSM** driver amplifier had no vendor-supplied EDA data — only a
datasheet — so its schematic symbol and PCB footprint were built from scratch
from the Marki Microwave mechanical drawings:

```
AMM-8211PSM.kicad_sym                                          custom symbol
AMM-8211PSM.pretty/QFN-16-1EP_3x3mm_P0.5mm_EP1.1x1.1mm_Marki.kicad_mod   custom footprint
```

The footprint is a 3 × 3 mm 16-lead QFN, 0.5 mm pitch, with a 1.10 × 1.10 mm
exposed ground paddle and a 3 × 3 array of thermal vias, matching the Marki
outline drawing and pin assignment.

## Repository layout

```
Radar1.kicad_pro      KiCad project
Radar1.kicad_sch      Schematic (in progress)
Radar1.kicad_pcb      PCB layout (not yet started)
Radar1.net            Exported netlist
AMM-8211PSM.kicad_sym Custom symbol (see above)
AMM-8211PSM.pretty/   Custom footprint library
sym-lib-table         Project symbol-library table
fp-lib-table          Project footprint-library table
```

## Tools

Designed in [KiCad](https://www.kicad.org/) 9. Open `Radar1.kicad_pro` to view
the schematic.

---

*Author: Christian Hearn*
