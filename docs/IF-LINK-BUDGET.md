# IF Gain vs Link Budget (Rev-F)

Sizing the OPA1656 IF stage between the BGT24LTR11 mixer and the ADS8353
(pseudo-diff vs VREF = 2.5 V, FS = ±2.5 V pk). Reproduce with
`tools/if_link_budget.py`.

## Assumptions

+6 dBm TX (BGT24 typ), 10 dBi per antenna, 1 dB per cable, 250 MHz / 1 ms
chirp (beat = 1.667 kHz/m, FFT bin 1 kHz), mixer Gc 20 dB / NF 10 dB SSB /
IP1dB −28 dBm (DS Table 4), anti-alias 49.9 Ω + 10 nF = 319 kHz.

## Gain choice: ×19 (R13/R16 = 18k, Rg = 1k)

The mixer compresses at −28 dBm input → 126 mVpk at its output. Gain
×19.9 maps that exactly onto ADC full scale, so the converter clips only
when the mixer itself is already compressing — no wasted ADC range, no
ADC-first clipping. E24 choice **×19 = 25.6 dB** (was ×11 placeholder,
which left ~5 dB of ADC range unusable).

Noise: chain noise at the ADC is 190 µVrms (269 nV/√Hz over the AA band)
vs ~44 µVrms ADC noise → solidly thermal-limited; sensitivity is set at
the mixer input and is independent of this gain within reason.

## Coupling corner: C36/C37 100 nF → 560 pF

Into the 100 kΩ VREF bias the old corner was **16 Hz** — passing the TX
leakage beat and closest clutter at full gain, which would dominate the
ADC range. 560 pF puts the corner at **2.8 kHz ≈ 1.7 m** minimum range;
the single pole also provides mild R⁴ compensation below the corner.

## Performance (per 1 kHz bin, 13 dB detection)

| Target | Range | Beat | ADC level (×19) | SNR/bin |
|---|---|---|---|---|
| person (1 m²) | 10 m | 17 kHz | 2.7 mVpk | 47 dB |
| person | 30 m | 50 kHz | 295 µVpk | 28 dB |
| car (10 m²) | 50 m | 83 kHz | 336 µVpk | 29 dB |
| car | 100 m | 167 kHz | 84 µVpk | 17 dB |
| car | 150 m | 250 kHz | 37 µVpk | 10 dB |

Detection range ≈ **70 m (person) / 125 m (car)**; the anti-alias corner
caps the instrumented range at 191 m. Sample both ADC channels at
≥640 kSPS (ADS8353 supports 700 kSPS dual).
