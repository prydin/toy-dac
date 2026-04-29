# toy-dac

A 1-bit ΔΣ stereo audio DAC implemented on a Xilinx 7-series FPGA
(Digilent Cmod S7), driven from an I²S input.

The signal chain is entirely homemade RTL:

- **I²S receiver** with double-flopped synchronizers and AXI-Stream output
- **Automatic input rate detection** (32 / 44.1 / 48 kHz)
- **Fractional-phase polyphase ASRC** — 256 phases × 64 taps × 18-bit
  coefficients with 16-bit alpha lerp, single time-shared MAC, PI servo
  on the input ring-buffer fill — resampling onto a fixed
  Fs<sub>out</sub> = MCLK / 64 = 1.6875 MHz output grid
- **2nd-order CIFB ΔΣ modulator** (per channel) with TPDF dither,
  driving a single-bit differential output pin pair on each channel
- A small reconstruction filter and analog output stage on the carrier PCB

Internal test sources (silent, DC, 1 kHz DDS sine) and a bypass mode
(raw I²S → modulator) are included for bring-up and characterisation.

Achieved performance with the current build is roughly 70 dB SNR and
−74 dB THD at −7 dBFS, 1 kHz; harmonic distortion is dominated by the
analog stage on the prototype board.

## Status

**Experimental.** This is a personal learning / hobby project. The
design works end-to-end on the target hardware, but it has had limited
testing, no formal verification, and the analog board is a hand-routed
prototype. Module ports, parameter names, and pinouts may change without
notice. Use it for inspiration or as a starting point, not as a drop-in
IP.

## Repository layout

```
fpga/toy-dac/   Vivado project (RTL under src/rtl, constraints under
                toy-dac.srcs/constrs_1, simulation under toy-dac.srcs/sim_1)
kicad/dac/      KiCad schematic + PCB for the analog carrier board
scripts/        Coefficient generation, simulation helpers, sanity checks
docs/           Block diagrams (top-level + ASRC internals)
```

## Building

Open `fpga/toy-dac/toy-dac.xpr` in Vivado 2024.2, run synthesis +
implementation + bitstream generation, and program the Cmod S7. The
target part and pin assignments live in
`fpga/toy-dac/toy-dac.srcs/constrs_1/new/root.xdc`.
