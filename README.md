# CIC Filter — Configurable Decimator / Interpolator

A parametrable **CIC (Cascaded Integrator–Comb)** filter, provided in **two equivalent
RTL implementations** (Verilog-2001 and VHDL-2008), with a parametrable testbench for
each language and a **bit-exact Python model** used to verify the HDL output.

The same module implements either a **decimator** or an **interpolator**, selected by a
single generic/parameter (`CONFIG`).

---

## Table of contents

- [Getting the repository](#getting-the-repository)
- [Overview](#overview)
- [Theory background](#theory-background)
- [Interface](#interface)
  - [Parameters / generics](#parameters--generics)
  - [Ports](#ports)
- [Simulation](#simulation)
- [References](#references)

---

## Getting the repository

```bash
git clone https://github.com/Anelay02/CIC-implementation-Verilog-VHDL-.git
```

The Python model additionally needs:

```bash
pip install matplotlib
```

---

## Overview

Repository layout:

```
.
├── Img/
│   ├── schematic.png        # CIC structure, decimator and interpolator
│   └── cic_compare.png       # Python model vs HDL simulation
├── python/
│   └── cic_model.py         # bit-exact model + CSV comparison + plot
├── results/
│   ├── tb_CIC.csv            # reference simulation log
│   └── tb_CIC.vcd            # reference waveform dump
├── rtl/
│   ├── CIC.v                 # Verilog-2001 implementation
│   └── CIC.vhd                # VHDL-2008 implementation
└── tb/
    ├── tb_CIC.v               # Verilog testbench
    └── tb_CIC.vhd             # VHDL testbench
```

| Item | Description |
|---|---|
| `rtl/CIC.v` `rtl/CIC.vhd` | Verilog & VHDL implementation |
| `tb/tb_CIC.v` `tb/tb_CIC.vhd` | Parametrable testbenches |
| `python/cic_model.py` | Bit-exact Python model of the RTL; reads the CSV produced by the simulator, re-runs the same scenario in Python, compares sample by sample and plots the result |

Key characteristics of the design:

- Order `N`, rate-change ratio `OSR` and data widths are all parameters.
- Differential delay is fixed to **M = 1**.

---

## Theory background

### From the delay sum to the integrator–comb form

The elementary block of a CIC is a plain **boxcar (moving-average) filter** of length
`R = OSR`, whose impulse response is `R` consecutive ones:

$$
y[n] = \sum_{k=0}^{R-1} x[n-k]
\qquad\Longrightarrow\qquad
H(z) = \sum_{k=0}^{R-1} z^{-k}
$$

That sum is a **geometric series** of common ratio $z^{-1}$, so it has a closed form:

$$
\sum_{k=0}^{R-1} z^{-k} = \frac{1-z^{-R}}{1-z^{-1}}
$$

This single identity is the whole idea behind the CIC. It splits a filter that would
naively need `R-1` additions per sample into two trivial pieces:

$$
H(z) = \underbrace{\left(1-z^{-R}\right)}_{\text{comb}}\cdot
       \underbrace{\frac{1}{1-z^{-1}}}_{\text{integrator}}
$$

- the **integrator** $1/(1-z^{-1})$ is an accumulator: `y[n] = y[n-1] + x[n]`;
- the **comb** $1-z^{-R}$ is a differentiator: `y[n] = x[n] - x[n-R]`.

Cascading `N` such sections gives the `N`-th order CIC:

$$
H(z) = \left(\frac{1-z^{-R}}{1-z^{-1}}\right)^{N}
     = \left(1-z^{-R}\right)^{N}\cdot\frac{1}{\left(1-z^{-1}\right)^{N}}
$$

All `N` integrators can be grouped on one side and all `N` combs on the other.
That regrouping is what makes the rate change free.

![CIC schematic — decimator and interpolator](Img/schematic.png)

- **Decimator** — Integrators run at the **high** rate, then decimate by `R`, then the
  combs run at the **low** rate.

- **Interpolator** — The combs run first at the **low** rate, the signal is
  **zero-stuffed** by `R`, and the integrators run at the **high** rate.

The generic/parameter `CONFIG` selects which of the two configurations is generated.

### Register sizing

The registers must be wide enough that the **largest value reachable anywhere in the
cascade** still fits over `OSR` samples. (The gain is lower in the interpolator
architecture because it is constrained by the zero-stuffing process.)

**Decimator** — worst-case gain $R^{N}$:

$$
B_{reg} = N\log_2(R) + B_{in}
$$

**Interpolator** — worst-case gain $R^{N-1}$:

$$
B_{reg} = (N-1)\log_2(R) + B_{in}
$$

Which is precisely the `REG_SIZE` localparam / constant in the RTL:

```verilog
localparam LOG_OSR  = $clog2(OSR);
localparam REG_SIZE = (CONFIG == "interpolator") ? (N-1)*LOG_OSR + INPUT_SIZE
                                                 :  N   *LOG_OSR + INPUT_SIZE;
```

Note that **every** register in the design — every integrator and every comb, in both
sections — is `REG_SIZE` bits wide. Hogenauer's per-stage pruning (giving each stage only
as many bits as it actually needs) is *not* implemented here.

### Why does it work?

An integrator has a pole *on* the unit circle at `z = 1`: its DC gain is infinite, so its
register **will** overflow and wrap around, and it will keep wrapping forever.

This is harmless, for a precise reason: all the arithmetic here is unsigned
two's-complement arithmetic modulo $2^{B_{reg}}$. The combs subtract the wrapped values
from each other, and the wrapping cancels exactly.

The practical consequence: **no saturation logic, no overflow flags and no wider
accumulators are needed.** The intermediate integrator outputs are meaningless on their
own if you probe them in a waveform viewer, but the output of the last comb is exact.

### Output truncation

All the stages are `REG_SIZE` bits, while the output port is defined by `OUTPUT_SIZE`.
The RTL keeps the **MSBs** and drops the `LSB_OUT` lowest bits:

```verilog
localparam LSB_OUT = (REG_SIZE > OUTPUT_SIZE) ? REG_SIZE - OUTPUT_SIZE : 0;
...
o_data <= final_stage[N*REG_SIZE-1 : LSB_OUT + (N-1)*REG_SIZE];
```

This is a plain truncation and costs no hardware, though it adds a small DC offset plus a
truncation noise floor. If `OUTPUT_SIZE ≥ REG_SIZE`, `LSB_OUT` is `0` and the result is
simply zero-extended: no information is lost, and the full gain appears at the output.

---

## Interface

Both implementations expose an identical interface.

### Parameters / generics

| Name | Type | Default | Description |
|---|---|---|---|
| `N` | integer | `3` | **Order** of the filter: number of integrator stages *and* number of comb stages. Sets the steepness of the response and the depth of the nulls; also sets the passband droop and the register growth. `N ≥ 1`. |
| `OSR` | integer | `512` | **Rate-change ratio** `R`. Decimation factor in decimator mode, interpolation factor in interpolator mode. **Must be a power of two** — the low-rate enable is produced by all-ones decoding of a `$clog2(OSR)`-bit counter, which is only periodic-by-`OSR` for powers of two. `OSR ≥ 2`. |
| `INPUT_SIZE` | integer | `1` | Width of `i_data`, in bits. Data is treated as **unsigned**. The default of `1` targets the classic use case of filtering a 1-bit sigma-delta bitstream. |
| `OUTPUT_SIZE` | integer | `16` | Width of `o_data`, in bits, **unsigned**. If smaller than `REG_SIZE` the result is truncated from the LSB side (see above); if larger, it is zero-extended. |
| `CONFIG` | string | `"decimator"` | Selects the topology: `"interpolator"` selects the interpolator branch, **any other value** selects the decimator branch. |

`REG_SIZE`, `LOG_OSR` and `LSB_OUT` are derived internally and are not meant to be
overridden.

### Ports

| Name | Dir | Width | Description |
|---|---|---|---|
| `i_clk` | in | 1 | Clock. Everything is on the **rising edge**. This is the **high-rate** clock in both modes; the low rate is derived with an enable, so there is a single clock domain. |
| `i_rst_n` | in | 1 | **Asynchronous, active-low** reset. Clears every integrator, every comb, the sample counter and the output registers. |
| `i_data` | in | `INPUT_SIZE` | Unsigned input sample. |
| `i_valid` | in | 1 | Input sample enable. **The whole filter only advances when `i_valid` is high** — the sample counter, the integrators and (via `accept_sample`) the combs are all gated by it. Holding it low freezes the filter without disturbing its state, so it doubles as a clock-enable. |
| `o_data` | out | `OUTPUT_SIZE` | Unsigned output sample, **registered**. Holds its previous value between updates. |
| `o_valid` | out | 1 | **Registered** flag, high for the one cycle during which `o_data` is new. |
| `o_ready` | out | 1 | Input-side flow control, **combinational**. See below. |

---

## Simulation

Both testbenches instantiate **one decimator and one interpolator at the same time**,
sharing a clock and a reset, drive both with a full-scale sine, and log the stimulus and
both outputs to `tb_CIC.csv` on every clock edge after reset.

Verified with **VHDL-2008** and **Verilog-2001** in **Questa-Lattice 2025.2**.

Testbench parameters (identical in both languages, edit them at the top of the file):

| Parameter | Default | Description |
|---|---|---|
| `CSV_ENABLE` | `1` / `true` | Enable CSV logging |
| `CSV_FILE` | `"tb_CIC.csv"` | Output CSV name |
| `N` | `3` | Order used for **both** DUTs |
| `OSR_DEC` | `64` | Decimator OSR (power of two) |
| `OSR_INT` | `64` | Interpolator OSR (power of two) |
| `INPUT_SIZE` | `8` | Input width for both DUTs |
| `OUTPUT_SIZE` | `16` | Output width for both DUTs |
| `CLK_PERIOD_NS` | `10.0` | Clock period, ns (100 MHz) |
| `F_SIGNAL_HZ` | `50_000.0` | Sine frequency — must stay well below `f_clk / (2·OSR)` |
| `NB_PERIODS` | `4` | Number of sine periods to simulate |

### Checking against the Python model

`python/cic_model.py` is a **bit-exact model of both the RTL and the testbench**: it
reimplements the register-transfer behaviour stage by stage and reproduces the same sine
stimulus with the same rounding. It reads the CSV the simulator produced, regenerates
the equivalent data in memory and compares them row by row.

Its parameters are set directly in the file and **must be kept in sync with the
testbench's**. Paths are relative, so run it from inside `python/`:

```bash
cd python
python cic_model.py
```

The script prints the results of the comparison and plots `Img/cic_compare.pdf`:

```
N=3 OSR_DEC=64 OSR_INT=64 INPUT_SIZE=8 OUTPUT_SIZE=16
samples_per_period=31  run_cycles=8192
--- Comparison ../results/tb_CIC.csv vs python model ---
  rows: reference=8192  python=8192
  RESULT: BIT-EXACT MATCH
```

![Python model vs HDL simulation](Img/cic_compare.png)

---

## References

E. Hogenauer, *An economical class of digital filters for decimation and interpolation*,
IEEE Transactions on Acoustics, Speech, and Signal Processing, vol. 29, no. 2,
pp. 155–162, 1981. [doi:10.1109/TASSP.1981.1163535](https://doi.org/10.1109/TASSP.1981.1163535)

R. Lyons, *A Beginner's Guide To Cascaded Integrator-Comb (CIC) Filters*, dsprelated.com —
<https://www.dsprelated.com/showarticle/1337.php>
