#!/usr/bin/env python3
# =============================================================================
# File: cic_model.py
# Author:	Ayoub el idrissi Achraf
# Description: Reproduces and compares the output of the HDL testbench.
# =============================================================================

import math
import os
import sys
import matplotlib.pyplot as plt

# =============================================================================
# 1. PARAMETERS - edit these directly
# =============================================================================

N             = 3          # order of the filters (number of stages)
OSR_DEC       = 64         # decimator  oversampling ratio (power of 2)
OSR_INT       = 64         # interpolator oversampling ratio (power of 2)
INPUT_SIZE    = 8          # input data width  (bits)
OUTPUT_SIZE   = 16         # output data width (bits)
CLK_PERIOD_NS = 10.0       # clock period in ns
F_SIGNAL_HZ   = 50_000.0   # sine frequency
NB_PERIODS    = 4          # number of sine periods to simulate

AMPLITUDE = None           # None -> full scale (2^(INPUT_SIZE-1) - 1)
OFFSET    = None           # None -> mid scale  (2^(INPUT_SIZE-1))

if AMPLITUDE is None:
    AMPLITUDE = (2.0 ** (INPUT_SIZE - 1)) - 1.0
if OFFSET is None:
    OFFSET = 2.0 ** (INPUT_SIZE - 1)

# ---- Paths ----
CSV_PATH = "../results/tb_CIC.csv"  # reference CSV produced by the Verilog testbench
PLOT_PATH = "../Img/cic_compare.png"  # where the comparison plot is saved

# ---- Pre-computation ----
t_full    = CLK_PERIOD_NS * 1.0e-9              # full-rate sample period [s]
t_int_low = OSR_INT * CLK_PERIOD_NS * 1.0e-9    # interpolator low-rate period [s]

samples_per_period = max(2, int(1.0 / (F_SIGNAL_HZ * t_int_low) + 0.5))

OSR_MAX       = max(OSR_DEC, OSR_INT)
DRAIN_CYCLES  = 4 * OSR_MAX
RUN_CYCLES    = NB_PERIODS * samples_per_period * OSR_INT + DRAIN_CYCLES
MAX_INPUT_CODE = (2 ** INPUT_SIZE) - 1


# =============================================================================
# 2. THE DUT
# =============================================================================

def cic_new(n, osr, input_size, output_size, config):
    """Create a fresh state dict for one CIC instance (decimator or interpolator)."""

    assert config in ("decimator", "interpolator")

    log_osr  = math.ceil(math.log2(osr))
    reg_size = ((n - 1) * log_osr + input_size) if config == "interpolator" \
               else (n * log_osr + input_size)  # Define the reg_size based on the configuration
    lsb_out  = max(0, reg_size - output_size)   # bits dropped at the output

    return {
        "N": n,
        "OSR": osr,
        "config": config,
        "REG_SIZE": reg_size,
        "LSB_OUT": lsb_out,
        "MASK": (1 << reg_size) - 1,
        "OUT_MASK": (1 << output_size) - 1,
        "samples_counter": 0,
        "integ_stor": [0] * n,   # integrator registers
        "comb_stor": [0] * n,    # comb (differentiator) registers
        "o_data": 0,
        "o_valid": 0,
    }


def cic_step(state, i_data, i_valid):
    """Simulate one rising clock edge with the given inputs, updating state in place."""

    n, mask = state["N"], state["MASK"]
    osr = state["OSR"]

    accept = (state["samples_counter"] == osr - 1) and bool(i_valid) # accept_sample = (&samples_counter) && i_valid

    if state["config"] == "decimator":
        # ---------- Integrators: running sums, fed by the input ----------
        integ, acc = [], i_data
        for i in range(n):
            acc = (acc + state["integ_stor"][i]) & mask
            integ.append(acc)

        # ---------- Combs: differences, clocked at the LOW rate ----------
        comb, c = [], integ[n - 1]
        for i in range(n):
            c = (c - state["comb_stor"][i]) & mask
            comb.append(c)

        final_stage     = comb[n - 1]   # output is taken after the combs
        condition_valid = accept        # one output every OSR input samples

        # ---------- Register updates ----------
        if i_valid:
            state["integ_stor"] = integ                 # integrators run at full rate
        if accept:
            # comb_stor[0] samples the last integrator, comb_stor[i] the previous comb
            state["comb_stor"] = [integ[n - 1]] + comb[:n - 1]

    else:  # ---------------------- interpolator ----------------------
        # ---------- Combs first, clocked at the LOW rate ----------
        comb, c = [], i_data
        for i in range(n):
            c = (c - state["comb_stor"][i]) & mask
            comb.append(c)

        stuffed_sample = comb[n - 1] if accept else 0 # Zero stuffing

        # ---------- Integrators, running at the HIGH rate ----------
        integ, acc = [], stuffed_sample
        for i in range(n):
            acc = (acc + state["integ_stor"][i]) & mask
            integ.append(acc)

        final_stage      = integ[n - 1]  # output is taken after the integrators
        condition_valid  = bool(i_valid) # one output every clock cycle

        # ---------- Register updates ----------
        if accept:
            state["comb_stor"] = [i_data] + comb[:n - 1]
        if i_valid:
            state["integ_stor"] = integ

    # ---------- Output registers (common to both configs) ----------
    if condition_valid:
        state["o_data"] = (final_stage >> state["LSB_OUT"]) & state["OUT_MASK"]
    state["o_valid"] = 1 if condition_valid else 0

    # samples_counter increments on every valid input, wraps modulo OSR
    if i_valid:
        state["samples_counter"] = (state["samples_counter"] + 1) % osr


# =============================================================================
# 3. THE TESTBENCH  (bit-exact model of tb_CIC.v)
# =============================================================================

def run_testbench():
    """Run the whole simulation in memory."""

    dec    = cic_new(N, OSR_DEC, INPUT_SIZE, OUTPUT_SIZE, "decimator")
    interp = cic_new(N, OSR_INT, INPUT_SIZE, OUTPUT_SIZE, "interpolator")

    in_mask = (1 << INPUT_SIZE) - 1

    # ---- Stimulus generator state (the two `always` driver blocks) ----
    dec_i_valid, dec_i_data, dec_cyc_count = 0, int(OFFSET) & in_mask, 0
    int_i_valid, int_i_data = 0, int(OFFSET) & in_mask
    int_cyc_count, int_low_sample_count = 0, 0

    first_edge_ns = 4.5 * CLK_PERIOD_NS # Reset is released at t = 4*CLK, so the first logged edge is at 4*CLK + CLK/2 = 4.5*CLK

    rows = []
    for k in range(RUN_CYCLES):
        # ---------------------------------------------------------------
        # (a) LOG: print the current state, before this clock edge acts.
        # ---------------------------------------------------------------
        time_ns = first_edge_ns + k * CLK_PERIOD_NS
        rows.append((time_ns,
                     dec_i_data, dec["o_valid"], dec["o_data"],
                     int_i_data, interp["o_valid"], interp["o_data"]))

        # ---------------------------------------------------------------
        # (b) CLOCK THE DUTs with the inputs that are present right now.
        # ---------------------------------------------------------------
        cic_step(dec, dec_i_data, dec_i_valid)
        cic_step(interp, int_i_data, int_i_valid)

        # ---------------------------------------------------------------
        # (c) CLOCK THE STIMULUS GENERATORS.
        # ---------------------------------------------------------------

        # --- Decimator driver: a new sine sample on EVERY clock cycle ---
        if dec_i_valid:
            sine = OFFSET + AMPLITUDE * math.sin(
                2.0 * math.pi * F_SIGNAL_HZ * (dec_cyc_count * t_full))
            code = int(sine + 0.5)                       # round-half-up, then truncate
            code = min(max(code, 0), MAX_INPUT_CODE)       # saturate to the input range
            dec_i_data = code & in_mask
            dec_cyc_count += 1
        dec_i_valid = 1

        # --- Interpolator driver: a new sine sample every OSR_INT cycles ---
        if int_i_valid:
            if (int_cyc_count % OSR_INT) == 0:
                sine = OFFSET + AMPLITUDE * math.sin(
                    2.0 * math.pi * F_SIGNAL_HZ * (int_low_sample_count * t_int_low))
                code = int(sine + 0.5)
                code = min(max(code, 0), MAX_INPUT_CODE)
                int_i_data = code & in_mask
                int_low_sample_count += 1
            int_cyc_count += 1
        int_i_valid = 1

    return rows


# =============================================================================
# 4. CSV READING / COMPARISON
# =============================================================================

CSV_HEADER = "time_ns,dec_i_data,dec_o_valid,dec_o_data,int_i_data,interp_o_valid,interp_o_data"


def read_csv(path):
    """Read an HDL testbench CSV into the same tuple format used by run_testbench()."""
    rows = []
    with open(path, "r") as f:
        next(f)  # skip header
        for line in f:
            line = line.strip()
            if line:
                v = line.split(",")
                rows.append((float(v[0]),) + tuple(int(x) for x in v[1:]))
    return rows


def compare(ref_rows, py_rows, label=""):
    """Compare two sets of rows column by column. Returns True if identical."""
    names = CSV_HEADER.split(",")
    print(f"--- Comparison {label} ---")
    print(f"  rows: reference={len(ref_rows)}  python={len(py_rows)}")

    if len(ref_rows) != len(py_rows):
        print("  /!\\ DIFFERENT NUMBER OF ROWS")

    mismatches = 0
    for i, (a, b) in enumerate(zip(ref_rows, py_rows)):
        if a != b:
            mismatches += 1
            if mismatches <= 5:   # only print the first few
                diff = [names[j] for j in range(len(a)) if a[j] != b[j]]
                print(f"  line {i+2}: {diff}  ref={a}  py={b}")

    ok = (mismatches == 0 and len(ref_rows) == len(py_rows))
    print("  RESULT: " + ("BIT-EXACT MATCH" if ok else f"{mismatches} mismatching rows"))
    return ok


# =============================================================================
# 5. PLOTTING
# =============================================================================

TEXT_COLOR = "#999999"   # titles, labels, ticks, spines
GRID_COLOR = "#888888"   # gridlines (kept faint via alpha)
REF_COLOR  = "#d2691e"   # HDL CSV trace (opaque, dark saturated orange)
PY_COLOR   = "#1f77b4"   # Python model trace (opaque, saturated blue)


def _style_axes(ax):
    """Apply the readable-on-light-and-dark styling to a single Axes."""
    ax.set_facecolor("none")
    ax.title.set_color(TEXT_COLOR)
    ax.xaxis.label.set_color(TEXT_COLOR)
    ax.yaxis.label.set_color(TEXT_COLOR)
    ax.tick_params(axis="both", colors=TEXT_COLOR)
    for spine in ax.spines.values():
        spine.set_color(TEXT_COLOR)
    ax.grid(alpha=0.25, color=GRID_COLOR)


def plot(ref_rows, py_rows, filename=PLOT_PATH):
    """Overlay the Python model and the HDL CSV outputs."""

    def col(rows, i):
        return [r[i] for r in rows]

    t_ref, t_py = col(ref_rows, 0), col(py_rows, 0)

    fig, ax = plt.subplots(1, 2, figsize=(15, 4.5), sharex=True)
    fig.patch.set_alpha(0.0)  # transparent figure background

    title = fig.suptitle(
        f"CIC: Python model vs HDL CSV  "
        f"(N={N}, OSR_dec={OSR_DEC}, OSR_int={OSR_INT}, "
        f"IN={INPUT_SIZE}b, OUT={OUTPUT_SIZE}b)"
    )
    title.set_color(TEXT_COLOR)

    panels = [
        # (column index, axis, title)
        (3, ax[0], "Decimator output (dec_o_data)"),
        (6, ax[1], "Interpolator output (interp_o_data)"),
    ]
    for idx, axis, subtitle in panels:
        axis.plot(t_ref, col(ref_rows, idx), lw=2.5,
                  color=REF_COLOR, label="HDL CSV")
        axis.plot(t_py, col(py_rows, idx), lw=1.3, ls="--",
                  color=PY_COLOR, label="Python model")
        axis.set_title(subtitle)
        axis.set_xlabel("time [ns]")
        _style_axes(axis)

        legend = axis.legend(loc="upper right", fontsize=8,
                              facecolor="none", edgecolor=TEXT_COLOR)
        for text in legend.get_texts():
            text.set_color(TEXT_COLOR)

    fig.tight_layout()
    fig.savefig(filename, dpi=110, transparent=True)
    print(f"Plot saved to {filename} (transparent, light/dark-mode friendly)")

    # ---- Error stats (printed instead of plotted) ----
    n = min(len(ref_rows), len(py_rows))
    for idx, title in [(3, "Decimator"), (6, "Interpolator")]:
        err = [py_rows[i][idx] - ref_rows[i][idx] for i in range(n)]
        max_err = max(map(abs, err), default=0)
        mean_err = (sum(err) / n) if n else 0.0
        print(f"  {title} error (python - HDL): max |err| = {max_err}, mean = {mean_err:.4f}")


# =============================================================================
# 6. MAIN
# =============================================================================

def main():
    print(f"N={N} OSR_DEC={OSR_DEC} OSR_INT={OSR_INT} "
          f"INPUT_SIZE={INPUT_SIZE} OUTPUT_SIZE={OUTPUT_SIZE}")
    print(f"samples_per_period={samples_per_period}  run_cycles={RUN_CYCLES}")

    if not os.path.exists(CSV_PATH):
        print(f"(reference CSV '{CSV_PATH}' not found - nothing to compare or plot)")
        sys.exit(1)

    ref_rows = read_csv(CSV_PATH)
    py_rows  = run_testbench()

    compare(ref_rows, py_rows, f"{CSV_PATH} vs python model")
    plot(ref_rows, py_rows)
    plt.show()


if __name__ == "__main__":
    main()
