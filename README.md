# CSNE-SoC — Configurable Systolic Neural Engine on SoC-FPGA

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-DE10--Nano-blue.svg)](https://www.terasic.com.tw/cgi-bin/page/archive.pl?Language=English&No=1046)
[![FPGA](https://img.shields.io/badge/FPGA-Cyclone%20V%205CSEBA6U23I7-orange.svg)](https://www.intel.com/content/www/us/en/products/details/fpga/cyclone/v.html)
[![HDL](https://img.shields.io/badge/HDL-VHDL--2008-green.svg)]()
[![Status](https://img.shields.io/badge/Status-Active%20Research-brightgreen.svg)]()

> **Paper:** _A Configurable Systolic Neural Engine Core on SoC-FPGA: Architecture, Integration, and Performance Characterization_
> **Authors:** Daniel G. Fajardo Lopez · Eduardo A. Gerlein Rey, Ph.D · Diego Mendez Chaves, Ph.D
> **Institution:** Department of Electronic Engineering, Pontificia Universidad Javeriana, Bogotá D.C., Colombia
> **Target venue:** IEEE (in preparation)

---

## Overview

**CSNE-SoC** is an open-source hardware/software co-design project implementing a scalable systolic array of MAC (Multiply-Accumulate) Processing Elements (PEs) on the Intel Cyclone V SoC-FPGA (Terasic DE10-Nano board).

The design demonstrates the full stack from RTL to bare-metal HPS software:

```
ARM Cortex-A9 (HPS / Linux)
        │
        │  Lightweight HPS-to-FPGA AXI Bridge
        │  (LW-HPS2FPGA @ 0xFF200000)
        │
   ┌────┴────────────────────────────────────┐
   │           FPGA Fabric                   │
   │                                         │
   │  PIO Bridge Layer (Avalon-MM)           │
   │       │                                 │
   │  IP_PE (Avalon-MM Slave Wrapper)        │
   │       │                                 │
   │  PE_TOP ──► FSM_PE ──► Timers           │
   │       │                                 │
   │  [MULT] → [SIGN-EXT] → [ACCUMULATOR]   │
   │   5 cyc      2 cyc        3 cyc         │
   └─────────────────────────────────────────┘
```

### Current status

|   Array size   |     Status     | Measured MOPS | Speedup vs ARM |
| :------------: | :------------: | :-----------: | :------------: |
|     1 × 1      |  ✅ Complete   |     0.160     |    0.14×\*     |
|     3 × 3      | 🔄 In progress |       —       |       —        |
| 9 × 9 (81 PEs) |   📋 Planned   |       —       |       —        |

> \* At 1×1 the bridge overhead dominates. Speedup is projected to exceed 1× at 3×3 and grow quadratically with array size — see [results](#results--benchmarks).

---

## Repository Structure

```
CSNE-SoC/
│
├── README.md               ← this file
├── LICENSE                 ← MIT License
├── CITATION.cff            ← citation metadata (GitHub/Zenodo standard)
│
├── hardware/               ← all synthesizable HDL (VHDL-2008)
│   ├── rtl/
│   │   ├── PE/             ← Processing Element core
│   │   │   ├── PE.vhd          datapath (MULT → SIGN-EXT → ACC)
│   │   │   ├── FSM_PE.vhd      Moore FSM controller
│   │   │   ├── timer_MULT.vhd  5-cycle latency timer
│   │   │   ├── timer_SIG.vhd   2-cycle latency timer
│   │   │   └── timer_ADD.vhd   3-cycle latency timer
│   │   ├── IP_PE/
│   │   │   ├── IP_PE.vhd       Avalon-MM slave wrapper
│   │   │   └── PE_TOP.vhd      top-level PE instantiation
│   │   └── arrays/             (future: 3×3, 9×9 systolic arrays)
│   ├── tb/
│   │   ├── tb_IP_PE.vhd        functional testbench (5 test vectors)
│   │   └── PE_OnChip_Tester.vhd on-chip hardware tester (FPGA-only)
│   ├── constraints/
│   │   └── de10_nano.sdc       timing constraints for Quartus Prime
│   └── platform_designer/
│       └── soc_system.qsys     Platform Designer system (HPS + PIO bridge)
│
├── soc/                    ← HPS software (ARM Linux)
│   ├── driver/
│   │   ├── pe_hps_driver.c     functional test driver (5 test vectors)
│   │   └── pe_benchmark.c      benchmark suite (latency/throughput/comms)
│   ├── scripts/
│   │   └── build.sh            cross-compile helper script
│   └── linux/
│       └── SETUP.md            Linux image setup notes for DE10-Nano
│
├── results/                ← measured data and figures
│   └── 1x1_PE/
│       ├── raw/
│       │   ├── pe_latency_samples.csv
│       │   ├── pe_comms_overhead.csv
│       │   ├── pe_poll_distribution.csv
│       │   └── pe_summary.txt
│       └── figures/
│           ├── fig1_latency_histogram.pdf
│           ├── fig2_latency_cdf.pdf
│           ├── fig3_boxplot_breakdown.pdf
│           ├── fig4_time_breakdown.pdf
│           ├── fig5_poll_histogram.pdf
│           ├── fig6_throughput_scaling.pdf
│           └── fig7_dashboard.pdf
│
├── tools/                  ← analysis and plotting scripts
│   └── pe_plot.py          benchmark visualization (matplotlib, IEEE style)
│
└── docs/                   ← documentation and references
    ├── architecture.md     design description and signal definitions
    ├── address_map.md      Avalon-MM register map reference
    ├── references.md       cited papers and external resources
    └── manuals/            (add datasheets and TRM here — not tracked by git)
```

---

## Hardware Specifications

| Parameter         | Value                                     |
| ----------------- | ----------------------------------------- |
| FPGA device       | Intel Cyclone V SoC 5CSEBA6U23I7          |
| Board             | Terasic DE10-Nano                         |
| FPGA clock        | 50 MHz                                    |
| HPS processor     | ARM Cortex-A9 dual-core @ 925 MHz         |
| HDL standard      | VHDL-2008                                 |
| Bus interface     | Avalon-MM (Intel Platform Designer)       |
| HPS↔FPGA bridge   | Lightweight HPS2FPGA AXI                  |
| Operand width     | 8-bit signed (INT8)                       |
| Accumulator width | 32-bit signed                             |
| Pipeline stages   | 3 (MULT 5 cyc, SIGN-EXT 2 cyc, ACC 3 cyc) |
| Pipeline latency  | 10 clock cycles                           |

---

## Benchmark Results — 1×1 PE

All measurements taken on DE10-Nano running embedded Linux.
Operands: A = 15, B = −7. Sample size: 1 000 iterations.

| Metric                  | Value            |
| ----------------------- | ---------------- |
| Latency mean            | 7 390.7 ns       |
| Latency std-dev         | 1 953.2 ns       |
| Latency P50             | ~7 000 ns        |
| Latency P95             | ~11 000 ns       |
| Throughput              | 0.160 MOPS       |
| Comms overhead (bridge) | 3 036.4 ns (41%) |
| Net compute time        | 4 354.3 ns (59%) |
| SW baseline (ARM)       | 1 052.8 ns       |
| Speedup vs SW           | 0.14×            |

> **Note on speedup:** The 0.14× figure at 1×1 is expected and physically meaningful.
> The LW bridge incurs a fixed ~3 µs overhead per transaction regardless of array size.
> As the array scales to N×N, N² MACs share this fixed cost, and speedup grows as:
> `Speedup(N) ≈ N² · t_sw / (t_comms + N² · t_compute)`

---

## Getting Started

### Prerequisites

- Terasic DE10-Nano board with embedded Linux SD card image
- Intel Quartus Prime (Lite or Standard) ≥ 18.1
- ARM cross-compiler: `arm-linux-gnueabihf-gcc` (or native GCC on board)
- Python ≥ 3.8 with: `matplotlib numpy pandas scipy seaborn`

### 1 — Synthesize the FPGA design

Open Quartus Prime, load the project, and compile:

```bash
# From the hardware/ directory
quartus_sh --flow compile de10_nano_csne
```

Program the board via JTAG or convert to `.rbf` for SD-card boot.

### 2 — Build the HPS driver

```bash
# On the DE10-Nano (native build)
cd soc/driver
gcc -O1 -Wall -Wextra -o pe_hps_driver pe_hps_driver.c
gcc -O1 -Wall -Wextra -lm -o pe_benchmark pe_benchmark.c
```

### 3 — Run the functional test

```bash
sudo ./pe_hps_driver
```

Expected output: 5/5 PASS for test vectors (5×3, −2×4, 10×−3, 127×1, −128×−1).

### 4 — Run the benchmark suite

```bash
sudo ./pe_benchmark
```

Outputs: `pe_latency_samples.csv`, `pe_comms_overhead.csv`,
`pe_poll_distribution.csv`, `pe_summary.txt`

### 5 — Generate figures

```bash
# On your host PC, in the results/1x1_PE/raw/ directory
pip install matplotlib numpy pandas scipy seaborn
python3 ../../../../tools/pe_plot.py
```

Figures saved to `results/1x1_PE/figures/` as PDF + PNG 300 DPI.

---

## Citing This Work

If you use this design or results in your research, please cite:

```bibtex
@misc{fajardo2026csne,
  author       = {Fajardo Lopez, Daniel Giovanni and
                  Gerlein Rey, Eduardo Andres and
                  Mendez Chaves, Diego},
  title        = {CSNE-SoC: A Configurable Systolic Neural Engine Core
                  on SoC-FPGA},
  year         = {2026},
  publisher    = {GitHub},
  url          = {https://github.com/danielfajardo1997/CSNE-SoC}
}
```

See also [`CITATION.cff`](CITATION.cff) for the machine-readable citation file
(used by GitHub's _Cite this repository_ button and Zenodo).

---

## References

Key references for this work — see [`docs/references.md`](docs/references.md)
for the full annotated list.

1. Kung, H.T. (1982). _Why systolic architectures?_ IEEE Computer.
2. Jouppi et al. (2017). _In-datacenter performance analysis of a tensor processing unit._ ISCA.
3. Terasic. _DE10-Nano User Manual._ Rev. 1.3.
4. Intel. _Cyclone V Hard Processor System Technical Reference Manual._
5. Intel. _Avalon Interface Specifications._ MNL-AVABUSREF.

---

## License

This project is licensed under the **MIT License** — see [LICENSE](LICENSE) for details.

You are free to use, modify, and distribute this work for academic and
commercial purposes with attribution.

---

## Authors

| Name                                 | Role            | Institution                      | Contact                         |
| ------------------------------------ | --------------- | -------------------------------- | ------------------------------- |
| **Daniel Giovanni Fajardo Lopez**    | Lead researcher | Pontificia Universidad Javeriana | daniel_fajardo@javeriana.edu.co |
| **Eduardo Andres Gerlein Rey, Ph.D** | Advisor         | Pontificia Universidad Javeriana | egerlein@javeriana.edu.co       |
| **Diego Mendez Chaves, Ph.D**        | Co-advisor      | Pontificia Universidad Javeriana | diego-mendez@javeriana.edu.co   |

Department of Electronic Engineering — Bogotá D.C., Colombia

Questions, collaborations, and feedback welcome via GitHub Issues.
