# References and External Resources

Annotated bibliography for the CSNE-SoC project.
Papers marked **[core]** are directly cited in the paper.

---

## Foundational — Systolic Arrays

**[core]** Kung, H.T. (1982).
*Why systolic architectures?*
IEEE Computer, 15(1), 37–46.
https://doi.org/10.1109/MC.1982.1653825
> The original paper defining systolic array architecture. Essential citation
> for any work on systolic MAC arrays.

Kung, H.T. & Leiserson, C.E. (1978).
*Systolic arrays (for VLSI).*
Sparse Matrix Proceedings, 256–282.
> First formal description of the systolic execution model.

---

## Neural Network Accelerators

**[core]** Jouppi, N.P. et al. (2017).
*In-datacenter performance analysis of a tensor processing unit.*
ISCA 2017.
https://doi.org/10.1145/3079856.3080246
> Google TPU paper — the most cited systolic array accelerator for ML.
> Key reference for the MAC array architecture motivation.

Chen, Y. et al. (2016).
*Eyeriss: A spatial architecture for energy-efficient dataflow
for convolutional neural networks.*
ISCA 2016.
https://doi.org/10.1109/ISCA.2016.40
> Row-stationary dataflow, highly relevant for systolic array scheduling.

Sze, V. et al. (2017).
*Efficient processing of deep neural networks: A tutorial and survey.*
Proceedings of the IEEE, 105(12), 2295–2329.
https://doi.org/10.1109/JPROC.2017.2761740
> Best survey on hardware accelerators for DNNs. Read this first.

---

## FPGA Accelerators

Qasaimeh, M. et al. (2019).
*Comparing energy efficiency of CPU, GPU and FPGA implementations
for vision kernels.*
IEEE ICESS 2019.
> Good reference for CPU vs GPU vs FPGA energy comparison.

Venieris, S.I. & Bouganis, C.S. (2018).
*fpgaConvNet: Mapping regular and irregular convolutional neural networks
on FPGAs.*
IEEE TNNLS, 30(1), 326–342.
> FPGA CNN mapping framework, useful background.

---

## SoC-FPGA and HPS Integration

**[core]** Intel/Altera. (2020).
*Cyclone V Hard Processor System Technical Reference Manual.*
Document: cv_5v4.
https://www.intel.com/content/dam/www/programmable/us/en/pdfs/literature/hb/cyclone-v/cv_5v4.pdf
> Official TRM for the Cyclone V HPS. Source for all bridge base addresses
> and memory map values used in this project.

**[core]** Intel/Altera. (2021).
*Avalon Interface Specifications.*
Document: MNL-AVABUSREF.
https://www.intel.com/content/dam/www/programmable/us/en/pdfs/literature/manual/mnl_avalon_spec.pdf
> Defines the Avalon-MM protocol implemented by IP_PE.

**[core]** Terasic. (2016).
*DE10-Nano User Manual.* Rev 1.3.
https://ftp.intel.com/Public/Pub/fpgaup/pub/Intel_Material/Boards/DE10-Nano/DE10_Nano_User_Manual.pdf
> Board schematics, pin assignments, clock sources.

---

## Edge Computing and Embedded Inference

Lin, J. et al. (2020).
*MCUNet: Tiny deep learning on IoT devices.*
NeurIPS 2020.
https://arxiv.org/abs/2007.10319
> TinyML on microcontrollers — useful contrast with FPGA approach.

Blott, M. et al. (2018).
*FINN-R: An end-to-end deep-learning framework for fast exploration
of quantized neural networks.*
ACM TRETS, 11(3).
https://doi.org/10.1145/3242897
> FPGA inference with quantized networks (INT8 and below).

---

## Related GitHub Repositories

| Repository | Description | Relevance |
|---|---|---|
| [google/gemmlowp](https://github.com/google/gemmlowp) | Low-precision matrix multiply | INT8 MAC reference |
| [Xilinx/finn](https://github.com/Xilinx/finn) | FPGA neural network inference | End-to-end comparison target |
| [tensorflow/tensorflow](https://github.com/tensorflow/tensorflow) | TensorFlow | SW baseline reference |
| [openhwgroup/cva6](https://github.com/openhwgroup/cva6) | Open RISC-V SoC | SoC integration reference |

---

## Standards and Specifications

| Document | Version | Source |
|---|---|---|
| IEEE Std 1076-2008 (VHDL) | 2008 | IEEE Xplore |
| Avalon Interface Specifications | 2021.04 | Intel FPGA |
| ARM AMBA AXI Protocol | v2.0 | ARM IHI0022 |
| Cyclone V Device Handbook | 2020 | Intel FPGA |

---

*Last updated: 2026-05-07*
