# Architecture Description

## Processing Element (PE) — 1×1

### Datapath pipeline

```
  a_in[7:0] ──┐
               ├──► [INT8 MULTIPLIER] ──► p[15:0]
  b_in[7:0] ──┘         5 cycles
                              │
                    [SIGN EXTENDER 16→32]
                              2 cycles
                              │
                    [32-bit ACCUMULATOR] ◄── feedback
                              3 cycles
                              │
                         acc_out[31:0]
```

### Signal definitions

| Signal | Direction | Width | Description |
|---|---|---|---|
| `clock` | IN | 1 | 50 MHz system clock |
| `reset` | IN | 1 | Active-high synchronous reset |
| `start` | IN | 1 | One-cycle pulse to begin computation |
| `a_in` | IN | 8 | Signed operand A (INT8) |
| `b_in` | IN | 8 | Signed operand B (INT8) |
| `flush` | IN | 1 | Clears accumulator register |
| `ip_ready` | OUT | 1 | High when result is valid |
| `acc_out` | OUT | 32 | Signed accumulator output |
| `a_out` | OUT | 8 | Forwarded A (for systolic chaining) |
| `b_out` | OUT | 8 | Forwarded B (for systolic chaining) |

### FSM states

```
IDLE → LOAD_MULT_INPUT → WAIT_MULT_DONE → UNLOAD_MULT_OUTPUT
     → LOAD_SIGN_INPUT → WAIT_SIGN_DONE → UNLOAD_SIGN_OUTPUT
     → LOAD_SUM_INPUT  → WAIT_SUM_DONE  → UNLOAD_SUM_OUTPUT → IDLE
```

---

## IP_PE — Avalon-MM Wrapper

### Register map

| Offset | Access | Description |
|---|---|---|
| 0x0 | W | Operand A [7:0] |
| 0x2 | W | Operand B [7:0] |
| 0x4 | W | Control — bit 0 = flush |
| 0x6 | W | Start (any write triggers computation) |
| 0x9 | R | Status — bit 0 = ip_ready |
| 0xA | R | acc_out [31:0] |
| 0xB | R | a_out [7:0] zero-extended |
| 0xC | R | b_out [7:0] zero-extended |

---

## HPS↔FPGA Integration

```
Physical address    Component           Export
─────────────────   ─────────────────   ────────────
0xFF200000          pio32_in_0          pe_readdata
0xFF200004          pio32_out_0         pe_address
0xFF200008          pio32_out_1         pe_writedata
0xFF20000C          pio8_out_0          pe_write
```

*Last updated: 2026-05-07*
