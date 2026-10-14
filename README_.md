# Parameterized Direct-Mapped Cache Controller (Verilog)

A fully parameterized, synthesizable **direct-mapped cache controller** written in plain
Verilog (no SystemVerilog), with a self-checking testbench.

## Features
- Direct-mapped, **write-back / write-allocate**
- Parameters: address width, data width, words per line, number of lines
- Byte-enable (write strobe) support
- Single-cycle hit; FSM-based miss handling (`IDLE -> [WRITEBACK] -> REFILL`)
- Hit / miss counters
- Self-checking testbench (directed + 5000 random ops vs. reference model)

## Parameters
| Parameter | Default | Meaning |
|---|---|---|
| `ADDR_WIDTH` | 32 | Byte-address width |
| `DATA_WIDTH` | 32 | Word width in bits (multiple of 8, power of 2) |
| `WORDS_PER_LINE` | 4 | Words per cache line (power of 2, >= 2) |
| `NUM_LINES` | 64 | Cache lines (power of 2, >= 2) |

Address layout: `| TAG | INDEX | WORD_OFFSET | BYTE_OFFSET |`

Cache size in bytes = `NUM_LINES * WORDS_PER_LINE * DATA_WIDTH/8`.

## Interfaces
**CPU side:** `cpu_req, cpu_we, cpu_addr, cpu_wdata, cpu_wstrb -> cpu_rdata, cpu_ready`
Hold the request until `cpu_ready` pulses (1 cycle). On hits, ready is asserted in the same cycle.

**Memory side:** `mem_req, mem_we, mem_addr, mem_wdata <- mem_rdata, mem_ready`
Word-granular handshake. Signals are held until `mem_ready`.

## Run the simulation
```
make sim        # requires Icarus Verilog
```
Expected output: `PASS: all checks passed`

Verified configurations (D = data width, W = words/line, N = lines): 32/4/16, 64/8/32, 8/2/8, 64/2/4, 16/16/2.

## Repository layout
```
rtl/direct_mapped_cache.v      # design
tb/tb_direct_mapped_cache.v    # self-checking testbench
Makefile
```

## Possible extensions
Set-associative variant, burst memory interface, flush/invalidate command, AXI wrapper.
