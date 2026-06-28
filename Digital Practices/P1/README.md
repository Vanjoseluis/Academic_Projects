# Practice 1: Electronic Voting System
## Overview
This project implements and verifies an electronic voting system for 8 voters, developed as part of the Digital Electronics course at the University of Málaga.
> The general module is composed of 3 main modules: Cuenta_Votos_Completo, Visualiza_Cuenta, Resultado_de_Votación.

> The voting is considered finished when all voters have cast their vote. This is indicated through an additional input (button 0 of the Nexys3).
![State Diagram](Diagrams/Diagrama_transición_de_estados_1B.jpg)

## Repository Contents

| File / Folder | Type | Description |
| --- | --- | --- |
| ``P1_A2-08.pdf`` | Document | Practice 1A: combinational design, schematics, Karnaugh maps, simulations. |
| ``P1_A2-08_VHDL.pdf`` | Document | VHDL implementations of ``Cuenta4_votos`` (logic, IF, CASE) + shared testbench. |
| ``P1B_A2-08.pdf`` | Document | Practice 1B: sequential design, vote register (``Reg_Vot``), state machine analysis. |
| ``P1A`` | Folder | First Part of the Practice, completely combinational |
| ``P2B`` | Folder | Second Part, which includes sequential module |
| ``Diagrams`` | Folder | State Diagram and Test Bench Cronograms |

---

## Technical Summary
### Architecture
The system is divided into three main blocks:
- Cuenta_Votos_Completo — counts votes from 8 voters (two symmetric 4‑voter blocks).
- Visualiza_Cuenta — displays partial results on seven‑segment displays.
- Resultado_de_Votación — determines Approved, Rejected, or No Qualified Majority.

## Vote Counting
Each 4‑voter block uses the Cuenta_4_votos module, derived analytically from truth tables and Karnaugh maps. Outputs S2:S0 encode the number of “yes” votes.

## Qualified Majority (7/8)
- A motion is approved only if 7 or 8 voters vote in favor.
- Rejected if 5–8 votes against.
- Otherwise: No qualified majority.

### Button 0 Behavior
Button 0 on the Nexys3 enables result visualization only while pressed, reflecting a purely combinational design.

## How to Run Simulations
### Environment
- Xilinx ISE
- ISim or compatible VHDL simulator

### Unit Tests
- Cuenta4_votos: exhaustive 16‑combination testbench using staggered toggling processes.
- Sumador_Completo, Restador, Complemento_a2: full enumeration using to_unsigned(i, n).

### System Simulation
- Cuenta_Votos_COMPLETO: iterates through all 256 combinations of m[7:0].
- Check relation: SVC = 8 − SVF.

### FPGA Deployment
- Load top.bit onto Nexys3.
- Validate LEDs and seven‑segment displays.

## VHDL Modules
### Main Modules
- Cuenta4_votosVHDL — logic, IF, and CASE versions.
- Reg_Vot_Exc — sequential logic with D flip‑flops and multiplexers.
- Reg_Vot_CASE — state machine version (Moore model).

### Testbenches
- Shared testbench for all Cuenta4_votos versions.
- Chronogram testbench for Reg_Vot (state evolution).
- Exhaustive testbenches for arithmetic modules.

## Known Issues & Notes
### ISE Static File References
ISE may keep stale references to removed testbenches.
Solution: remove both files, delete the old testbench, re‑add the correct one.

### Button Debounce
Puls_On_Off mitigates bounce using flip‑flops and edge detection, but slow presses may still cause multiple transitions.

### Sequential Logic Considerations
In Reg_Vot_CASE, flip‑flops must update on every rising edge; gating updates with ENA leads to mismatches with hardware behavior.

---

Credits
Authors: Raúl Escudero Jiménez, José Luis Pérez Martín
Course: Digital Electronics
Instructor: Javier López García
University of Málaga — Escuela de Ingenierías Industriales
