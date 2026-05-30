# ServiceSystems2026
# Shuttle Bus Vehicle & Driver Scheduling Solver

This repository contains a heuristic-based optimization solver designed to solve the combined Vehicle and Driver Scheduling Problem (VDSP) for a shuttle-bus service operating back-and-forth between two terminals ($A$ and $B$) with a central depot ($D$).

## Features & Methodology
The solver is designed to be entirely general, data-driven, and free of instance-specific hardcoding. It utilizes the following multi-stage heuristic pipeline:

1. **Multi-Start Core Framework:** Executes the greedy assignment engine across a diverse set of parameterized policies (varying the `max_append_shift_len` constraint between 510 to 660 minutes) to thoroughly sample the solution space.
2. **Phase-Aware Greedy Trip Chaining:** Build feasible driver duties chronologically. It prioritizes same-location service continuations to eliminate deadhead overhead, and falls back to depot-mediated deadheads ($A \rightarrow D \rightarrow B$) only when a duty is on the verge of stalling.
3. **Smart Break Placement:** Instead of blindly pushing safety breaks to the first legal slot, the solver dynamically computes and evaluates idle intervals across the entire constructed route, prioritizing the largest, most natural wait gaps.
4. **Duty Post-Processor (Absorber):** Iteratively attempts to eliminate sub-optimal or sparse driver duties by safely dispersing and absorbing their service trips into other existing feasible chains.
5. **Physical Vehicle Reuse:** Consolidates non-overlapping duties onto the minimum required fleet size using an explicit scheduling matrix.

## File Structure
* `solver.py` — The core algorithmic solver and greedy framework.
* `check_solution.py` — The official academic feasibility and cost validation engine.
* `*.json` — Benchmark inputs (`small_01`, `small_02`, `medium_01`) and schema configurations.

## Setup & Requirements
No external packages or MILP licensing (e.g., Gurobi/CPLEX) are required. Built entirely using standard Python 3 libraries.

Ensure Python is available in your environment path and run via terminal.

## Execution Guide

### Running the Solver
To execute the solver on a specific instance and generate a compatible JSON solution file, use:
```bash
python solver.py small_01.json solution_small_01.json
