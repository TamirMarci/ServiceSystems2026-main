# CLAUDE.md — ServiceSystems2026 Solver Guide

## Project Context

This repository contains a university term project for Service Systems 2026.

The task is to solve a shuttle bus vehicle-and-driver scheduling problem.

The goal is to build a practical Python solver that produces feasible and reasonably good solutions. The solution does not need to be optimal, but it must be robust, general, and pass the official checker.

Current public instances:

* `small_01.json`
* `small_02.json`
* `medium_01.json`

Reference solution:

* `sol_small_01_suboptimal.json`

Future instances are expected, including larger inputs such as `medium_02` and `large_01`. The solver should therefore avoid overfitting and should run in reasonable time.

---

## Source of Truth

Before making changes, read these files directly:

1. `assignment.pdf`
   Project requirements, rules, objective, input/output format, deadlines, and grading.

2. `check_solution.py`
   Official feasibility and cost checker. This is the most important file for validation.

3. `solution_schema.json`
   Required output JSON structure. Generated solutions must conform exactly to this schema.

4. `sol_small_01_suboptimal.json`
   Published feasible reference solution for `small_01`. Use it as a format example and cost benchmark. Do not copy or hard-code it.

5. `small_01.json`, `small_02.json`, `medium_01.json`
   Current public test instances.

If there is any ambiguity between files, prioritize:

1. Passing `check_solution.py`
2. Conforming to `solution_schema.json`
3. Following `assignment.pdf`

---

## Main Goal

Implement and improve `solver.py`.

The solver must run as:

```bash
python solver.py instance.json solution.json
```

Examples:

```bash
python solver.py small_01.json solution_small_01.json
python check_solution.py small_01.json solution_small_01.json
```

The generated solution files must pass the checker on all current public instances.

---

## Required Output Files

The solver should generate:

* `solution_small_01.json`
* `solution_small_02.json`
* `solution_medium_01.json`

The output JSON must follow `solution_schema.json` exactly.

Do not add extra fields.

---

## Development Strategy

Work incrementally.

Do not attempt to solve everything at once.

Recommended order:

1. Make `small_01` pass.
2. Then make `small_02` pass.
3. Then make `medium_01` pass.
4. Only after all pass, improve solution quality.
5. Then update `README.md`.

Feasibility comes before optimization.

A robust feasible solution is better than a fragile optimized solution.

---

## Algorithmic Direction

Use a practical heuristic approach.

Do not use a full MILP or exhaustive enumeration unless it remains fast and simple.

Recommended heuristic:

1. Read and sort trips by departure time.
2. Build duties by chaining compatible service trips.
3. A trip can be appended to a duty if:

   * the vehicle is at the trip origin,
   * there is enough time before departure,
   * the duty can still include a legal break,
   * the duty can still return to depot,
   * shift length remains legal,
   * terminal capacity is not violated.
4. Insert one legal break into a natural waiting gap when possible.
5. If no valid break is possible, split the chain into shorter duties.
6. After duties are built, reuse physical vehicles across non-overlapping duties.
7. Use fallback duties for uncovered trips if needed.

The solver should not simply create one duty per trip unless used only as a fallback.

---

## Important Feasibility Rules

Always validate against `check_solution.py`.

Common failure points:

* Every service trip must be covered exactly once.
* Service activity start time must equal the trip departure time.
* Service duration must match the time-dependent travel table.
* Deadhead duration must match the time-dependent travel table.
* Activities must be time-contiguous.
* Activities must be location-contiguous.
* Each duty must start with a deadhead from depot `D`.
* Each duty must end with a deadhead to depot `D`.
* Each duty must include exactly one break.
* Break fields at duty level must match the break activity.
* Shift start must be on a 15-minute boundary.
* Shift duration must be between the minimum and maximum allowed shift length.
* Vehicle duties must not overlap.
* Terminal capacity must not be violated.
* Long unnecessary waits at terminals should be avoided when possible.

---

## Vehicle Reuse

After building all duties, assign vehicle IDs using greedy interval coloring:

1. Sort duties by `shift_start_min`.
2. Reuse an existing vehicle if its previous duty ended before the new duty starts.
3. Otherwise create a new vehicle.

This reduces fixed vehicle cost without changing the duty activities.

Do not assign the same vehicle to overlapping duties.

---

## Reference Solution Analysis

Use `sol_small_01_suboptimal.json` as a benchmark.

Run:

```bash
python check_solution.py small_01.json sol_small_01_suboptimal.json
python check_solution.py small_01.json solution_small_01.json
```

Compare:

* total cost
* vehicles used
* deadhead minutes
* driver cost
* number of duties

The reference solution is feasible but suboptimal. Do not copy it.

Try to learn its structure:

* It chains multiple trips per duty.
* It inserts breaks into natural waiting gaps.
* It avoids unnecessary depot returns.
* It reuses vehicles across non-overlapping duties.

---

## Runtime and Scalability

The final project will include larger instances.

Avoid:

* hard-coded trip IDs,
* hard-coded schedules,
* instance-specific hacks,
* exhaustive enumeration of all possible duties,
* algorithms that scale exponentially.

Aim for predictable runtime.

The future large instance should run within the assignment’s 5-minute wall-clock limit.

---

## README.md Requirements

Update `README.md` after the solver works.

Keep it short and practical.

Include:

1. Project overview
2. Solver approach
3. How to run
4. How to validate
5. Tested instances
6. Comparison against `sol_small_01_suboptimal.json`
7. Scalability notes
8. Dependencies
9. AI usage statement

Suggested AI usage statement:

```text
AI assistance was used for coding/debugging support; the final algorithmic choices and validation were reviewed by the team.
```

---

## Final Validation Checklist

Before finishing, run:

```bash
python solver.py small_01.json solution_small_01.json
python check_solution.py small_01.json solution_small_01.json

python solver.py small_02.json solution_small_02.json
python check_solution.py small_02.json solution_small_02.json

python solver.py medium_01.json solution_medium_01.json
python check_solution.py medium_01.json solution_medium_01.json

python check_solution.py small_01.json sol_small_01_suboptimal.json
```

All generated solutions must pass.

Then report a concise summary table:

```text
instance | feasible | total_cost | vehicles_used | duties | deadhead_minutes | runtime_seconds
```

Do not stop after writing code. Validate, fix checker violations, and only then update the README.
