#!/usr/bin/env python3
"""
Greedy solver for ServiceSystems2026 shuttle bus vehicle & driver scheduling.

Usage:
    python solver.py instance.json solution.json

The solver is intentionally general and heuristic-based:
1. Read the instance JSON.
2. Build feasible driver duties greedily from chronological trips.
3. Insert a legal 1-hour break inside an idle interval.
4. Reuse physical vehicles across non-overlapping duties.
5. Write a solution JSON that matches solution_schema.json.

No external dependencies are required.
"""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

Location = str
Trip = Dict[str, object]
Activity = Dict[str, object]


class TravelTable:
    LEG_ORDER = ["A-B", "B-A", "D-A", "A-D", "D-B", "B-D"]

    def __init__(self, spec: dict):
        self.legs = spec["legs"]
        if self.legs != self.LEG_ORDER:
            raise ValueError(f"travel_time.legs must be {self.LEG_ORDER}, got {self.legs}")
        self.buckets = sorted(spec["buckets"], key=lambda b: b["from_min"])

    def lookup(self, frm: Location, to: Location, start_min: int) -> int:
        key = f"{frm}-{to}"
        if key not in self.legs:
            raise ValueError(f"Leg {key} is not tabulated")
        idx = self.legs.index(key)
        for b in self.buckets:
            if b["from_min"] <= start_min < b["to_min"]:
                return int(b["minutes"][idx])
        raise ValueError(f"Departure time {start_min} outside travel-time table")


def load_json(path: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_json(data: dict, path: str) -> None:
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def floor_q(t: int) -> int:
    return (t // 15) * 15


def ceil_q(t: int) -> int:
    return ((t + 14) // 15) * 15


def service_end(trip: Trip, travel: TravelTable) -> int:
    dep = int(trip["departure_min"])
    return dep + travel.lookup(str(trip["origin"]), str(trip["destination"]), dep)


def service_activity(trip: Trip, travel: TravelTable) -> Activity:
    dep = int(trip["departure_min"])
    return {
        "type": "service",
        "trip_id": int(trip["trip_id"]),
        "start_min": dep,
        "end_min": service_end(trip, travel),
    }


def deadhead(frm: Location, to: Location, start: int, travel: TravelTable) -> Activity:
    return {
        "type": "deadhead",
        "from": frm,
        "to": to,
        "start_min": start,
        "end_min": start + travel.lookup(frm, to, start),
    }


def wait(at: Location, start: int, end: int) -> Optional[Activity]:
    if end <= start:
        return None
    return {"type": "wait", "at": at, "start_min": start, "end_min": end}


def brk(at: Location, start: int, end: int) -> Activity:
    return {"type": "break", "at": at, "start_min": start, "end_min": end}


def latest_start_arrive_by(travel: TravelTable, frm: Location, to: Location, latest_arrival: int) -> Optional[int]:
    """Latest quarter-hour start from frm to to that arrives no later than latest_arrival."""
    best = None
    for s in range(0, latest_arrival + 1, 15):
        try:
            if s + travel.lookup(frm, to, s) <= latest_arrival:
                best = s
        except ValueError:
            pass
    return best


def exact_deadhead_start_to_end(
    travel: TravelTable,
    frm: Location,
    to: Location,
    earliest_start: int,
    desired_end: int,
) -> Optional[int]:
    """Find a start s >= earliest_start such that s + travel(frm,to,s) == desired_end."""
    for s in range(earliest_start, desired_end + 1):
        try:
            if s + travel.lookup(frm, to, s) == desired_end:
                return s
        except ValueError:
            continue
    return None


def route_is_time_compatible(route: List[Trip], travel: TravelTable) -> bool:
    if not route:
        return False
    current_loc = str(route[0]["origin"])
    current_time = int(route[0]["departure_min"])
    for trip in route:
        origin = str(trip["origin"])
        dep = int(trip["departure_min"])
        if dep < current_time:
            return False
        if origin != current_loc:
            # Through-depot deadhead required: current_loc → D → origin
            dh1 = travel.lookup(current_loc, "D", current_time)
            t_depot = current_time + dh1
            dh2 = travel.lookup("D", origin, t_depot)
            if t_depot + dh2 > dep:
                return False
        current_time = service_end(trip, travel)
        current_loc = str(trip["destination"])
    return True


def find_break_slot(
    route: List[Trip],
    travel: TravelTable,
    shift_start: int,
    shift_end: int,
    first_arrival: int,
    return_start: int,
    params: dict,
) -> Optional[Tuple[int, int, Location]]:
    break_len = int(params["break_length_hours"] * 60)
    alpha = int(params["break_min_from_start_hours"] * 60)
    beta = int(params["break_min_from_end_hours"] * 60)
    legal_start = shift_start + alpha
    legal_end = shift_end - beta

    intervals: List[Tuple[int, int, Location]] = []

    # before first service, at first origin
    intervals.append((first_arrival, int(route[0]["departure_min"]), str(route[0]["origin"])))

    # between services (accounting for through-depot deadheads)
    for prev, nxt in zip(route, route[1:]):
        prev_dest = str(prev["destination"])
        nxt_orig  = str(nxt["origin"])
        t_after   = service_end(prev, travel)
        t_before  = int(nxt["departure_min"])
        if prev_dest == nxt_orig:
            intervals.append((t_after, t_before, prev_dest))
        else:
            # Vehicle is moving through depot; idle time only available at destination
            dh1 = travel.lookup(prev_dest, "D", t_after)
            t_depot = t_after + dh1
            dh2 = travel.lookup("D", nxt_orig, t_depot)
            t_arrive = t_depot + dh2
            if t_arrive < t_before:
                intervals.append((t_arrive, t_before, nxt_orig))

    # after last service, before returning to depot
    intervals.append((service_end(route[-1], travel), return_start, str(route[-1]["destination"])))

    # Prefer the break in the largest idle gap — maximises natural rest placement
    # and leaves tighter gaps available for service chaining.
    valid: List[Tuple[int, int, int, Location]] = []  # (gap_size, b0, b1, loc)
    for s, e, loc in intervals:
        b0 = max(s, legal_start)
        b1 = b0 + break_len
        if b1 <= e and b1 <= legal_end:
            valid.append((e - s, b0, b1, loc))

    if not valid:
        return None
    valid.sort(reverse=True)   # largest gap first
    _, b0, b1, loc = valid[0]
    return b0, b1, loc


def construct_duty_for_route(
    route: List[Trip],
    duty_idx: int,
    params: dict,
    travel: TravelTable,
) -> Optional[dict]:
    """Return a feasible duty for this fixed service route, or None."""
    if not route_is_time_compatible(route, travel):
        return None

    min_shift = int(params["shift_min_hours"] * 60)
    max_shift = int(params["shift_max_hours"] * 60)
    first = route[0]
    last = route[-1]
    first_origin = str(first["origin"])
    first_dep = int(first["departure_min"])
    last_dest = str(last["destination"])
    last_end = service_end(last, travel)

    # Candidate starts: prefer the latest possible start to reduce terminal dwell.
    starts = []
    latest = latest_start_arrive_by(travel, "D", first_origin, first_dep)
    if latest is None:
        return None
    earliest = max(0, latest - (max_shift - min_shift) - 240)
    for s0 in range(latest, earliest - 1, -15):
        try:
            arr = s0 + travel.lookup("D", first_origin, s0)
        except ValueError:
            continue
        if arr <= first_dep:
            starts.append((s0, arr))

    # Try late starts first, and among them shortest shifts first.
    for shift_start, first_arrival in starts:
        for shift_len in range(min_shift, max_shift + 1, 15):
            shift_end = shift_start + shift_len
            if shift_end < last_end:
                continue
            return_start = exact_deadhead_start_to_end(
                travel, last_dest, "D", last_end, shift_end
            )
            if return_start is None:
                continue

            slot = find_break_slot(route, travel, shift_start, shift_end, first_arrival, return_start, params)
            if slot is None:
                continue
            break_start, break_end, break_loc = slot

            acts: List[Activity] = []
            first_dh = deadhead("D", first_origin, shift_start, travel)
            acts.append(first_dh)
            current_time = int(first_dh["end_min"])
            current_loc = first_origin

            break_inserted = False

            def add_wait_or_break_until(target_time: int, loc: Location) -> None:
                nonlocal current_time, break_inserted
                if (not break_inserted) and loc == break_loc and current_time <= break_start and break_end <= target_time:
                    w1 = wait(loc, current_time, break_start)
                    if w1:
                        acts.append(w1)
                    acts.append(brk(loc, break_start, break_end))
                    break_inserted = True
                    current_time = break_end
                    w2 = wait(loc, current_time, target_time)
                    if w2:
                        acts.append(w2)
                    current_time = target_time
                else:
                    w = wait(loc, current_time, target_time)
                    if w:
                        acts.append(w)
                    current_time = target_time

            for trip in route:
                dep = int(trip["departure_min"])
                origin = str(trip["origin"])
                if origin != current_loc:
                    # Insert through-depot deadhead: current_loc → D → origin
                    dh1_act = deadhead(current_loc, "D", current_time, travel)
                    acts.append(dh1_act)
                    current_time = int(dh1_act["end_min"])
                    current_loc = "D"
                    dh2_act = deadhead("D", origin, current_time, travel)
                    acts.append(dh2_act)
                    current_time = int(dh2_act["end_min"])
                    current_loc = origin
                if current_time > dep:
                    return None
                add_wait_or_break_until(dep, current_loc)
                svc = service_activity(trip, travel)
                acts.append(svc)
                current_time = int(svc["end_min"])
                current_loc = str(trip["destination"])

            # gap before final deadhead
            add_wait_or_break_until(return_start, current_loc)
            if not break_inserted:
                return None
            acts.append(deadhead(current_loc, "D", return_start, travel))

            return {
                "duty_id": f"k{duty_idx}",
                "driver_id": f"d{duty_idx}",
                "vehicle_id": "TO_ASSIGN",
                "shift_start_min": shift_start,
                "shift_end_min": shift_end,
                "break_start_min": break_start,
                "break_end_min": break_end,
                "break_location": break_loc,
                "activities": acts,
            }

    return None


def build_greedy_duties(
    trips: List[Trip],
    params: dict,
    travel: TravelTable,
    max_append_shift_len: int = 570,
) -> List[dict]:
    unassigned = {int(t["trip_id"]): t for t in trips}
    ordered = sorted(trips, key=lambda t: (int(t["departure_min"]), int(t["trip_id"])))
    duties: List[dict] = []

    while unassigned:
        # start from earliest still-unassigned trip
        start_trip = next(t for t in ordered if int(t["trip_id"]) in unassigned)
        route = [start_trip]
        assert construct_duty_for_route(route, len(duties) + 1, params, travel) is not None

        # greedily append compatible trips using two-phase candidate expansion
        improved = True
        while improved:
            improved = False
            current_loc = str(route[-1]["destination"])
            current_time = service_end(route[-1], travel)

            # Phase 1: same-location candidates (earliest departure = minimum idle).
            same_loc = [
                t for t in ordered
                if int(t["trip_id"]) in unassigned
                and str(t["origin"]) == current_loc
                and int(t["departure_min"]) >= current_time
            ]
            same_loc.sort(key=lambda t: (int(t["departure_min"]), int(t["trip_id"])))

            for cand in same_loc[:12]:
                candidate_duty = construct_duty_for_route(route + [cand], len(duties) + 1, params, travel)
                if candidate_duty is not None and (
                    candidate_duty["shift_end_min"] - candidate_duty["shift_start_min"] <= max_append_shift_len
                ):
                    route = route + [cand]
                    improved = True
                    break

            if improved:
                continue

            # Phase 2: through-depot candidates (only when Phase 1 found nothing).
            # Generic: any origin that is not current_loc and not the depot.
            cross: List[Tuple[int, Trip]] = []
            for t in ordered:
                if int(t["trip_id"]) not in unassigned:
                    continue
                origin = str(t["origin"])
                if origin == current_loc or origin == "D":
                    continue
                dep = int(t["departure_min"])
                try:
                    dh1 = travel.lookup(current_loc, "D", current_time)
                    t_depot = current_time + dh1
                    dh2 = travel.lookup("D", origin, t_depot)
                    t_arrive = t_depot + dh2
                except ValueError:
                    continue
                if t_arrive <= dep:
                    slack = dep - t_arrive   # idle at destination after deadhead
                    cross.append((slack, t))
            cross.sort(key=lambda x: (x[0], int(x[1]["trip_id"])))

            for _, cand in cross[:8]:
                candidate_duty = construct_duty_for_route(route + [cand], len(duties) + 1, params, travel)
                if candidate_duty is not None and (
                    candidate_duty["shift_end_min"] - candidate_duty["shift_start_min"] <= max_append_shift_len
                ):
                    route = route + [cand]
                    improved = True
                    break

        duty = construct_duty_for_route(route, len(duties) + 1, params, travel)
        if duty is None:
            raise RuntimeError("Internal error: route became infeasible")
        duties.append(duty)
        for trip in route:
            del unassigned[int(trip["trip_id"])]

    return duties


def assign_vehicle_ids(duties: List[dict]) -> None:
    """Reuse vehicles across non-overlapping duties to reduce fixed vehicle cost."""
    vehicles: List[Tuple[str, int]] = []  # (vehicle_id, available_from)
    for duty in sorted(duties, key=lambda d: (d["shift_start_min"], d["shift_end_min"])):
        assigned = None
        for i, (vid, available) in enumerate(vehicles):
            if available <= duty["shift_start_min"]:
                assigned = i
                break
        if assigned is None:
            vid = f"v{len(vehicles) + 1}"
            vehicles.append((vid, duty["shift_end_min"]))
            duty["vehicle_id"] = vid
        else:
            vid, _ = vehicles[assigned]
            duty["vehicle_id"] = vid
            vehicles[assigned] = (vid, duty["shift_end_min"])



def try_insert_trip_into_route(
    existing_trip_ids: List[int],
    new_trip: Trip,
    trips_by_id: dict,
    duty_idx: int,
    params: dict,
    travel: TravelTable,
) -> Optional[dict]:
    """Try inserting new_trip at every position in the route. Return first feasible duty."""
    for i in range(len(existing_trip_ids) + 1):
        route = (
            [trips_by_id[tid] for tid in existing_trip_ids[:i]]
            + [new_trip]
            + [trips_by_id[tid] for tid in existing_trip_ids[i:]]
        )
        result = construct_duty_for_route(route, duty_idx, params, travel)
        if result is not None:
            return result
    return None


def eliminate_small_duties(
    duties: List[dict],
    trips_by_id: dict,
    params: dict,
    travel: TravelTable,
) -> List[dict]:
    """Post-processing: eliminate duties with few service trips by redistributing them."""
    improved = True
    while improved:
        improved = False
        by_size = sorted(
            range(len(duties)),
            key=lambda i: sum(1 for a in duties[i]["activities"] if a["type"] == "service"),
        )
        for small_idx in by_size:
            small = duties[small_idx]
            small_tids = [
                int(a["trip_id"]) for a in small["activities"] if a["type"] == "service"
            ]
            others = [d for i, d in enumerate(duties) if i != small_idx]
            new_others = list(others)
            all_inserted = True
            for tid in small_tids:
                new_trip = trips_by_id[tid]
                inserted = False
                for j, other in enumerate(new_others):
                    other_tids = [
                        int(a["trip_id"]) for a in other["activities"] if a["type"] == "service"
                    ]
                    new_duty = try_insert_trip_into_route(
                        other_tids, new_trip, trips_by_id,
                        int(other["duty_id"][1:]), params, travel,
                    )
                    if new_duty is not None:
                        new_others[j] = new_duty
                        inserted = True
                        break
                if not inserted:
                    all_inserted = False
                    break
            if all_inserted:
                duties = new_others
                improved = True
                break
    return duties


def solution_cost(solution: dict, instance: dict) -> float:
    """Compute the same objective as the checker for comparing our own variants."""
    params = instance["parameters"]
    vehicles = {d["vehicle_id"] for d in solution["duties"]}
    deadhead_min = 0
    driver_cost = 0.0
    for d in solution["duties"]:
        for a in d["activities"]:
            if a["type"] == "deadhead":
                deadhead_min += int(a["end_min"]) - int(a["start_min"])
        length_h = (int(d["shift_end_min"]) - int(d["shift_start_min"])) / 60.0
        driver_cost += 8.0 * params["cost_driver_regular_per_h"] + max(0.0, length_h - 8.0) * params["cost_driver_overtime_per_h"]
    return (
        params["cost_fixed_vehicle"] * len(vehicles)
        + params["cost_variable_per_min"] * deadhead_min
        + driver_cost
    )

def solve(instance: dict, instance_id: str) -> dict:
    params = instance["parameters"]
    travel = TravelTable(instance["travel_time"])
    trips = sorted(instance["trips"], key=lambda t: (int(t["departure_min"]), int(t["trip_id"])))
    trips_by_id = {int(t["trip_id"]): t for t in trips}

    policies = [510, 540, 570, 600, 630, 660]
    best_solution = None
    best_cost = float("inf")
    deadline = time.time() + 270  # grader hard-kills at 300 s; 30 s margin

    for max_len in policies:
        if time.time() > deadline:
            break

        duties = build_greedy_duties(trips, params, travel, max_append_shift_len=max_len)

        # Run post-processing only when budget allows; otherwise use the greedy result.
        if time.time() <= deadline:
            duties = eliminate_small_duties(duties, trips_by_id, params, travel)

        assign_vehicle_ids(duties)
        duties.sort(key=lambda d: int(d["duty_id"][1:]))
        sol = {"instance_id": instance_id, "duties": duties}
        cost = solution_cost(sol, instance)
        if cost < best_cost:
            best_cost = cost
            best_solution = sol

    assert best_solution is not None
    return best_solution


def main(argv: List[str]) -> int:
    if len(argv) != 3:
        print("Usage: python solver.py instance.json solution.json", file=sys.stderr)
        return 2
    instance_path, output_path = argv[1], argv[2]
    instance = load_json(instance_path)
    instance_id = Path(instance_path).stem
    solution = solve(instance, instance_id)
    save_json(solution, output_path)
    print(f"Wrote {output_path} with {len(solution['duties'])} duties")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
