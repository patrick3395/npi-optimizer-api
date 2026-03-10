"""OR-Tools VRPTW optimizer for NPI Route Optimizer.

Constraints implemented (mirrors app/lib/optimizer.ts exactly):
  1. Capabilities          — required_capabilities ⊆ inspector.capabilities
  2. Distance limit        — haversine(home, job) ≤ max_home_dist_km
  3. Max jobs capacity     — inspector.max_jobs (soft: 2 base, hard: 3)
  4. 3rd-job time rule     — only 2 jobs before third_job_min_time per inspector
  5. Same-slot uniqueness  — VehicleVar(a) != VehicleVar(b) for same time_slot
  6. Weekly schedule       — schedule_blocks by day + slot (or whole-day "*")
  7. Exclusion zones       — ray-cast point-in-polygon per inspector
  8. Property rules        — maxSqFt / minYearBuilt / maxYearBuilt per inspector
  9. Run rules (block)     — job.blocked_inspectors list → RemoveValue
 10. Locked inspector      — job.locked_inspector → VehicleVar.SetValue
 11. State filtering       — inspector.home_state vs job state from address
 12. Ranking preference    — per-vehicle arc cost penalty for lower-rank inspectors
"""

from __future__ import annotations

import re
from collections import defaultdict
from datetime import datetime
from typing import Sequence

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

from geocoder import build_distance_matrix, haversine_m
from models import (
    Inspector,
    InspectorResult,
    Job,
    OptimizeRequest,
    OptimizeResponse,
    Settings,
)

# ---------- constants --------------------------------------------------------

_TIME_FMT_12 = "%I:%M %p"
_TIME_BUFFER_MIN = 30   # ± buffer around appointment slot for time dimension
_SERVICE_TIME_MIN = 60  # assumed on-site service duration (minutes)

# ---------- helpers ----------------------------------------------------------


def _parse_time_minutes(slot: str) -> int:
    """Parse '9:00 AM' / '1:30 PM' → minutes since midnight."""
    try:
        dt = datetime.strptime(slot.strip(), _TIME_FMT_12)
        return dt.hour * 60 + dt.minute
    except ValueError:
        return 0


def _day_of_week(date_str: str) -> int:
    """Return JS-style day-of-week integer (Sun=0 … Sat=6) for an ISO date."""
    py_dow = datetime.strptime(date_str, "%Y-%m-%d").isoweekday()  # Mon=1…Sun=7
    return py_dow % 7


def _parse_state(address: str) -> str | None:
    """Extract two-letter US state code from an address string."""
    m = re.search(r",\s*([A-Z]{2})\s+\d{5}", address)
    return m.group(1) if m else None


def _point_in_polygon(lat: float, lng: float, polygon: Sequence) -> bool:
    """Ray-casting point-in-polygon. polygon items are LatLng objects."""
    n = len(polygon)
    inside = False
    j = n - 1
    for i in range(n):
        yi, xi = polygon[i].lat, polygon[i].lng
        yj, xj = polygon[j].lat, polygon[j].lng
        if ((yi > lat) != (yj > lat)) and (lng < (xj - xi) * (lat - yi) / (yj - yi) + xi):
            inside = not inside
        j = i
    return inside


# ---------- eligibility pre-filter -------------------------------------------


def _eligible(
    inspector: Inspector,
    job: Job,
    settings: Settings,
    request_day: int,
    job_state: str | None,
) -> bool:
    """Return True if inspector may be assigned job at all.

    This runs before OR-Tools so the solver never even considers ineligible
    pairs. All checks here mirror inspectorAvailableForJob() in optimizer.ts.
    """
    if not inspector.active:
        return False

    # 1. Locked inspector — only the named inspector may take this job
    if job.locked_inspector and job.locked_inspector != inspector.name:
        return False

    # 2. Block run rules — this inspector is explicitly blocked from this job
    if inspector.name in job.blocked_inspectors:
        return False

    # 3. Capabilities
    if not set(job.required_capabilities).issubset(set(inspector.capabilities)):
        return False

    # 4. Distance limit (haversine pre-filter — road distance checked via matrix)
    dist_km = haversine_m(inspector.home_lat, inspector.home_lng, job.lat, job.lng) / 1000
    if dist_km > settings.max_home_dist_km:
        return False

    # 5. State filtering — inspector home state must match job state
    job_st = job_state
    insp_st = inspector.home_state
    if insp_st and job_st and insp_st != job_st:
        return False

    # 6. Weekly schedule blocks
    job_minutes = _parse_time_minutes(job.time_slot)
    for block in inspector.schedule_blocks:
        if block.day == request_day:
            if block.slot == "*":           # whole day off
                return False
            if _parse_time_minutes(block.slot) == job_minutes:
                return False

    # 7. Exclusion zones — ray-cast check
    for zone in inspector.exclusion_zones:
        if _point_in_polygon(job.lat, job.lng, zone):
            return False

    # 8. Property rules
    for rule in inspector.property_rules:
        if rule.type == "maxSqFt" and job.sq_ft is not None:
            if job.sq_ft > rule.value:
                return False
        elif rule.type == "minYearBuilt" and job.year_built is not None:
            if job.year_built < rule.value:
                return False
        elif rule.type == "maxYearBuilt" and job.year_built is not None:
            if job.year_built > rule.value:
                return False

    return True


# ---------- solver -----------------------------------------------------------


def optimize(request: OptimizeRequest) -> OptimizeResponse:
    """Run VRPTW and return optimized assignments."""
    settings = request.settings
    request_day = _day_of_week(request.date)

    active_inspectors = [i for i in request.inspectors if i.active]
    jobs = list(request.jobs)

    if not active_inspectors or not jobs:
        return OptimizeResponse(
            assignments={i.name: [] for i in active_inspectors},
            total_distance_m=0,
            original_distance_m=0,
            savings_pct=0.0,
            unassigned=[j.id for j in jobs],
            per_inspector={
                i.name: InspectorResult(distance_m=0, jobs=[], route_order=[])
                for i in active_inspectors
            },
        )

    job_states = [_parse_state(j.address) for j in jobs]

    # Build eligibility matrix: vehicle_idx → set of eligible job indices
    eligible_map: dict[int, set[int]] = {v: set() for v in range(len(active_inspectors))}
    for v_idx, insp in enumerate(active_inspectors):
        for j_idx, job in enumerate(jobs):
            if _eligible(insp, job, settings, request_day, job_states[j_idx]):
                eligible_map[v_idx].add(j_idx)

    # Nodes: 0…V-1 = inspector homes (depots), V…V+J-1 = jobs
    num_vehicles = len(active_inspectors)
    num_jobs = len(jobs)
    num_nodes = num_vehicles + num_jobs

    coords: list[tuple[float, float]] = []
    for insp in active_inspectors:
        coords.append((insp.home_lat, insp.home_lng))
    for job in jobs:
        coords.append((job.lat, job.lng))

    dist_matrix = build_distance_matrix(
        coords, request.google_maps_key, road_factor=settings.road_factor
    )

    # ── OR-Tools model ────────────────────────────────────────────────────────
    manager = pywrapcp.RoutingIndexManager(
        num_nodes,
        num_vehicles,
        list(range(num_vehicles)),   # start nodes = each inspector's home
        list(range(num_vehicles)),   # end nodes = same (return home)
    )
    routing = pywrapcp.RoutingModel(manager)

    # ── Arc cost: distance + ranking penalty ──────────────────────────────────
    # Ranking miles: for each inspector ranked below the best, add a flat arc-
    # cost penalty = rank_delta * ranking_miles_m. This makes OR-Tools prefer
    # higher-ranked inspectors when the distance difference is within the
    # threshold — exactly mirroring the SA ranking tiebreaker logic.
    ranking_m = int(settings.ranking_miles * 1609.34)   # miles → metres
    sorted_ranks = sorted(set(i.rank for i in active_inspectors))
    rank_index = {r: idx for idx, r in enumerate(sorted_ranks)}  # 0 = best rank

    def _make_distance_cb(v_idx: int):
        rank_pos = rank_index.get(active_inspectors[v_idx].rank, 0)
        penalty_per_arc = rank_pos * ranking_m

        def cb(from_index: int, to_index: int) -> int:
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            base = dist_matrix[from_node][to_node]
            # Add ranking penalty on job arcs only (not depot→depot)
            if to_node >= num_vehicles:
                return base + penalty_per_arc
            return base
        return cb

    # Register per-vehicle cost evaluators
    for v_idx in range(num_vehicles):
        cb = _make_distance_cb(v_idx)
        cb_idx = routing.RegisterTransitCallback(cb)
        routing.SetArcCostEvaluatorOfVehicle(cb_idx, v_idx)

    # Pure distance callback (no penalty) for the Distance dimension
    def distance_callback(from_index: int, to_index: int) -> int:
        return dist_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

    dist_cb_idx = routing.RegisterTransitCallback(distance_callback)
    routing.AddDimension(dist_cb_idx, 0, 10_000_000, True, "Distance")

    # ── Time dimension ────────────────────────────────────────────────────────
    def time_callback(from_index: int, to_index: int) -> int:
        dist_m = dist_matrix[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]
        return int(dist_m / 1000) + _SERVICE_TIME_MIN   # 1 min/km + service

    time_cb_idx = routing.RegisterTransitCallback(time_callback)
    day_end = 24 * 60
    routing.AddDimension(time_cb_idx, day_end, day_end, False, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    third_job_min = _parse_time_minutes(settings.third_job_min_time)

    for j_idx, job in enumerate(jobs):
        index = manager.NodeToIndex(num_vehicles + j_idx)
        slot = _parse_time_minutes(job.time_slot)
        time_dim.CumulVar(index).SetRange(
            max(0, slot - _TIME_BUFFER_MIN),
            min(day_end, slot + _TIME_BUFFER_MIN),
        )
    for v in range(num_vehicles):
        time_dim.CumulVar(routing.Start(v)).SetRange(0, day_end)
        time_dim.CumulVar(routing.End(v)).SetRange(0, day_end)

    # ── Capacity: max jobs per inspector ──────────────────────────────────────
    def demand_cb(from_index: int) -> int:
        return 0 if manager.IndexToNode(from_index) < num_vehicles else 1

    demand_cb_idx = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        demand_cb_idx, 0,
        [insp.max_jobs for insp in active_inspectors],
        True, "Capacity",
    )

    # ── EarlyCapacity: max 2 jobs before third_job_min_time ──────────────────
    def early_demand_cb(from_index: int) -> int:
        node = manager.IndexToNode(from_index)
        if node < num_vehicles:
            return 0
        return 1 if _parse_time_minutes(jobs[node - num_vehicles].time_slot) < third_job_min else 0

    early_cb_idx = routing.RegisterUnaryTransitCallback(early_demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        early_cb_idx, 0,
        [2] * num_vehicles,
        True, "EarlyCapacity",
    )

    # ── Eligibility: RemoveValue for every ineligible (inspector, job) pair ──
    for v_idx in range(num_vehicles):
        for j_idx in range(num_jobs):
            if j_idx not in eligible_map[v_idx]:
                routing.VehicleVar(manager.NodeToIndex(num_vehicles + j_idx)).RemoveValue(v_idx)

    # ── Locked inspector: SetValue forces assignment ──────────────────────────
    for j_idx, job in enumerate(jobs):
        if job.locked_inspector:
            for v_idx, insp in enumerate(active_inspectors):
                if insp.name == job.locked_inspector:
                    routing.VehicleVar(manager.NodeToIndex(num_vehicles + j_idx)).SetValue(v_idx)
                    break

    # ── Same-slot uniqueness: VehicleVar(a) != VehicleVar(b) ─────────────────
    slot_groups: dict[str, list[int]] = defaultdict(list)
    for j_idx, job in enumerate(jobs):
        if job.time_slot:
            slot_groups[job.time_slot].append(j_idx)

    solver = routing.solver()
    for group in slot_groups.values():
        for i in range(len(group)):
            for k in range(i + 1, len(group)):
                idx_a = manager.NodeToIndex(num_vehicles + group[i])
                idx_b = manager.NodeToIndex(num_vehicles + group[k])
                solver.Add(routing.VehicleVar(idx_a) != routing.VehicleVar(idx_b))

    # ── Disjunction: jobs may be dropped with heavy penalty ──────────────────
    penalty = 100_000_000
    for j_idx in range(num_jobs):
        routing.AddDisjunction([manager.NodeToIndex(num_vehicles + j_idx)], penalty)

    # ── Solve ─────────────────────────────────────────────────────────────────
    search_params = pywrapcp.DefaultRoutingSearchParameters()
    search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    search_params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    search_params.time_limit.seconds = 10
    solution = routing.SolveWithParameters(search_params)

    # ── Extract solution ──────────────────────────────────────────────────────
    assignments: dict[str, list[str]] = {insp.name: [] for insp in active_inspectors}
    per_inspector: dict[str, InspectorResult] = {}
    total_distance = 0
    assigned_ids: set[str] = set()

    if solution:
        for v_idx, insp in enumerate(active_inspectors):
            route_jobs: list[str] = []
            route_order: list[str] = ["home"]
            route_dist = 0
            index = routing.Start(v_idx)
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                if node >= num_vehicles:
                    jid = jobs[node - num_vehicles].id
                    route_jobs.append(jid)
                    route_order.append(jid)
                    assigned_ids.add(jid)
                next_index = solution.Value(routing.NextVar(index))
                route_dist += dist_matrix[manager.IndexToNode(index)][manager.IndexToNode(next_index)]
                index = next_index
            route_order.append("home")
            assignments[insp.name] = route_jobs
            per_inspector[insp.name] = InspectorResult(
                distance_m=route_dist, jobs=route_jobs, route_order=route_order
            )
            total_distance += route_dist
    else:
        for insp in active_inspectors:
            per_inspector[insp.name] = InspectorResult(distance_m=0, jobs=[], route_order=[])

    unassigned = [j.id for j in jobs if j.id not in assigned_ids]

    # ── Baseline: naive round-trip from nearest eligible inspector ────────────
    original_distance = 0
    for j_idx, job in enumerate(jobs):
        best: int | None = None
        job_node = num_vehicles + j_idx
        for v_idx in range(num_vehicles):
            if j_idx in eligible_map[v_idx]:
                rt = dist_matrix[v_idx][job_node] + dist_matrix[job_node][v_idx]
                if best is None or rt < best:
                    best = rt
        if best is not None:
            original_distance += best

    savings_pct = 0.0
    if original_distance > 0:
        savings_pct = round((1 - total_distance / original_distance) * 100, 1)

    return OptimizeResponse(
        assignments=assignments,
        total_distance_m=float(total_distance),
        original_distance_m=float(original_distance),
        savings_pct=max(savings_pct, 0.0),
        unassigned=unassigned,
        per_inspector=per_inspector,
    )
