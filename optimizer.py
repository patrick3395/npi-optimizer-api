"""OR-Tools VRPTW optimizer for NPI Route Optimizer — v2.

Architecture:
  Frontend pre-computes eligible_inspectors[] per job (all constraint checks).
  This solver only needs to assign jobs to eligible inspectors minimizing total drive distance.

OR-Tools constraints:
  1. Eligibility    — VehicleVar restricted to eligible_inspectors[] per job
  2. Locked         — locked_inspector forces VehicleVar to single vehicle
  3. Max jobs       — per-inspector capacity dimension
  4. Time conflicts — same time_minutes → different inspectors (no double-booking)
  5. Disjunction    — jobs may be dropped with heavy penalty (prevents infeasible)
"""

from __future__ import annotations

from collections import defaultdict

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

from geocoder import build_distance_matrix
from models import InspectorResult, Job, OptimizeRequest, OptimizeResponse

_TIME_BUFFER_MIN = 45   # ± window around appointment time for time dimension
_SERVICE_MIN = 150      # default on-site time in minutes (2.5 hrs)


def optimize(request: OptimizeRequest) -> OptimizeResponse:
    inspectors = request.inspectors
    jobs = list(request.jobs)

    if not inspectors or not jobs:
        return OptimizeResponse(
            assignments={i.name: [] for i in inspectors},
            total_distance_m=0,
            original_distance_m=0,
            savings_pct=0.0,
            unassigned=[j.id for j in jobs],
            per_inspector={i.name: InspectorResult(distance_m=0, jobs=[], route_order=[]) for i in inspectors},
        )

    num_vehicles = len(inspectors)
    num_jobs = len(jobs)
    num_nodes = num_vehicles + num_jobs

    # Build name → vehicle index map
    insp_idx: dict[str, int] = {ins.name: v for v, ins in enumerate(inspectors)}

    # Node coords: [homes..., jobs...]
    coords: list[tuple[float, float]] = (
        [(i.home_lat, i.home_lng) for i in inspectors] +
        [(j.lat, j.lng) for j in jobs]
    )
    # Use Google Maps distance matrix if provided by frontend, else Haversine × 1.35
    if request.distance_matrix and len(request.distance_matrix) == len(coords):
        dist_matrix = request.distance_matrix
        print(f"[optimizer] Using Google Maps distance matrix ({len(coords)}×{len(coords)})")
    else:
        dist_matrix = build_distance_matrix(coords)
        print(f"[optimizer] Using Haversine distance matrix ({len(coords)}×{len(coords)})")

    # ── OR-Tools model ────────────────────────────────────────────────────────
    manager = pywrapcp.RoutingIndexManager(
        num_nodes,
        num_vehicles,
        list(range(num_vehicles)),   # start: each inspector's home node
        list(range(num_vehicles)),   # end:   same (return home)
    )
    routing = pywrapcp.RoutingModel(manager)

    # ── Arc cost: distance + small rank penalty for tie-breaking ─────────────
    sorted_ranks = sorted(set(i.rank for i in inspectors))
    rank_pos = {r: idx for idx, r in enumerate(sorted_ranks)}

    def _make_cost_cb(v: int):
        penalty = rank_pos.get(inspectors[v].rank, 0) * 25   # 25m per rank step (tie-breaking only)
        def cb(from_idx: int, to_idx: int) -> int:
            base = dist_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)]
            return base + penalty if manager.IndexToNode(to_idx) >= num_vehicles else base
        return cb

    for v in range(num_vehicles):
        cb_idx = routing.RegisterTransitCallback(_make_cost_cb(v))
        routing.SetArcCostEvaluatorOfVehicle(cb_idx, v)

    # ── Capacity: max jobs per inspector ──────────────────────────────────────
    def demand_cb(from_idx: int) -> int:
        return 1 if manager.IndexToNode(from_idx) >= num_vehicles else 0

    demand_cb_idx = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        demand_cb_idx, 0,
        [i.max_jobs for i in inspectors],
        True, "Capacity",
    )

    # ── Time dimension ────────────────────────────────────────────────────────
    def time_cb(from_idx: int, to_idx: int) -> int:
        dist_m = dist_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)]
        return int(dist_m / 1000) + _SERVICE_MIN  # 1 min/km travel + service time

    time_cb_idx = routing.RegisterTransitCallback(time_cb)
    day_end = 24 * 60
    routing.AddDimension(time_cb_idx, day_end, day_end, False, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    for j_idx, job in enumerate(jobs):
        if job.time_minutes is not None:
            node_idx = manager.NodeToIndex(num_vehicles + j_idx)
            lo = max(0, job.time_minutes - _TIME_BUFFER_MIN)
            hi = min(day_end, job.time_minutes + _TIME_BUFFER_MIN)
            time_dim.CumulVar(node_idx).SetRange(lo, hi)

    for v in range(num_vehicles):
        time_dim.CumulVar(routing.Start(v)).SetRange(0, day_end)
        time_dim.CumulVar(routing.End(v)).SetRange(0, day_end)

    # ── Eligibility: restrict VehicleVar to eligible_inspectors only ──────────
    for j_idx, job in enumerate(jobs):
        node_idx = manager.NodeToIndex(num_vehicles + j_idx)
        eligible_set = set(job.eligible_inspectors)
        for v, ins in enumerate(inspectors):
            if ins.name not in eligible_set:
                routing.VehicleVar(node_idx).RemoveValue(v)

    # ── Locked inspector ──────────────────────────────────────────────────────
    for j_idx, job in enumerate(jobs):
        if job.locked_inspector and job.locked_inspector in insp_idx:
            v = insp_idx[job.locked_inspector]
            routing.VehicleVar(manager.NodeToIndex(num_vehicles + j_idx)).SetValue(v)

    # ── No double-booking: same time_minutes → different inspectors ───────────
    time_groups: dict[int, list[int]] = defaultdict(list)
    for j_idx, job in enumerate(jobs):
        if job.time_minutes is not None:
            time_groups[job.time_minutes].append(j_idx)

    solver = routing.solver()
    for group in time_groups.values():
        for a in range(len(group)):
            for b in range(a + 1, len(group)):
                idx_a = manager.NodeToIndex(num_vehicles + group[a])
                idx_b = manager.NodeToIndex(num_vehicles + group[b])
                solver.Add(routing.VehicleVar(idx_a) != routing.VehicleVar(idx_b))

    # ── Disjunction: allow dropping jobs with heavy penalty ───────────────────
    penalty = 100_000_000
    for j_idx in range(num_jobs):
        routing.AddDisjunction([manager.NodeToIndex(num_vehicles + j_idx)], penalty)

    # ── Solve ─────────────────────────────────────────────────────────────────
    params = pywrapcp.DefaultRoutingSearchParameters()
    params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    params.local_search_metaheuristic = routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH
    params.time_limit.seconds = 10
    solution = routing.SolveWithParameters(params)

    # ── Extract solution ──────────────────────────────────────────────────────
    assignments: dict[str, list[str]] = {i.name: [] for i in inspectors}
    per_inspector: dict[str, InspectorResult] = {}
    total_dist = 0
    assigned_ids: set[str] = set()

    if solution:
        for v, ins in enumerate(inspectors):
            route_jobs: list[str] = []
            route_dist = 0
            index = routing.Start(v)
            while not routing.IsEnd(index):
                node = manager.IndexToNode(index)
                next_index = solution.Value(routing.NextVar(index))
                route_dist += dist_matrix[node][manager.IndexToNode(next_index)]
                if node >= num_vehicles:
                    jid = jobs[node - num_vehicles].id
                    route_jobs.append(jid)
                    assigned_ids.add(jid)
                index = next_index
            assignments[ins.name] = route_jobs
            per_inspector[ins.name] = InspectorResult(
                distance_m=route_dist,
                jobs=route_jobs,
                route_order=["home"] + route_jobs + ["home"],
            )
            total_dist += route_dist
    else:
        for ins in inspectors:
            per_inspector[ins.name] = InspectorResult(distance_m=0, jobs=[], route_order=[])

    unassigned = [j.id for j in jobs if j.id not in assigned_ids]

    # ── Baseline: naive nearest-inspector round-trip per job ─────────────────
    original_dist = 0
    for j_idx in range(num_jobs):
        job_node = num_vehicles + j_idx
        eligible_vs = [insp_idx[name] for name in jobs[j_idx].eligible_inspectors if name in insp_idx]
        if eligible_vs:
            best = min(dist_matrix[v][job_node] + dist_matrix[job_node][v] for v in eligible_vs)
            original_dist += best

    savings_pct = round((1 - total_dist / original_dist) * 100, 1) if original_dist > 0 else 0.0

    return OptimizeResponse(
        assignments=assignments,
        total_distance_m=float(total_dist),
        original_distance_m=float(original_dist),
        savings_pct=max(savings_pct, 0.0),
        unassigned=unassigned,
        per_inspector=per_inspector,
    )
