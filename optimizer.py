"""OR-Tools VRPTW optimizer for NPI Route Optimizer — v3.

Architecture:
  Frontend pre-computes eligible_inspectors[] per job (all constraint checks).
  This solver assigns jobs to eligible inspectors minimizing total drive distance.

OR-Tools constraints:
  1. Eligibility    — VehicleVar restricted to eligible_inspectors[] per job
  2. Locked         — locked_inspector forces VehicleVar to single vehicle
  3. Max jobs       — per-inspector capacity dimension
  4. Time windows   — appointment time + duration_hours prevents overlapping jobs
  5. Disjunction    — jobs may be dropped with heavy penalty (prevents infeasible)

v3 changes:
  - Duration-based scheduling: uses job.duration_hours instead of hardcoded 150 min
  - Overlap prevention via time dimension (not just exact-match time_minutes)
  - Removed _SERVICE_MIN constant
"""

from __future__ import annotations

from ortools.constraint_solver import pywrapcp, routing_enums_pb2

from geocoder import build_distance_matrix
from models import InspectorResult, Job, OptimizeRequest, OptimizeResponse

_TRAVEL_SPEED_KM_PER_MIN = 1.0  # ~60 km/h average


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

    # ── Per-job service time (minutes) ────────────────────────────────────────
    job_service_min: list[int] = [
        int(round(j.duration_hours * 60)) for j in jobs
    ]

    # ── OR-Tools model ────────────────────────────────────────────────────────
    manager = pywrapcp.RoutingIndexManager(
        num_nodes,
        num_vehicles,
        list(range(num_vehicles)),   # start: each inspector's home node
        list(range(num_vehicles)),   # end:   same (return home)
    )
    routing = pywrapcp.RoutingModel(manager)

    # ── Arc cost: pure distance ───────────────────────────────────────────────
    def dist_cb(from_idx: int, to_idx: int) -> int:
        return dist_matrix[manager.IndexToNode(from_idx)][manager.IndexToNode(to_idx)]

    dist_cb_idx = routing.RegisterTransitCallback(dist_cb)
    routing.SetArcCostEvaluatorOfAllVehicles(dist_cb_idx)

    # ── Capacity: max jobs per inspector ──────────────────────────────────────
    def demand_cb(from_idx: int) -> int:
        return 1 if manager.IndexToNode(from_idx) >= num_vehicles else 0

    demand_cb_idx = routing.RegisterUnaryTransitCallback(demand_cb)
    routing.AddDimensionWithVehicleCapacity(
        demand_cb_idx, 0,
        [i.max_jobs for i in inspectors],
        True, "Capacity",
    )

    # ── Time dimension: travel time + service time (duration_hours) ───────────
    # Transit = travel time (distance / speed) + service time at the FROM node.
    # Service time at home nodes (inspector depots) is 0.
    # Service time at job nodes is the job's duration in minutes.
    def time_cb(from_idx: int, to_idx: int) -> int:
        from_node = manager.IndexToNode(from_idx)
        to_node = manager.IndexToNode(to_idx)
        # Travel time: distance in metres → km → minutes at travel speed
        travel_min = int(dist_matrix[from_node][to_node] / 1000 / _TRAVEL_SPEED_KM_PER_MIN)
        # Service time at the FROM node (only for job nodes, not depots)
        service = 0
        if from_node >= num_vehicles:
            service = job_service_min[from_node - num_vehicles]
        return travel_min + service

    time_cb_idx = routing.RegisterTransitCallback(time_cb)
    day_start = 0
    day_end = 24 * 60  # 1440 minutes
    routing.AddDimension(time_cb_idx, day_end, day_end, False, "Time")
    time_dim = routing.GetDimensionOrDie("Time")

    # ── Time windows: each job must be visited at its appointment time ────────
    # The cumul var represents ARRIVAL time at the node.
    # Setting [time_minutes, time_minutes] forces arrival at the exact appointment.
    # Jobs without a time get the full day window.
    for j_idx, job in enumerate(jobs):
        node_idx = manager.NodeToIndex(num_vehicles + j_idx)
        if job.time_minutes is not None:
            # Fixed appointment time — arrive at this exact minute
            lo = max(day_start, job.time_minutes)
            hi = min(day_end, job.time_minutes)
            time_dim.CumulVar(node_idx).SetRange(lo, hi)
        else:
            time_dim.CumulVar(node_idx).SetRange(day_start, day_end)

    # Inspector start/end windows: full day
    for v in range(num_vehicles):
        time_dim.CumulVar(routing.Start(v)).SetRange(day_start, day_end)
        time_dim.CumulVar(routing.End(v)).SetRange(day_start, day_end)

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
