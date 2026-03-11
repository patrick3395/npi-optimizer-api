"""Distance matrix using haversine * road_factor. No external API needed."""

from __future__ import annotations

import math
from typing import Sequence

EARTH_RADIUS_M = 6_371_000
ROAD_FACTOR = 1.35


def haversine_m(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    """Return straight-line distance in metres between two lat/lng points."""
    rlat1, rlng1, rlat2, rlng2 = (
        math.radians(lat1), math.radians(lng1),
        math.radians(lat2), math.radians(lng2),
    )
    dlat = rlat2 - rlat1
    dlng = rlng2 - rlng1
    a = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlng / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def build_distance_matrix(
    coords: Sequence[tuple[float, float]],
    road_factor: float = ROAD_FACTOR,
) -> list[list[int]]:
    """Build NxN integer distance matrix (metres) using haversine * road_factor."""
    n = len(coords)
    matrix: list[list[int]] = [[0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if i != j:
                lat1, lng1 = coords[i]
                lat2, lng2 = coords[j]
                matrix[i][j] = int(haversine_m(lat1, lng1, lat2, lng2) * road_factor)
    return matrix
