"""Google Geocoding & Distance Matrix helpers with haversine fallback."""

from __future__ import annotations

import math
from typing import Sequence

import requests

EARTH_RADIUS_M = 6_371_000


def haversine_m(lat1: float, lng1: float, lat2: float, lng2: float) -> float:
    """Return distance in metres between two lat/lng points."""
    rlat1, rlng1, rlat2, rlng2 = (
        math.radians(lat1),
        math.radians(lng1),
        math.radians(lat2),
        math.radians(lng2),
    )
    dlat = rlat2 - rlat1
    dlng = rlng2 - rlng1
    a = math.sin(dlat / 2) ** 2 + math.cos(rlat1) * math.cos(rlat2) * math.sin(dlng / 2) ** 2
    return 2 * EARTH_RADIUS_M * math.asin(math.sqrt(a))


def _batch_distance_matrix(
    origins: list[tuple[float, float]],
    destinations: list[tuple[float, float]],
    api_key: str,
) -> list[list[float | None]]:
    """Call Google Distance Matrix API for one batch.

    Returns a 2-D list of distances in metres (None on failure).
    """
    origin_str = "|".join(f"{lat},{lng}" for lat, lng in origins)
    dest_str = "|".join(f"{lat},{lng}" for lat, lng in destinations)
    resp = requests.get(
        "https://maps.googleapis.com/maps/api/distancematrix/json",
        params={
            "origins": origin_str,
            "destinations": dest_str,
            "key": api_key,
            "units": "metric",
        },
        timeout=15,
    )
    data = resp.json()
    rows: list[list[float | None]] = []
    if data.get("status") != "OK":
        return [[None] * len(destinations) for _ in origins]
    for row in data["rows"]:
        cols: list[float | None] = []
        for elem in row["elements"]:
            if elem.get("status") == "OK":
                cols.append(float(elem["distance"]["value"]))
            else:
                cols.append(None)
        rows.append(cols)
    return rows


def build_distance_matrix(
    coords: Sequence[tuple[float, float]],
    api_key: str,
    road_factor: float = 1.35,
    batch_size: int = 10,
) -> list[list[int]]:
    """Build a full NxN distance matrix (values in metres, integers for OR-Tools).

    Uses Google Distance Matrix API in ``batch_size x batch_size`` chunks.
    Falls back to haversine * road_factor when the API call fails.
    """
    n = len(coords)
    matrix: list[list[int]] = [[0] * n for _ in range(n)]

    for o_start in range(0, n, batch_size):
        o_end = min(o_start + batch_size, n)
        for d_start in range(0, n, batch_size):
            d_end = min(d_start + batch_size, n)
            origins = list(coords[o_start:o_end])
            destinations = list(coords[d_start:d_end])
            try:
                batch = _batch_distance_matrix(origins, destinations, api_key)
            except Exception:
                batch = [[None] * len(destinations) for _ in origins]

            for i_local, i_global in enumerate(range(o_start, o_end)):
                for j_local, j_global in enumerate(range(d_start, d_end)):
                    val = batch[i_local][j_local] if batch else None
                    if val is not None:
                        matrix[i_global][j_global] = int(val)
                    else:
                        lat1, lng1 = coords[i_global]
                        lat2, lng2 = coords[j_global]
                        matrix[i_global][j_global] = int(
                            haversine_m(lat1, lng1, lat2, lng2) * road_factor
                        )
    return matrix
