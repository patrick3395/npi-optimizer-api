"""Pydantic data models for the NPI Route Optimizer API — v2.

Payload shape (from frontend):
  - eligible_inspectors[] pre-computed per job — all constraint logic runs on frontend
  - No API keys, no capabilities, no schedules, no exclusion zones
  - Cloud Run does pure distance-based global assignment optimization
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class Inspector(BaseModel):
    name: str
    home_lat: float
    home_lng: float
    max_jobs: int = 2       # hard cap — frontend sets per-inspector value
    rank: int = 999         # lower = higher priority (used for tie-breaking)


class Job(BaseModel):
    id: str
    lat: float
    lng: float
    time_minutes: int | None = None         # minutes since midnight, None = no scheduled time
    duration_hours: float = 2.5
    eligible_inspectors: list[str]          # pre-filtered by frontend — only valid candidates
    locked_inspector: str | None = None     # must be assigned to this inspector


class OptimizeRequest(BaseModel):
    date: str
    inspectors: list[Inspector]
    jobs: list[Job]


class InspectorResult(BaseModel):
    distance_m: float
    jobs: list[str]
    route_order: list[str]


class OptimizeResponse(BaseModel):
    assignments: dict[str, list[str]]
    total_distance_m: float
    original_distance_m: float
    savings_pct: float
    unassigned: list[str]
    per_inspector: dict[str, InspectorResult]
