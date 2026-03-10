"""Pydantic data models for the NPI Route Optimizer API."""

from __future__ import annotations

from pydantic import BaseModel, Field


class ScheduleBlock(BaseModel):
    day: int  # 0=Sun, 1=Mon … 6=Sat (JS getDay convention)
    slot: str  # "8:30 AM" or "*" for whole-day block


class LatLng(BaseModel):
    lat: float
    lng: float


class PropertyRule(BaseModel):
    type: str   # "maxSqFt" | "minYearBuilt" | "maxYearBuilt"
    value: float


class Inspector(BaseModel):
    name: str
    home_lat: float
    home_lng: float
    home_state: str | None = None          # 2-letter state code, e.g. "TX"
    rank: int = 999                        # lower number = higher priority
    capabilities: list[str] = Field(default_factory=list)
    schedule_blocks: list[ScheduleBlock] = Field(default_factory=list)
    exclusion_zones: list[list[LatLng]] = Field(default_factory=list)
    property_rules: list[PropertyRule] = Field(default_factory=list)
    max_jobs: int = 3
    active: bool = True


class Job(BaseModel):
    id: str
    address: str
    lat: float
    lng: float
    time_slot: str
    required_capabilities: list[str] = Field(default_factory=list)
    locked_inspector: str | None = None
    blocked_inspectors: list[str] = Field(default_factory=list)  # block_inspector_job / _city run rules
    sq_ft: float | None = None
    year_built: int | None = None


class Settings(BaseModel):
    max_home_dist_km: float = 200
    third_job_min_time: str = "11:30 AM"
    road_factor: float = 1.35
    ranking_miles: float = 0               # max miles to concede for a higher-ranked inspector


class OptimizeRequest(BaseModel):
    date: str
    google_maps_key: str
    inspectors: list[Inspector]
    jobs: list[Job]
    settings: Settings = Field(default_factory=Settings)


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
