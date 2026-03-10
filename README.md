# NPI Route Optimizer API

A Google Cloud Run service that optimizes inspector-to-job assignments using OR-Tools VRPTW (Vehicle Routing Problem with Time Windows).

## Local Development

```bash
pip install -r requirements.txt
python main.py
```

The server starts on `http://localhost:8080`.

## Endpoints

| Method | Path        | Description              |
|--------|-------------|--------------------------|
| POST   | `/optimize` | Run route optimization   |
| GET    | `/health`   | Health check             |

## Deploy to Cloud Run

```bash
gcloud run deploy npi-optimizer-api \
  --source . \
  --region us-central1 \
  --allow-unauthenticated \
  --memory 1Gi \
  --timeout 60
```

## Request / Response

Send a `POST /optimize` with JSON body:

```json
{
  "date": "2026-03-11",
  "google_maps_key": "YOUR_KEY",
  "inspectors": [
    {
      "name": "Calvin Williams",
      "home_lat": 29.99,
      "home_lng": -95.48,
      "capabilities": ["Mold", "Pool", "WDO/Termite", "Foundation"],
      "schedule_blocks": [{"day": "Monday", "slot": "8:30 AM"}],
      "exclusion_zones": [[{"lat": 29.7, "lng": -95.3}]],
      "max_jobs": 3,
      "active": true
    }
  ],
  "jobs": [
    {
      "id": "job-1",
      "address": "29611 Legends Stone Dr, Spring, TX 77386",
      "lat": 30.12,
      "lng": -95.38,
      "time_slot": "9:00 AM",
      "required_capabilities": ["Foundation"],
      "locked_inspector": null
    }
  ],
  "settings": {
    "max_home_dist_km": 200,
    "third_job_min_time": "11:30 AM",
    "road_factor": 1.35
  }
}
```

Response:

```json
{
  "assignments": {"Calvin Williams": ["job-1"]},
  "total_distance_m": 245000,
  "original_distance_m": 312000,
  "savings_pct": 21.5,
  "unassigned": [],
  "per_inspector": {
    "Calvin Williams": {
      "distance_m": 120000,
      "jobs": ["job-1"],
      "route_order": ["home", "job-1", "home"]
    }
  }
}
```

## Constraints

All constraints are driven by the request body — nothing is hardcoded.

| # | Constraint | How it works |
|---|-----------|--------------|
| 1 | **Capabilities** | `inspector.capabilities` must be a superset of `job.required_capabilities`. Both are dynamic string arrays — any value is valid. |
| 2 | **Time windows** | No two jobs with the same `time_slot` are assigned to the same inspector. OR-Tools time windows (slot ± 30 min) plus service time enforce this. |
| 3 | **Capacity / 3rd-job rule** | Each inspector has `max_jobs` capacity. A 3rd job is only allowed if its `time_slot` is at or after `settings.third_job_min_time`. |
| 4 | **Distance** | Haversine distance from inspector home to job must be ≤ `settings.max_home_dist_km`. |
| 5 | **Locked inspector** | When `job.locked_inspector` is set, only that inspector can be assigned the job. |
| 6 | **Exclusion zones** | Point-in-polygon ray-casting test. If a job's coordinates fall inside any of an inspector's exclusion zone polygons, that inspector is ineligible. |
| 7 | **Schedule blocks** | If the inspector has a block on the same weekday and time slot as the job, they are skipped. |
| 8 | **State filtering** | The two-letter US state code is parsed from `job.address`. Inspectors are only assigned jobs in the same state (when determinable). |

## Distance Matrix

The service calls the Google Distance Matrix API in 10×10 batches using the provided `google_maps_key`. If any batch fails, it falls back to haversine distance multiplied by `settings.road_factor`.
