"""Flask app for the NPI Route Optimizer API."""

from __future__ import annotations

import os

from flask import Flask, jsonify, request
from pydantic import ValidationError

from models import OptimizeRequest
from optimizer import optimize

app = Flask(__name__)


@app.route("/optimize", methods=["POST"])
def optimize_route():
    """Accept an optimization request and return assignments."""
    try:
        payload = OptimizeRequest.model_validate(request.get_json(force=True))
    except ValidationError as exc:
        return jsonify({"error": exc.errors()}), 422

    result = optimize(payload)
    return jsonify(result.model_dump())


@app.route("/health", methods=["GET"])
def health():
    return jsonify({"status": "ok"})


if __name__ == "__main__":
    port = int(os.environ.get("PORT", 8080))
    app.run(host="0.0.0.0", port=port, debug=True)
