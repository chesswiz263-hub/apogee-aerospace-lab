"""Standalone programmatic access to the physics engine (no API needed)."""
import argparse
import csv
import json
from math import pi, sqrt

import numpy as np

from .constants import EARTH_EQUATORIAL_RADIUS_KM as R_EARTH, EARTH_MU_KM3_S2 as MU
from .physics import state_from_elements, integrate_orbit, trajectory_diagnostics, elements_from_state


def main():
    p = argparse.ArgumentParser(description="APOGEE v0.1 Newtonian orbital propagator")
    p.add_argument("--altitude", type=float, default=400, help="Periapsis altitude (km)")
    p.add_argument("--eccentricity", type=float, default=0.0)
    p.add_argument("--inclination", type=float, default=51.6, help="degrees")
    p.add_argument("--orbits", type=float, default=1)
    p.add_argument("--steps-per-orbit", type=int, default=360)
    p.add_argument("--csv", help="optional path for time/position/velocity output")
    args = p.parse_args()
    if args.altitude < 160 or args.altitude > 50000 or not 0 <= args.eccentricity <= 0.75:
        p.error("altitude must be in [160,50000] km and eccentricity in [0,0.75]")
    if not (0 <= args.inclination <= 180 and 0.25 <= args.orbits <= 8 and 90 <= args.steps_per_orbit <= 720):
        p.error("inclination [0,180], orbits [0.25,8], steps-per-orbit [90,720]")
    a = (R_EARTH + args.altitude) / (1 - args.eccentricity)
    state = state_from_elements(a, args.eccentricity, args.inclination)
    period = 2 * pi * sqrt(a ** 3 / MU)
    result = integrate_orbit(state, period * args.orbits, round(args.steps_per_orbit * args.orbits))
    diag = trajectory_diagnostics(result.states)
    print(json.dumps({"orbital_elements": elements_from_state(state).to_dict(),
                      "period_min": period / 60, "sample_count": len(result.states),
                      "impact": result.impact,
                      "energy_drift": diag["max_relative_energy_drift"],
                      "angular_momentum_drift": diag["max_relative_angular_momentum_drift"]}, indent=2))
    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(["time_s", "x_km", "y_km", "z_km", "vx_km_s", "vy_km_s", "vz_km_s"])
            for t, s in zip(result.time_s, result.states):
                writer.writerow([float(t), *[float(x) for x in s]])
        print("Saved", args.csv)


if __name__ == "__main__":
    main()
