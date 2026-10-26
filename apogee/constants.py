"""Constants and units: kilometers, seconds, kilograms, radians.

Earth GM: NASA/JPL DE440 astrodynamic parameters:
https://ssd.jpl.nasa.gov/astro_par.html
Earth equatorial radius: WGS-84 reference ellipsoid (km).
The Earth reference radius is used for nominal spherical altitude and impact checks.
"""
EARTH_MU_KM3_S2 = 398600.435507
EARTH_EQUATORIAL_RADIUS_KM = 6378.137
EARTH_J2 = 1.08262668e-3  # Leading zonal harmonic; active in coupled and perturbation models.
SECONDS_PER_HOUR = 3600.0
