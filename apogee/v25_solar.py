"""APOGEE v2.5: approximate JPL heliocentric orrery states.

Source: https://ssd.jpl.nasa.gov/planets/approx_pos.html (Table 1, 1800--2050).
Planetary coordinates are heliocentric, J2000 mean ecliptic/equinox, in AU.
The Earth row is the Earth-Moon barycenter, NOT an exact Earth center.
These fitted elements are visualization-grade, not a numerical n-body simulation,
not Horizons ephemerides, and not suitable for flight/navigation.
"""
from __future__ import annotations

from datetime import datetime, timezone
from math import cos, pi, radians, sin, sqrt

AU_KM = 149597870.7
# Each row: a, e, I, L, longitude of perihelion, longitude of ascending node;
# second row: derivatives per Julian century. Orbital elements from JPL Table 1.
_ELEMENTS = {
    "Mercury": ((.38709927,.20563593,7.00497902,252.25032350,77.45779628,48.33076593),(.00000037,.00001906,-.00594749,149472.67411175,.16047689,-.12534081)),
    "Venus": ((.72333566,.00677672,3.39467605,181.97909950,131.60246718,76.67984255),(.00000390,-.00004107,-.00078890,58517.81538729,.00268329,-.27769418)),
    "Earth": ((1.00000261,.01671123,-.00001531,100.46457166,102.93768193,0.0),(.00000562,-.00004392,-.01294668,35999.37244981,.32327364,0.0)),
    "Mars": ((1.52371034,.09339410,1.84969142,-4.55343205,-23.94362959,49.55953891),(.00001847,.00007882,-.00813131,19140.30268499,.44441088,-.29257343)),
    "Jupiter": ((5.20288700,.04838624,1.30439695,34.39644051,14.72847983,100.47390909),(-.00011607,-.00013253,-.00183714,3034.74612775,.21252668,.20469106)),
    "Saturn": ((9.53667594,.05386179,2.48599187,49.95424423,92.59887831,113.66242448),(-.00125060,-.00050991,.00193609,1222.49362201,-.41897216,-.28867794)),
    "Uranus": ((19.18916464,.04725744,.77263783,313.23810451,170.95427630,74.01692503),(-.00196176,-.00004397,-.00242939,428.48202785,.40805281,.04240589)),
    "Neptune": ((30.06992276,.00859048,1.77004347,-55.12002969,44.96476227,131.78422574),(.00026291,.00005105,.00035372,218.45945325,-.32241464,-.00508664)),
}

def parse_utc(value: str | None) -> datetime:
    if not value:
        return datetime.now(timezone.utc)
    try:
        dt = datetime.fromisoformat(value.replace('Z', '+00:00'))
    except (ValueError, TypeError) as exc:
        raise ValueError('at must be an ISO-8601 datetime with timezone, e.g. 2026-10-25T12:00:00Z') from exc
    if dt.tzinfo is None:
        raise ValueError('at must specify a timezone (Z or an offset)')
    return dt.astimezone(timezone.utc)

def _jd_utc(dt: datetime) -> float:
    """Julian UTC day; approximate TDB for display only (~minute difference)."""
    y, m = dt.year, dt.month
    day = dt.day + (dt.hour + (dt.minute + (dt.second + dt.microsecond/1e6)/60)/60)/24
    if m <= 2:
        y -= 1; m += 12
    a = y//100
    b = 2-a+a//4
    return int(365.25*(y+4716))+int(30.6001*(m+1))+day+b-1524.5

# Pluto is not part of JPL Table 1. Its elements are included for orientation only.
_PLUTO = ((39.482117,.248827,17.140012,238.929038,224.068916,110.303937),
          (0,0,0,360*100/248.0,0,0))
# Circular relative orbits with arbitrary phase: not satellite ephemerides.
_MOONS = {
    "Earth": (("Moon",.00257,27.3217),),
    "Mars": (("Phobos",.000063,.319), ("Deimos",.000157,1.263)),
    "Jupiter": (("Io",.00282,1.769),("Europa",.00449,3.551),
                ("Ganymede",.00715,7.155),("Callisto",.01259,16.690)),
    "Saturn": (("Enceladus",.00159,1.370),("Titan",.00817,15.945)),
    "Uranus": (("Titania",.0029,8.706),("Oberon",.0039,13.463)),
    "Neptune": (("Triton",.00237,5.877),),
    "Pluto": (("Charon",.000131,6.387),)
}

def _planet(name: str, t_centuries: float) -> dict:
    base, rate = _PLUTO if name == 'Pluto' else _ELEMENTS[name]
    a, e, inc, L, peri, node = (base[k] + rate[k]*t_centuries for k in range(6))
    if not 0 <= e < 1:
        raise ValueError('JPL orbital eccentricity out of model range')
    mean = radians((L - peri + 180) % 360 - 180)
    E = mean
    for _ in range(25):
        step = (E - e*sin(E) - mean) / (1 - e*cos(E))
        E -= step
        if abs(step) < 1e-14:
            break
    px, py = a*(cos(E)-e), a*sqrt(1-e*e)*sin(E)
    w, O, i = radians(peri-node), radians(node), radians(inc)
    cw,sw,cO,sO,ci,si = cos(w),sin(w),cos(O),sin(O),cos(i),sin(i)
    x = (cw*cO-sw*sO*ci)*px+(-sw*cO-cw*sO*ci)*py
    y = (cw*sO+sw*cO*ci)*px+(-sw*sO+cw*cO*ci)*py
    z = (sw*si)*px+(cw*si)*py
    return {"name":name,"type":"dwarf_planet" if name=="Pluto" else "planet","position_au":[round(x,10),round(y,10),round(z,10)],
            "distance_au":round(sqrt(x*x+y*y+z*z),9), "semi_major_axis_au":round(a,9),
            "eccentricity":round(e,9),"inclination_deg":round(inc,7),
            "ephemeris_quality":"illustrative Kepler" if name=="Pluto" else "JPL fitted approximation"}

def solar_system_state(at: str | None = None) -> dict:
    dt = parse_utc(at)
    if not 1800 <= dt.year <= 2050:
        raise ValueError('JPL Table 1 model only supports years 1800 through 2050')
    jd = _jd_utc(dt)
    # UTC used in lieu of ephemeris/TDB time to keep this lower-accuracy model standalone.
    T = (jd - 2451545.0)/36525.0
    majors=[_planet(name,T) for name in _ELEMENTS]
    pluto=_planet('Pluto',T)
    all_parents={b['name']:b for b in [*majors,pluto]}
    demo_moons=[]
    for parent, moons in _MOONS.items():
        px,py,pz=all_parents[parent]['position_au']
        for name,semi_au,period_days in moons:
            phase=2*pi*((jd-2451545.0)/period_days)  # epoch phase chosen arbitrarily
            xyz=[px+semi_au*cos(phase),py+semi_au*sin(phase),pz]
            demo_moons.append({'name':name,'type':'moon','parent':parent,
                               'position_au':[round(x,10) for x in xyz],
                               'relative_distance_au':semi_au,
                               'orbital_period_days':period_days,
                               'ephemeris_quality':'illustrative circular orbit; phase arbitrary'})
    return {"model":"JPL approximate planetary elements, Table 1",
            "epoch_utc":dt.isoformat().replace('+00:00','Z'),
            "frame":"heliocentric, mean ecliptic and equinox J2000",
            "time_approximation":"Julian UTC used as proxy for ephemeris time; visualization only",
            "valid_years":[1800,2050],
            "bodies":[{"name":"Sun","type":"star","position_au":[0,0,0],"distance_au":0}] +
                     majors+[pluto]+demo_moons,
            "major_planet_count":len(majors),"illustrative_moon_count":len(demo_moons),
            "limitations":["Earth entry is Earth-Moon barycenter", "No n-body perturbations or measured positions",
                           "Pluto and moon positions are illustrative, not JPL Horizons ephemerides"]}
