"""APOGEE v2.5 Vallado SGP4 reference-vector gate.

Replaces the earlier v2.4 import gate, which incorrectly treated Vallado's
intentionally nonstandard SGP4-VER.TLE verification corpus as regular clean TLEs.

This gate NEVER edits either downloaded input. It pins original file bytes,
uses WGS72 SGP4, compares every readable reference position/velocity state,
requires substantial coverage, and writes a reproducible JSON audit.

Scope: independently check the INSTALLED ``sgp4`` library against Vallado
reference states. This does not certify other APOGEE integrators or predict
real satellite truth accuracy.
"""
from __future__ import annotations

import hashlib
import json
import math
import os
import re
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

# Exact raw-reference-file fingerprints from existing CI runs. These refer to
# the original, UNMODIFIED files, including intentionally malformed test TLEs.
# Do not update without inspecting the new official reference byte by byte.
PIN_TLE = 'd246d1d9d768ace445a38a965713fa9ba52d80fd8a41a0502ff83d7acffe2881'
PIN_VECTORS = '687bf28dbe52df86e8e60ab5cb4a08d1aa3dbcaf4e63b1f7ab95f044fbe3833b'

# 1 millimeter/s, 1 meter. Thresholds are broad enough for small floating
# point / reference rounding differences while rejecting substantive drifts.
POS_LIMIT_KM = 0.001
VEL_LIMIT_KM_S = 0.000001

# The Vallado suite includes deliberately unphysical/error-producing cases.
# An initial epoch failure with a reused 'previous' data line is permitted
# ONLY for the three documented artificial test identifiers.
KNOWN_INITIAL_ERROR_CASES = {33333, 33334, 33335}
HEADER = re.compile(r'^\s*(\d+)\s+xx\s*$', re.I)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def corpus_satellites(path: Path):
    from sgp4.api import Satrec, WGS72
    lines = path.read_text(encoding='ascii').splitlines()
    sats = []
    for i, line in enumerate(lines):
        if not line.startswith('1 '):
            continue
        if i + 1 >= len(lines) or not lines[i+1].startswith('2 '):
            raise ValueError(f'Unpaired TLE line at source line {i+1}')
        second = lines[i+1]
        number = int(line[2:7])
        if int(second[2:7]) != number:
            raise ValueError(f'Mismatched satellite numbers near line {i+1}')
        # Important: the official corpus deliberately contains bad checksums,
        # nonstandard legacy records, and appended timing fields. The SHA pin
        # authenticates the exact test corpus being interpreted here. SGP4's
        # parser intentionally does not call optional strict verify_checksum().
        sat = Satrec.twoline2rv(line[:69], second[:69], WGS72)
        if int(sat.satnum) != number:
            raise ValueError(f'Satellite-number mismatch in parser: {number}')
        sats.append((number, sat))
    if len(sats) < 20:
        raise ValueError(f'Insufficient Vallado satellite cases: {len(sats)}')
    return sats


def vector_rows(path: Path):
    current = None
    headers = []
    rows = []
    case_index = -1
    for lineno, line in enumerate(path.read_text(encoding='ascii').splitlines(), 1):
        if not line.strip():
            continue
        match = HEADER.fullmatch(line)
        if match:
            current = int(match.group(1))
            headers.append(current)
            case_index += 1
            continue
        if current is None:
            raise ValueError(f'Reference row before satellite header at line {lineno}')
        fields = line.split()
        if len(fields) < 7:
            raise ValueError(f'Short reference vector row at line {lineno}: {line[:80]}')
        try:
            numbers = tuple(float(s) for s in fields[:7])
        except ValueError as exc:
            raise ValueError(f'Invalid reference vector at line {lineno}: {line[:80]}') from exc
        if not all(math.isfinite(x) for x in numbers):
            raise ValueError(f'Non-finite reference row at line {lineno}')
        rows.append((lineno, case_index, current, numbers))
    if len(headers) < 20:
        raise ValueError(f'Insufficient reference headers: {len(headers)}')
    if len(rows) < 100:
        raise ValueError(f'Insufficient reference vectors: {len(rows)}')
    return headers, rows


def audit(tle_path: Path, vectors_path: Path):
    sats = corpus_satellites(tle_path)
    headers, rows = vector_rows(vectors_path)
    source_ids = [number for number, _sat in sats]
    if headers != source_ids:
        raise ValueError(f'Reference case order differs from TLE corpus: '
                         f'corpus={source_ids}, vectors={headers}')

    errors = []
    skipped = []
    failures = []
    tested = 0
    max_pos = max_vel = 0.0
    tested_satellites = set()
    for lineno, case_index, number, values in rows:
        minutes, *reference_state = values
        code, position, velocity = sats[case_index][1].sgp4_tsince(minutes)
        if code:
            # An official special case can contain a reused previous state
            # at epoch because propagation immediately signaled an error.
            if number in KNOWN_INITIAL_ERROR_CASES and minutes == 0.0:
                skipped.append({'satellite': number, 'minute': minutes,
                                'line': lineno, 'sgp4_error': code,
                                'reason': 'documented initial error fixture'})
                continue
            failures.append({'satellite': number, 'minute': minutes,
                             'line': lineno, 'sgp4_error': code})
            continue
        if not all(math.isfinite(x) for x in (*position, *velocity)):
            failures.append({'satellite': number, 'minute': minutes,
                             'line': lineno, 'problem': 'non-finite SGP4 state'})
            continue
        pos_error = math.dist(position, reference_state[:3])
        vel_error = math.dist(velocity, reference_state[3:])
        max_pos = max(max_pos, pos_error)
        max_vel = max(max_vel, vel_error)
        tested += 1
        tested_satellites.add(number)
        if pos_error > POS_LIMIT_KM or vel_error > VEL_LIMIT_KM_S:
            if len(errors) < 20:
                errors.append({'satellite': number, 'minute': minutes,
                               'line': lineno, 'position_error_km': pos_error,
                               'velocity_error_km_s': vel_error})
    if tested < 100 or len(tested_satellites) < 15:
        failures.append({'problem': 'insufficient numerical coverage',
                         'verified_rows': tested, 'verified_satellites': len(tested_satellites)})

    return {'release_gate': 'FAIL' if errors or failures else 'PASS',
            'scope': 'WGS72 SGP4 vs published Vallado reference vectors only',
            'reference_cases': len(headers), 'reference_vector_rows': len(rows),
            'validated_rows': tested, 'validated_satellites': len(tested_satellites),
            'documented_initial_errors': skipped,
            'position_limit_km': POS_LIMIT_KM,
            'velocity_limit_km_s': VEL_LIMIT_KM_S,
            'max_position_error_km': max_pos,
            'max_velocity_error_km_s': max_vel,
            'mismatch_examples': errors, 'hard_failures': failures}


def main():
    root = Path(__file__).resolve().parents[1]
    reports = root / 'reports'
    reports.mkdir(parents=True, exist_ok=True)
    output = reports / 'v25-render-audit.json'
    inputs = root / 'reference_inputs'
    tle = inputs / 'SGP4-VER.TLE'
    vec = inputs / 'tcppver.out'
    base = {'release_gate': 'FAIL', 'created_utc': datetime.now(timezone.utc).isoformat(),
            'implementation': 'sgp4.api.Satrec.twoline2rv / sgp4_tsince',
            'reference_unmodified': True,
            'flight_certified': False}
    try:
        if not (tle.is_file() and vec.is_file()):
            script = root / 'scripts_fetch_vallado.py'
            if not script.is_file():
                raise RuntimeError('Missing scripts_fetch_vallado.py')
            result = subprocess.run([sys.executable, str(script), '--confirm-download'],
                                    check=False)
            if result.returncode:
                raise RuntimeError(f'Reference download failed, exit {result.returncode}')
        for path, name, expected in ((tle, 'tle', PIN_TLE),
                                     (vec, 'vectors', PIN_VECTORS)):
            if not path.is_file():
                raise RuntimeError(f'Missing reference input: {path}')
            actual = sha256(path)
            base[f'{name}_sha256'] = actual
            env_name = 'APOGEE_V25_TLE_SHA256' if name == 'tle' else 'APOGEE_V25_VECTORS_SHA256'
            # Env hashes may tighten provenance but cannot override built-in pins.
            if actual != expected or (os.environ.get(env_name) and
                                      actual != os.environ[env_name].lower()):
                raise RuntimeError(f'Unrecognized {name} reference SHA-256: {actual}')
        base.update(audit(tle, vec))
    except Exception as exc:
        base['release_gate'] = 'FAIL'
        base['reason'] = f'{type(exc).__name__}: {exc}'
    output.write_text(json.dumps(base, indent=2, allow_nan=False) + '\n', encoding='utf8')
    print(f'APOGEE v2.5 reference audit: {base["release_gate"]}')
    print(f'Audit: {output}')
    if 'reason' in base:
        print('Error:', base['reason'])
    else:
        print('Reference vectors verified:', base['validated_rows'],
              'of', base['reference_vector_rows'],
              'maximum position error (km):', base['max_position_error_km'],
              'maximum velocity error (km/s):', base['max_velocity_error_km_s'])
        print('Mismatches:', len(base['mismatch_examples']),
              'failures:', len(base['hard_failures']))
    if base['release_gate'] != 'PASS':
        sys.exit(1)


if __name__ == '__main__':
    main()
