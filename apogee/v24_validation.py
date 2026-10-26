"""APOGEE v2.4 reproducible, fail-closed *software* verification gate.

This is not validation against observed spacecraft trajectories.  Only an
explicitly supplied external Vallado corpus can qualify as full coverage.
A skipped dependency can never produce a release-gate PASS.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
from importlib import metadata
import json
from pathlib import Path
import platform
import sys
from typing import Any

from .independent_validation import analytic_benchmark
from .v23_validation import CURATED, astropy_roundtrip, compare_many, parse_vallado_raw

VERSION = '2.4.0'
MIN_CASES = 33
MIN_VECTORS = 666


def _package_version(name: str) -> str | None:
    try:
        return metadata.version(name)
    except metadata.PackageNotFoundError:
        return None


def _fingerprint(content: str) -> str:
    return hashlib.sha256(content.encode('utf-8')).hexdigest()


def evaluate(*, steps_per_orbit: int = 240, orbits: int = 2,
             tle_text: str | None = None, vector_text: str | None = None,
             expected_tle_sha256: str | None = None,
             expected_vector_sha256: str | None = None) -> dict[str, Any]:
    """Build numerical, provenance and coverage audit; all checks are explicit.

    Passing the v2.4 strict gate means only that the named software-verification
    criteria ran and passed. It NEVER authenticates satellite observations.
    """
    if (tle_text is None) != (vector_text is None):
        raise ValueError('Supply BOTH Vallado files or NEITHER; no mixed-source comparisons')
    if not 40 <= steps_per_orbit <= 600 or not 1 <= orbits <= 5:
        raise ValueError('Steps must be 40..600 per orbit and orbits must be 1..5')
    for digest in (expected_tle_sha256, expected_vector_sha256):
        if digest is not None and (len(digest) != 64 or any(c not in '0123456789abcdef' for c in digest.lower())):
            raise ValueError('Expected SHA256 must be 64 hexadecimal characters')
    if tle_text is None and (expected_tle_sha256 or expected_vector_sha256):
        raise ValueError('Hash pins require supplied source files')

    baseline = analytic_benchmark(steps_per_orbit=steps_per_orbit, orbits=orbits)
    refined = analytic_benchmark(steps_per_orbit=steps_per_orbit * 2, orbits=orbits)
    p0, p1 = baseline['max_position_error_m'], refined['max_position_error_m']
    ratio = p0 / max(p1, 1e-15)
    convergence = {
        'case': 'rk4_refinement',
        'status': 'PASS' if ratio >= 8 and p1 < p0 and refined['status'] == 'PASS' else 'FAIL',
        'coarse_steps_per_orbit': steps_per_orbit,
        'fine_steps_per_orbit': steps_per_orbit * 2,
        'coarse_max_position_error_m': p0,
        'fine_max_position_error_m': p1,
        'error_reduction_factor': ratio,
        'minimum_reduction_factor': 8,
        'reference_type': 'independent_analytic_kepler_solution',
    }

    supplied = tle_text is not None
    if supplied:
        parsed = parse_vallado_raw(tle_text, vector_text)
        source = parsed['reference']
        tle_sha = _fingerprint(tle_text)
        vectors_sha = _fingerprint(vector_text)
        pins = {
            'tle': expected_tle_sha256 is not None,
            'vectors': expected_vector_sha256 is not None,
        }
        pin_ok = (expected_tle_sha256 is None or expected_tle_sha256.lower() == tle_sha) and (
            expected_vector_sha256 is None or expected_vector_sha256.lower() == vectors_sha
        )
        verification = {'tle_sha256': tle_sha, 'vectors_sha256': vectors_sha,
                        'expected_sha256_provided': pins,
                        'expected_sha256_match': pin_ok,
                        'upstream_signature_verified': False}
        if not pin_ok:
            # Fail hard before propagating untrusted/misidentified inputs.
            raise ValueError('External source hash does not match the expected SHA256 pin')
        source_name = 'user_supplied_vallado_files'
    else:
        source_bytes = CURATED.read_bytes()
        source = json.loads(source_bytes)
        verification = {'fixture_sha256': hashlib.sha256(source_bytes).hexdigest(),
                        'expected_sha256_provided': {'tle': False, 'vectors': False},
                        'expected_sha256_match': None,
                        'upstream_signature_verified': False}
        parsed = None
        source_name = 'bundled_four_case_excerpt'

    sgp4 = compare_many(source)
    case_count = sgp4['cases']
    sample_count = sgp4['sample_count']
    complete = supplied and case_count >= MIN_CASES and sample_count >= MIN_VECTORS
    coverage = {
        'case': 'vallado_full_corpus_coverage',
        'status': 'PASS' if complete else 'SKIP',
        'external_source_supplied': supplied,
        'cases_with_vectors': case_count,
        'state_vectors': sample_count,
        'expected_min_cases': MIN_CASES,
        'expected_min_vectors': MIN_VECTORS,
        'source': source_name,
        'reason': 'Complete candidate corpus supplied and counted' if complete else
                  'Bundled excerpts or incomplete corpus: NOT a full Vallado regression',
    }
    frames = astropy_roundtrip()
    checks = [baseline, convergence, coverage, sgp4, frames]
    failed = sum(x['status'] == 'FAIL' for x in checks)
    skipped = sum(x['status'] == 'SKIP' for x in checks)
    passed = sum(x['status'] == 'PASS' for x in checks)
    gate = 'FAIL' if failed else ('PARTIAL' if skipped else 'PASS')
    return {
        'version': VERSION,
        'suite_status': gate,
        'release_gate': {'status': gate,
                         'meaning': 'SOFTWARE_VERIFICATION_ONLY',
                         'requirements': ['analytical Kepler accuracy', 'RK4 convergence',
                                          'complete supplied Vallado corpus', 'SGP4 vector comparison',
                                          'Astropy frame roundtrip'],
                         'real_flight_accuracy_validated': False},
        'passed': passed, 'failed': failed, 'skipped': skipped,
        'results': checks,
        'reference_bundle': {**verification, 'source': source_name,
                             'provenance': source.get('provenance', {}),
                             'classification': 'published_software_verification_vectors_not_observations'},
        'environment': {'python': platform.python_version(),
                        'numpy': _package_version('numpy'),
                        'scipy': _package_version('scipy'),
                        'sgp4': _package_version('sgp4'),
                        'astropy': _package_version('astropy')},
        'import_summary': None if parsed is None else {
            key: value for key, value in parsed.items() if key != 'reference'},
        'limitations': [
            'No flight-accuracy claims: published SGP4 states are software-verification outputs.',
            'Vallado coverage threshold checks size, not official authenticity; compare SHA256 pins against a trusted separately recorded source.',
            'Astropy GCRS/ITRS forward/inverse roundtrip is NOT independent geodetic validation.',
            'Absent SGP4 or Astropy is a SKIP, never a PASS.',
            'Error-only propagation cases require additional SGP4 error-code validation beyond vector comparisons.',
        ],
    }


def main() -> int:
    p = argparse.ArgumentParser(description='APOGEE v2.4 fail-closed software-verification audit')
    p.add_argument('--tle', type=Path, help='Original Vallado SGP4-VER.TLE')
    p.add_argument('--vectors', type=Path, help='Original Vallado tcppver.out')
    p.add_argument('--expected-tle-sha256')
    p.add_argument('--expected-vectors-sha256')
    p.add_argument('--steps-per-orbit', type=int, default=240)
    p.add_argument('--orbits', type=int, default=2)
    p.add_argument('--output', type=Path, default=Path('APOGEE-v2.4-audit.json'))
    p.add_argument('--allow-partial', action='store_true',
                   help='Return zero for a partial audit; JSON will still report PARTIAL')
    args = p.parse_args()
    if bool(args.tle) != bool(args.vectors):
        p.error('Provide both --tle and --vectors')
    try:
        report = evaluate(steps_per_orbit=args.steps_per_orbit, orbits=args.orbits,
                          tle_text=args.tle.read_text(encoding='ascii') if args.tle else None,
                          vector_text=args.vectors.read_text(encoding='ascii') if args.vectors else None,
                          expected_tle_sha256=args.expected_tle_sha256,
                          expected_vector_sha256=args.expected_vectors_sha256)
    except (ValueError, OSError, UnicodeError) as exc:
        print(f'APOGEE verification rejected: {exc}', file=sys.stderr)
        return 3
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False) + '\n', encoding='utf-8')
    print(f'APOGEE v2.4: {report["suite_status"]} ({report["passed"]} PASS, '
          f'{report["failed"]} FAIL, {report["skipped"]} SKIP). Audit: {args.output}')
    if report['suite_status'] == 'PASS' or (args.allow_partial and report['suite_status'] == 'PARTIAL'):
        return 0
    return 1 if report['suite_status'] == 'FAIL' else 2


if __name__ == '__main__':
    raise SystemExit(main())
