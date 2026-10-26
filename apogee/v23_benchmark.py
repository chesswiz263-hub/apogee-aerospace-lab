"""Command-line verification report generator for APOGEE v2.3.

Examples:
 python -m apogee.v23_benchmark --output validation_report.json
 python -m apogee.v23_benchmark --tle SGP4-VER.TLE --vectors tcppver.out --output full_vallado.json
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
from .v23_validation import validation_report, parse_vallado_raw, compare_many


def main() -> None:
    parser=argparse.ArgumentParser(description='Reproducible APOGEE software-verification report')
    parser.add_argument('--tle',type=Path,help='Published SGP4-VER.TLE file')
    parser.add_argument('--vectors',type=Path,help='Published tcppver.out file')
    parser.add_argument('--steps-per-orbit',type=int,default=240)
    parser.add_argument('--orbits',type=int,default=2)
    parser.add_argument('--output',type=Path,default=Path('APOGEE-v2.3-audit.json'))
    args=parser.parse_args()
    if bool(args.tle)!=bool(args.vectors):parser.error('--tle and --vectors must be given together')
    if args.tle:
        parsed=parse_vallado_raw(args.tle.read_text(encoding='ascii'),args.vectors.read_text(encoding='ascii'))
        report={'version':'2.3.0','import_summary':{k:v for k,v in parsed.items() if k!='reference'},
                 'reference_fingerprints':parsed['reference']['provenance'],
                 'comparison':compare_many(parsed['reference'])}
    else:
        report=validation_report(steps_per_orbit=args.steps_per_orbit,orbits=args.orbits)
    args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(f"Report saved: {args.output} · status: {report.get('suite_status',report.get('comparison',{}).get('status'))}")

if __name__=='__main__': main()
