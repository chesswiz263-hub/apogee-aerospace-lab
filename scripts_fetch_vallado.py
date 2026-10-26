"""Explicitly fetch public python-sgp4 repo Vallado verification data.

An HTTPS retrieval is a convenience, not proof of upstream authenticity. The
user should compare printed SHA256 with independently trusted, pinned values.
"""
from __future__ import annotations
import argparse
import hashlib
from pathlib import Path
from urllib.request import Request, urlopen

BASE='https://raw.githubusercontent.com/brandon-rhodes/python-sgp4/master/sgp4/'
FILES=('SGP4-VER.TLE','tcppver.out')

def main():
    p=argparse.ArgumentParser(description='Fetch Vallado software-verification inputs (explicit network access)')
    p.add_argument('--destination',default='reference_inputs',type=Path)
    p.add_argument('--confirm-download',required=True,action='store_true')
    args=p.parse_args()
    args.destination.mkdir(exist_ok=True,parents=True)
    for name in FILES:
        req=Request(BASE+name,headers={'User-Agent':'APOGEE-v2.4-reproducibility'})
        with urlopen(req,timeout=20) as resp:
            data=resp.read(8_000_001)
        if not 100<len(data)<=8_000_000:
            raise ValueError(f'Unexpected input size: {name}')
        (args.destination/name).write_bytes(data)
        print(f'{name}: sha256={hashlib.sha256(data).hexdigest()} bytes={len(data)}')
    print('Downloaded inputs are NOT authenticated by HTTPS alone. Independently pin SHA256 hashes.')
    print(f'python -m apogee.v24_validation --tle "{args.destination/FILES[0]}" --vectors "{args.destination/FILES[1]}" --output audit.json')

if __name__=='__main__':main()
