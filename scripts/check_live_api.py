"""Diagnose live APOGEE browser/API failure. Makes only read/preflight requests.

Usage: python scripts/check_live_api.py
"""
import urllib.error
import urllib.request

BASE = 'https://apogee-physics-api.onrender.com'
ORIGIN = 'https://chesswiz263-hub.github.io'


def probe(label, path, method='GET', headers=None):
    url = BASE + path
    req = urllib.request.Request(url, method=method, headers=headers or {})
    try:
        with urllib.request.urlopen(req, timeout=35) as response:
            print(label, 'HTTP', response.status)
            print('  allow-origin:', response.headers.get('Access-Control-Allow-Origin', '(none)'))
            print('  allow-methods:', response.headers.get('Access-Control-Allow-Methods', '(none)'))
            if method == 'GET':
                print('  body:', response.read(700).decode('utf8', errors='replace'))
    except urllib.error.HTTPError as error:
        print(label, 'HTTP', error.code)
        print('  allow-origin:', error.headers.get('Access-Control-Allow-Origin', '(none)'))
        print('  allow-methods:', error.headers.get('Access-Control-Allow-Methods', '(none)'))
        print('  body:', error.read(400).decode('utf8', errors='replace'))
    except Exception as error:
        print(label, f'{type(error).__name__}: {error}')


if __name__ == '__main__':
    probe('Deployed v2.5 check', '/api/v25/capabilities')
    probe('Existing endpoint preflight', '/api/navigation/integrity', 'OPTIONS', {
        'Origin': ORIGIN,
        'Access-Control-Request-Method': 'POST',
        'Access-Control-Request-Headers': 'content-type',
    })
