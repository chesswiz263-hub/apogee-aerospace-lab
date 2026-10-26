"""Reproducible WGS84/ITRS geometric range/rate residual fixture."""
import json
import sys
from pathlib import Path
import numpy as np
from .real_tracking import wgs84_station_itrs,topocentric_observation,compare_records


def benchmark():
    station={'latitude_deg':45.,'longitude_deg':-120.,'height_m':100.}
    site=wgs84_station_itrs(station)
    # Synthetic epoch-tagged Cartesian ITRS vectors, not observed satellite data.
    y0=np.r_[site+np.array([350.,240.,250.]),[.8,-.4,1.2]]
    records=[]
    for i,t in enumerate([0,10,20,30,40,50]):
        state=y0.copy();state[:3]+=t*y0[3:]
        geometry=topocentric_observation(state,station)
        record={'epoch_utc':f'2026-10-25T12:00:{t:02d}Z',
                'station':station,'frame':'ITRS',
                'state':{'position_km':state[:3].tolist(),'velocity_km_s':state[3:].tolist()},
                'observed':{'range_m':geometry['range_m']+2.*(-1)**i,
                            'range_rate_m_s':geometry['range_rate_m_s']+.01*(-1)**i}}
        records.append(record)
    payload={'reference_id':'APOGEE-v21-SYNTHETIC-ITRS-EXAMPLE',
             'provenance':'synthetic','records':records}
    report=compare_records(records,payload['provenance'],payload['reference_id'])
    return payload,report


def main():
    payload,report=benchmark()
    if len(sys.argv)>1:
        target=Path(sys.argv[1]);target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(payload,indent=2)+'\n')
    if len(sys.argv)>2:
        target=Path(sys.argv[2]);target.parent.mkdir(parents=True,exist_ok=True)
        target.write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'sample_count':report['sample_count'],
                      **report['statistics'],'validation_scope':report['validation_scope']},indent=2))

if __name__=='__main__':main()
