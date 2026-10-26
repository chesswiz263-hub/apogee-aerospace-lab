"""Reproduce the v1.3 delayed-navigation coast and powered-burn examples."""
import json
from pathlib import Path
from .integrated_nav import IntegratedNavConfig,simulate_integrated_navigation


def run():
    reports={}
    for label,seconds in [('coast',0),('powered_burn',120)]:
        result=simulate_integrated_navigation(IntegratedNavConfig(burn_duration_s=seconds))
        nav=result['integrated_navigation']
        reports[label]={
            'final_position_error_m':nav['final_position_error_m'],
            'final_position_1sigma_rss_m':nav['final_position_1sigma_rss_m'],
            'final_velocity_error_m_s':nav['final_velocity_error_m_s'],
            'final_attitude_estimation_error_deg':nav['final_attitude_estimation_error_deg'],
            'packets_received':nav['received_at_finish'],
            'packets_generated':nav['packets_generated'],
            'propellant_consumed_kg':result['metrics']['propellant_used_kg']}
    path=Path(__file__).resolve().parents[1]/'docs'/'v13-benchmark.json'
    path.write_text(json.dumps({'version':'1.3.0','research_only':True,'scenarios':reports},indent=2)+'\n')
    print(path)
    print(json.dumps(reports,indent=2))
    return reports


if __name__=='__main__':run()
