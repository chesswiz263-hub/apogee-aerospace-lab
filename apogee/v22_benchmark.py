"""Run v2.2 repeatable independent benchmark and export JSON report."""
import json
import sys
from pathlib import Path
from .independent_validation import suite_report

def main():
    dest=Path(sys.argv[1]) if len(sys.argv)>1 else Path('v22_validation_report.json')
    result=suite_report()
    dest.parent.mkdir(parents=True,exist_ok=True)
    dest.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n',encoding='utf8')
    print(json.dumps({k:result[k] for k in ('suite_status','passed','failed','skipped')},indent=2))
    print('Report:',dest)
    return 1 if result['failed'] else 0

if __name__=='__main__':raise SystemExit(main())
