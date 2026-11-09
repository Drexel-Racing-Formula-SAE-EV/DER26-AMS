#!/usr/bin/env python3
"""Run real driver/accumulator against deterministic HAL/time fault fixtures.
No RTOS, analogue plant, CPU-time or electrical qualification is implied.
"""
import argparse,csv,io,json,shlex,subprocess
from pathlib import Path
ap=argparse.ArgumentParser();ap.add_argument('--baseline',type=Path);args=ap.parse_args()
host=Path(__file__).resolve().parents[1];out=host.parent/'validation';out.mkdir(exist_ok=True)
results={}
for name,root in [('candidate',host.parent)]+([('baseline',args.baseline.resolve())] if args.baseline else []):
    text=subprocess.check_output(['make','-n',f'AMS_ROOT={root}','adbms-wake-policy-test'],cwd=host,text=True).replace('\\\n',' ')
    command=next(shlex.split(line) for line in text.splitlines() if line.startswith('cc ') or line.startswith('gcc '))
    binary=host/'build'/f'acquisition_sil_{name}';binary.parent.mkdir(exist_ok=True)
    command=[v.replace('production/adbms_wake_policy_test.c','sil/acquisition_sil.c') for v in command]
    command[command.index('-o')+1]=str(binary)
    if name=="candidate": command.append("-DSIL_CANDIDATE=1")
    subprocess.run(command,cwd=host,check=True)
    run=subprocess.run([str(binary)],cwd=host,text=True,capture_output=True)
    (out/f'ACQUISITION_SIL_{name.upper()}.csv').write_text(run.stdout)
    (out/f'ACQUISITION_SIL_{name.upper()}.stderr.txt').write_text(run.stderr)
    if run.returncode:raise SystemExit(f'{name}: assertions failed; see validation logs')
    results[name]=list(csv.DictReader(io.StringIO(run.stdout)))
print(json.dumps(results,indent=2))
