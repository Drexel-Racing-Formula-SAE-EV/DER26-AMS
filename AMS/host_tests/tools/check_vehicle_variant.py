"""Native configuration/source tests. Synthetic acknowledgements are test-only."""
from pathlib import Path
import subprocess, shlex, xml.etree.ElementTree as E

repo=Path(__file__).resolve().parents[2]
cwd=repo/'host_tests'
dry=subprocess.check_output(['make','-n','vehicle-variant-test'],cwd=cwd,text=True).replace('\\\n',' ')
args=shlex.split(next(s for s in dry.splitlines() if s.startswith('cc ')))
flags=args[:args.index('src/ams_host_test_runner.c')]
flags=[x for x in flags if not x.startswith(('-DAMS_BUILD_PROFILE=','-DAMS_HOST_VEHICLE_VARIANT_TEST='))]
configs=E.parse(repo/'.cproject').findall('.//cconfiguration')
assert len(configs)==4
names=[c.find('.//configuration').get('name') for c in configs]
assert names == [
    'VEHICLE COMMS 1M Debug', 'VEHICLE COMMS 1M Release',
    'VEHICLE RELEASE GATED 1M Debug', 'VEHICLE RELEASE GATED 1M Release',
],names
builders=[b.get('buildPath') for c in configs for b in c.findall('.//builder') if b.get('buildPath')]
assert len(builders)==4 and len(set(builders))==4,builders
assert all('/Vehicle_' in path for path in builders),builders
for i,c in enumerate(configs):
    definitions=[x.get('value') for o in c.findall('.//option')
                 if o.get('superClass','').endswith('.definedsymbols')
                 for x in o.findall('listOptionValue')]
    assert 'DER26_CAN_BITRATE_KBPS=1000' in definitions,definitions
    assert ('AMS_BUILD_PROFILE=4' if i<2 else 'AMS_BUILD_PROFILE=3') in definitions,definitions
    assert not any('VALIDATED=1' in x or 'REVIEWED=1' in x for x in definitions)
assert 'CAN1.Prescaler=3' in next(repo.glob('*.ioc')).read_text()
assert 'CAN1.CalculateBaudRate=1000000' in next(repo.glob('*.ioc')).read_text()
assert E.parse(repo/'.project').findtext('name')=='DER26-AMS-VEHICLE-1M-CLI4'
print('PASS four CubeIDE configurations, identity, 1-Mbit/s runtime/project/Cube timing; no fabricated release evidence')
probe=cwd/'build/vehicle_contract_probe.c'
probe.write_text('''#include "app.h"
_Static_assert(NSMBS==5 && ACCUMULATOR_PHYSICAL_CHAIN_COUNT==5,"five SMBs");
_Static_assert(AMS_ENABLE_APM_2950==0 && AMS_ENABLE_FAN_TASK==0,"installed devices");
_Static_assert(AMS_TESTDAY_C1_EXCLUDED_IC_MASK==0,"drive requires C1");
_Static_assert(AMS_MONITORED_SERIES_CELLS==75,"full drive observation");
_Static_assert(AMS_ENABLE_CLI==0 && AMS_ENABLE_SERVICE_CLI==0,"release CLI disabled");
''')
negative=subprocess.run(flags+['-DAMS_BUILD_PROFILE=3','-fsyntax-only',str(probe)],cwd=cwd,capture_output=True,text=True)
assert negative.returncode!=0 and 'Vehicle profile requires' in negative.stderr
(cwd/'build/vehicle_missing_evidence.log').write_text(negative.stderr)
print('PASS drive release rejects missing validation evidence')
gate_dry=subprocess.check_output(['make','-n','profile-gates'],cwd=cwd,text=True).replace('\\\n',' ')
gate=shlex.split(next(s for s in gate_dry.splitlines() if s.startswith('cc ') and '-DAMS_EXPECTED_PROFILE=3' in s))
evidence=[x for x in gate if x.startswith('-D') and not x.startswith(('-DNSMBS=','-DACCUMULATOR_PHYSICAL_CHAIN_COUNT=','-DAMS_ENABLE_APM_2950=','-DAMS_EXPECTED_PROFILE=','-DSTM32','-DUSE_','-DAMS_HOST_TEST='))]
sources=['Core/Src/app.c','Core/Src/tasks/error_task.c','Core/Src/tasks/canbus_task.c',
         'Core/Src/tasks/adbms_task.c','Core/Src/ext_drivers/canbus.c','Core/Src/main.c']
positive=subprocess.run(flags+evidence+['-fsyntax-only',str(probe)]+[str(repo/s) for s in sources],cwd=cwd,capture_output=True,text=True)
if positive.returncode: print(positive.stdout+positive.stderr)
positive.check_returncode()
print('PASS native drive-profile sources and full-75-cell/no-CLI/five-device contract with synthetic test-only evidence')
