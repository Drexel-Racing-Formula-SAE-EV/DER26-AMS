"""Check standalone charger project identity/timing and forbidden overrides."""
from pathlib import Path
import subprocess
import xml.etree.ElementTree as E

root = Path(__file__).resolve().parents[2]
name = 'DER26-AMS-75S-74OBS-DIAG-250'
assert E.parse(root / '.project').getroot().findtext('name') == name
tree = E.parse(root / '.cproject')
configs = [c for c in tree.findall('.//configuration') if c.get('artifactExtension') == 'elf']
assert [c.get('name') for c in configs] == ['CHARGER LV 250k Debug', 'CHARGER LV 250k Release', '75S 74OBS CHARGE-REVIEW 250k Debug', '75S 74OBS CHARGE-REVIEW 250k Release']
for c in configs:
    defs = [v.get('value') for o in c.findall('.//option')
            if 'definedsymbols' in o.get('superClass', '') for v in o.findall('listOptionValue')]
    expected_profile = 'AMS_BUILD_PROFILE=6' if '75S 74OBS CHARGE-REVIEW' in c.get('name') else 'AMS_BUILD_PROFILE=4'
    assert expected_profile in defs
    assert 'DER26_CAN_BITRATE_KBPS=250' in defs
    assert 'AMS_ENABLE_TUNING_CAN=0' in defs
    assert not any('BITRATE_KBPS=1000' in d for d in defs)
subprocess.run(['python3', str(root / 'host_tests/tools/check_can_clock_contract.py')], check=True)
includes = ['-I' + str(p) for p in [root/'Core/Inc', root/'host_tests/include']]
source = '#include "ams_build_profile.h"\nint main(void) {return 0;}\n'
ok = subprocess.run(['gcc', '-x', 'c', '-fsyntax-only', *includes, '-'], input=source, text=True, capture_output=True)
assert ok.returncode == 0, ok.stderr
for override, expected in [('-DDER26_CAN_BITRATE_KBPS=1000', 'requires 250'),
                           ('-DAMS_BUILD_PROFILE=3', 'requires TESTDAY CAN-test or CHARGE_REVIEW'),
                           ('-DAMS_ENABLE_TUNING_CAN=1', 'does not permit tuning'),
                           ('-DAMS_ENABLE_MISSION_CAN=1', 'does not accept ECU')]:
    bad = subprocess.run(['gcc', '-x', 'c', '-fsyntax-only', *includes, override, '-'],
                         input=source, text=True, capture_output=True)
    assert bad.returncode != 0 and expected in bad.stderr, (override, bad.stderr)
print('PASS charger project identity, Debug/Release flags, source defaults and forbidden overrides')
