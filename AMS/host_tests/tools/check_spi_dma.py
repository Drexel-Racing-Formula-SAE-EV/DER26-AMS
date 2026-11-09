"""Native syntax and wire-rate SIL; not an ARM link or DMA electrical model."""
from pathlib import Path
import subprocess, shlex
host=Path(__file__).resolve().parents[1]
out=host.parent/'validation'
dry=subprocess.check_output(['make','-n','adbms-wake-policy-test'],cwd=host,text=True).replace('\\\n',' ')
command=next(shlex.split(x) for x in dry.splitlines() if x.startswith('cc '))
flags=command[:command.index('production/adbms_wake_policy_test.c')]
sources=['ext_drivers/accumulator.c','diagnostics/ams_perf.c','ext_drivers/ams_spi6_transport.c','ext_drivers/adbms6830.c','main.c','stm32f7xx_it.c','tasks/cli_task.c']
for div in (256,128,64):
    args=flags+[f'-DAMS_ADBMS_SPI_PRESCALER_DIV={div}','-DAMS_ADBMS_SPI_DMA=1','-D__ARM_ARCH_7EM__=1','-fsyntax-only']+['../Core/Src/'+x for x in sources]
    subprocess.run(args,cwd=host,check=True)
    cmd=[x.replace('production/adbms_wake_policy_test.c','sil/acquisition_sil.c') for x in command]
    binary=host/'build'/f'spi_rate_sil_{div}'
    cmd[cmd.index('-o')+1]=str(binary)
    cmd += [f'-DAMS_ADBMS_SPI_PRESCALER_DIV={div}','-DSIL_CANDIDATE=1']
    subprocess.run(cmd,cwd=host,check=True)
    run=subprocess.run([str(binary)],cwd=host,text=True,capture_output=True)
    (out/f'SPI_RATE_SIL_DIV{div}.csv').write_text(run.stdout)
    (out/f'SPI_RATE_SIL_DIV{div}.stderr.txt').write_text(run.stderr)
    run.check_returncode()
print('PASS DMA target-source native syntax and production driver SIL at /256, /128 and /64')
