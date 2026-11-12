"""Parse target branches and kernel trace hooks with native GCC, not an ARM build.
The host newlib shim supplies declarations only. No evidence gates are bypassed.
"""
from pathlib import Path
import subprocess, shlex
h=Path(__file__).resolve().parents[1]
dry=subprocess.check_output(['make','-n','adbms-wake-policy-test'],cwd=h,text=True).replace('\\\n',' ')
command=next(shlex.split(x) for x in dry.splitlines() if x.startswith('cc '))
flags=[x for x in command[:command.index('production/adbms_wake_policy_test.c')] if x!='-DAMS_HOST_TEST=1']
sources=['../Middlewares/Third_Party/FreeRTOS/Source/tasks.c',
         '../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c']
sources += ['../Core/Src/'+x for x in ['diagnostics/ams_perf.c','main.c',
    'tasks/estimator_task.c','estimator/ams_thermal_observer.c','tasks/adbms_task.c','tasks/cli_task.c','ext_drivers/accumulator.c','ext_drivers/adbms6830.c','ext_drivers/cli.c','ext_drivers/current_sensor.c','stm32f7xx_it.c','stm32f7xx_hal_msp.c']]
sources += ['../Drivers/STM32F7xx_HAL_Driver/Src/'+x for x in ['stm32f7xx_hal_uart.c','stm32f7xx_hal_uart_ex.c','stm32f7xx_hal_dma.c']]
args=flags+['-D__ARM_ARCH_7EM__=1','-D__VFP_FP__=1',
    '-DAMS_ENABLE_ADBMS_THERM_OPEN_WIRE_DIAG=1','-DAMS_ENABLE_ADBMS_AUX2_REDUNDANCY=1',
    '-DAMS_CLI_UART_DMA=1','-DAMS_CURRENT_PAIRED_START=1','-DAMS_CURRENT_TIMER_TRIGGER=1','-DAMS_ADBMS_READY_POLL=1','-DAMS_BENCHMARK_ICACHE=1','-DAMS_BENCHMARK_DCACHE=1','-fsyntax-only']+sources
run=subprocess.run(args,cwd=h,text=True,capture_output=True)
(h.parent/'validation/STAGE3_TARGET_SYNTAX.txt').write_text(
    'Native target-branch syntax only; newlib shim declarations; no ARM link\n'+run.stdout+run.stderr+'\nexit='+str(run.returncode))
if run.returncode: print(run.stdout+run.stderr)
run.check_returncode()
print('PASS target-branch native syntax: kernel/cache, driver/task/pipeline, UART DMA IRQs, paired ADC and AUX polling')
