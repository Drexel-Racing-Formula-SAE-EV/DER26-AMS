"""Prevent CubeMX/source drift on schematic-derived RTM/IMD/current pins."""
from pathlib import Path
import re
r=Path(__file__).resolve().parents[2]
h=(r/'Core/Inc/main.h').read_text()
ioc=dict(line.split('=',1) for line in (r/'DER26-AMS.ioc').read_text().splitlines() if '=' in line)
for label,port,pin in [('HV_ACTIVE','GPIOF',9),('AIR_CTRL','GPIOF',11),('IMD_STAT','GPIOC',5),('IMD_PWM','GPIOA',5),('C_SNS_H','GPIOA',3),('C_SNS_L','GPIOC',0)]:
 assert re.search(r'#define\s+'+label+r'_Pin\s+GPIO_PIN_'+str(pin)+r'\b',h),label
 assert re.search(r'#define\s+'+label+r'_GPIO_Port\s+'+port+r'\b',h),label
 assert ioc['P'+port[-1]+str(pin)+'.GPIO_Label']==label,label
assert ioc['PF9.Signal']=='GPIO_Input'
c=(r/'Core/Src/main.c').read_text()
# Check every GPIOF initialization, with symbolic pins resolved from main.h.
mask='';mode='';digital=False
for statement in c[c.index('static void MX_GPIO_Init(void)\n{'):].split(';'):
 if 'GPIO_InitStruct.Pin =' in statement: mask=statement.split('GPIO_InitStruct.Pin =')[-1]
 if 'GPIO_InitStruct.Mode =' in statement: mode=statement.split('GPIO_InitStruct.Mode =')[-1].strip()
 if re.search(r'HAL_GPIO_Init\((?:GPIOF|HV_ACTIVE_GPIO_Port),',statement):
  if 'GPIO_PIN_9' in mask or 'HV_ACTIVE_Pin' in mask:
   assert mode=='GPIO_MODE_INPUT', 'PF9 must never be reinitialized analog/output'
   digital=True
assert digital,'PF9 has no digital-input initialization'
print('PASS schematic pin mapping and RTM PF9 digital-input initialization/CubeMX metadata')
