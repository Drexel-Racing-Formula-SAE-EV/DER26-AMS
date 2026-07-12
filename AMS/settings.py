# Settings for Tracealyzer streaming through STM32CubeIDE/ST-LINK.

GDB_SERVER_PATH = (
    r'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins'
    r'\com.st.stm32cube.ide.mcu.externaltools.stlink-gdb-server.win32_2.2.500.202604010938'
    r'\tools\bin\ST-LINK_gdbserver.exe'
)

STLINK_PROG_DIR = (
    r'C:\ST\STM32CubeIDE_2.2.0\STM32CubeIDE\plugins'
    r'\com.st.stm32cube.ide.mcu.externaltools.cubeprogrammer.win32_2.2.500.202603051304'
    r'\tools\bin'
)

# Tracealyzer connects here.
TRACE_OUTPUT_PORT = '5000'

# CubeIDE remote GDB connection must use this port.
GDB_SERVER_PORT = '60230'

# Internal GDB-server SWO stream port.
GDB_SWO_PORT = '61997'

# CubeIDE SWV configuration must use this port.
IDE_SWO_PORT = '61035'