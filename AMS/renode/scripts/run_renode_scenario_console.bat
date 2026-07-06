@echo off
setlocal

set ROOT=%~dp0..\..\..
for %%I in ("%ROOT%") do set ROOT=%%~fI

set ELF=%ROOT%\AMS\build\DER26-AMS.elf
set SCRIPT=%ROOT%\AMS\renode\scripts\ams_f767_cli_scenario_console.resc

if not exist "%ELF%" (
  echo ERROR: missing ELF:
  echo   %ELF%
  echo.
  echo Build from MSYS2 first:
  echo   AMS_RENODE=1 ./ci/stm32/build_ams_headless_gcc.sh
  pause
  exit /b 2
)

echo Launching Renode with:
echo   %SCRIPT%
echo.
echo Paste commands from:
echo   %ROOT%\AMS\renode\scripts\cli_scenario_commands.txt
echo.

renode --console -e "include @%SCRIPT%"
set STATUS=%ERRORLEVEL%

echo.
echo Renode exited with code %STATUS%
pause
exit /b %STATUS%
