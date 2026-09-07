@echo off
setlocal
set "CHERE_INVOKING=1"

if not defined DER26_MSYS2_BASH set "DER26_MSYS2_BASH=C:\msys64\usr\bin\bash.exe"
if not exist "%DER26_MSYS2_BASH%" (
    echo ERROR: MSYS2 bash not found at "%DER26_MSYS2_BASH%" 1>&2
    echo Set DER26_MSYS2_BASH to your MSYS2 usr\bin\bash.exe path. 1>&2
    exit /b 2
)

pushd "%~dp0" >nul
"%DER26_MSYS2_BASH%" -lc "export PATH=/ucrt64/bin:/usr/bin:$PATH; make clean all test"
set "RC=%ERRORLEVEL%"
popd >nul
exit /b %RC%
