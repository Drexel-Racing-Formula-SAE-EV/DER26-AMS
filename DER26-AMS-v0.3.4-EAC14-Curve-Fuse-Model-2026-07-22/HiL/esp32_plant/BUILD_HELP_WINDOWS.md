# Windows ESP-IDF build help

Use an ESP-IDF shell, not a plain PowerShell/CMD shell.

Recommended:

1. Open **ESP-IDF PowerShell** from the Start Menu.
2. `cd` into this folder.
3. Build:

```powershell
idf.py set-target esp32
idf.py fullclean
idf.py build
```

Flash and monitor:

```powershell
idf.py -p COMx flash monitor
```

If `idf.py` is not recognized, activate ESP-IDF manually:

```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass
cd $env:USERPROFILE\esp\esp-idf
.\export.ps1
cd <path-to-repo>\hil\esp32_plant
idf.py build
```

Do not commit the `build/` folder, `.metadata/`, Eclipse workspace files, generated `.bin/.elf/.map`, or `sdkconfig.old`.
