#!/usr/bin/env bash
set -euo pipefail

required_paths=(
  "AMS/Core"
  "AMS/Drivers"
  "AMS/Middlewares"
  "AMS/host_tests/Makefile"
  "AMS/Core/Inc/app.h"
  "AMS/Core/Src/app.c"
  "AMS/Core/Inc/estimator/ams_soc_ekf.h"
  "AMS/Core/Src/estimator/ams_soc_ekf.c"
  "AMS/Core/Src/estimator/ams_estimator_lut.c"
  "AMS/STM32F767ZITX_FLASH.ld"
  "AMS/Core/Startup/startup_stm32f767zitx.s"
)

for path in "${required_paths[@]}"; do
  if [[ ! -e "$path" ]]; then
    echo "Missing required path: $path"
    exit 1
  fi
done

echo "Project structure check passed."
