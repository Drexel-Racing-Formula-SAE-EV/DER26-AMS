#!/usr/bin/env bash
set -euo pipefail
ROOT="${1:-..}"
missing=0
for p in \
  Core/Inc/app.h \
  Core/Src/tasks/canbus_task.c \
  Core/Src/tasks/adbms_task.c \
  Core/Src/tasks/error_task.c \
  Core/Src/ext_drivers/accumulator.c \
  Core/Src/ext_drivers/canbus.c \
  Core/Src/ext_drivers/charger.c; do
  if [[ ! -e "$ROOT/$p" ]]; then
    echo "missing: $ROOT/$p"
    missing=1
  fi
done
exit "$missing"
