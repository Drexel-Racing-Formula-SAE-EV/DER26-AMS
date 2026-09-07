#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
AMS_DIR="$ROOT_DIR/AMS"
AMS_BUILD_TYPE="${AMS_BUILD_TYPE:-Debug}"
case "$AMS_BUILD_TYPE" in
  Debug) MODE_FLAGS=(-O0 -g3 -DDEBUG) ;;
  # Match the CubeIDE Release configuration: size optimization and no debug
  # information. Build type is deliberately independent of the authority
  # profile selected below.
  Release) MODE_FLAGS=(-Os -g0) ;;
  *) echo "AMS_BUILD_TYPE must be Debug or Release" >&2; exit 2 ;;
esac

# The checked-in CubeIDE configurations use BENCH_VALIDATION (5). Keep the
# headless path behaviorally identical by default, but make the selected
# authority profile explicit in both compiler flags and provenance. Callers
# may select another valid profile intentionally (for example VEHICLE=3 once
# all compile-time release gates are supplied).
AMS_BUILD_PROFILE="${AMS_BUILD_PROFILE:-5}"
case "$AMS_BUILD_PROFILE" in
  1) AMS_BUILD_PROFILE_NAME="bench" ;;
  2) AMS_BUILD_PROFILE_NAME="hil" ;;
  3) AMS_BUILD_PROFILE_NAME="vehicle" ;;
  4) AMS_BUILD_PROFILE_NAME="testday" ;;
  5) AMS_BUILD_PROFILE_NAME="bench_validation" ;;
  *) echo "AMS_BUILD_PROFILE must be one of 1,2,3,4,5" >&2; exit 2 ;;
esac

AMS_BENCH_VALIDATION_SINGLE_SMB="${AMS_BENCH_VALIDATION_SINGLE_SMB:-0}"
case "$AMS_BENCH_VALIDATION_SINGLE_SMB" in
  0|1) ;;
  *) echo "AMS_BENCH_VALIDATION_SINGLE_SMB must be 0 or 1" >&2; exit 2 ;;
esac
# Explicit opt-in, independent of optimization and authority profile.
AMS_WARNINGS_AS_ERRORS="${AMS_WARNINGS_AS_ERRORS:-0}"
case "$AMS_WARNINGS_AS_ERRORS" in
  0|1) ;;
  *) echo "AMS_WARNINGS_AS_ERRORS must be 0 or 1" >&2; exit 2 ;;
esac
if [[ "$AMS_BUILD_TYPE" == "Release" && -z "${SOURCE_DATE_EPOCH:-}" ]]; then
  echo "Release builds require SOURCE_DATE_EPOCH for deterministic build metadata" >&2
  exit 2
fi
if [[ -n "${SOURCE_DATE_EPOCH:-}" ]]; then
  [[ "$SOURCE_DATE_EPOCH" =~ ^[0-9]+$ ]] || { echo "Invalid SOURCE_DATE_EPOCH" >&2; exit 2; }
  export SOURCE_DATE_EPOCH
fi
for tool in arm-none-eabi-gcc arm-none-eabi-size arm-none-eabi-objcopy arm-none-eabi-objdump sha256sum awk find sort; do
  command -v "$tool" >/dev/null || { echo "Missing required tool: $tool" >&2; exit 2; }
done
command -v python3 >/dev/null || { echo "Missing required tool: python3" >&2; exit 2; }
python3 "$AMS_DIR/host_tests/tools/check_freertos_provenance.py"

# Hash the actual compile/link input tree, independent of archive filename or
# working-directory metadata. This remains available even when the release ZIP
# is detached from Git, and makes the provenance record identify immutable
# source/configuration bytes rather than only a human version string.
SOURCE_INPUT_TREE_SHA256="$({
  find "$AMS_DIR/Core" "$AMS_DIR/Drivers" "$AMS_DIR/Middlewares" -type f -print0
  printf '%s\0' "$AMS_DIR/STM32F767ZITX_FLASH.ld" "$AMS_DIR/DER26-AMS.ioc" "$AMS_DIR/.cproject"
} | LC_ALL=C sort -z | while IFS= read -r -d '' file; do
  rel="${file#$ROOT_DIR/}"
  hash="$(sha256sum "$file" | awk '{print $1}')"
  printf '%s  %s\n' "$hash" "$rel"
done | sha256sum | awk '{print $1}')"

# Preserve previous ELF/MAP evidence, including failed-build diagnostics.
mkdir -p "$AMS_DIR/build"
BUILD_DIR="$(mktemp -d "$AMS_DIR/build/${AMS_BUILD_TYPE}.XXXXXX")"
echo "Build artifacts: $BUILD_DIR"

{
  echo "build_type=$AMS_BUILD_TYPE"
  echo "build_profile=$AMS_BUILD_PROFILE"
  echo "build_profile_name=$AMS_BUILD_PROFILE_NAME"
  echo "bench_validation_single_smb=$AMS_BENCH_VALIDATION_SINGLE_SMB"
  echo "mode_flags=${MODE_FLAGS[*]}"
  echo "warnings_as_errors=$AMS_WARNINGS_AS_ERRORS"
  echo "source_date_epoch=${SOURCE_DATE_EPOCH:-unset}"
  echo "source_input_tree_sha256=$SOURCE_INPUT_TREE_SHA256"
  echo "compiler=$(arm-none-eabi-gcc --version | head -n 1)"
  echo "firmware_revision=$(awk '/^#define AMS_VERSION_MAJOR /{maj=$3} /^#define AMS_VERSION_MINOR /{min=$3} /^#define AMS_VERSION_PATCH /{pat=$3} /^#define AMS_RELEASE_DATE /{gsub(/"/, "", $3); date=$3} END{printf "DER26-AMS-v%s.%s.%s-%s", maj,min,pat,date}' "$AMS_DIR/Core/Inc/ams_version.h")"
  python3 "$AMS_DIR/host_tests/tools/check_freertos_provenance.py" --manifest
  echo "linker_script_sha256=$(sha256sum "$AMS_DIR/STM32F767ZITX_FLASH.ld" | awk '{print $1}')"
  echo "cubemx_ioc_sha256=$(sha256sum "$AMS_DIR/DER26-AMS.ioc" | awk '{print $1}')"
  echo "cubeide_cproject_sha256=$(sha256sum "$AMS_DIR/.cproject" | awk '{print $1}')"
} > "$BUILD_DIR/BUILD_PROVENANCE.txt"

CFLAGS=(
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -std=gnu11
  "${MODE_FLAGS[@]}"
  -fstack-usage
  "-ffile-prefix-map=$ROOT_DIR=."
  "-fdebug-prefix-map=$ROOT_DIR=."
  -ffunction-sections
  -fdata-sections
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wno-missing-field-initializers
  -DUSE_HAL_DRIVER
  -DSTM32F767xx
  -DAMS_BUILD_PROFILE="$AMS_BUILD_PROFILE"
  -DAMS_BENCH_VALIDATION_SINGLE_SMB="$AMS_BENCH_VALIDATION_SINGLE_SMB"
  -DDER26_CAN_BITRATE_KBPS=1000
)

INCLUDES=(
  -I"$AMS_DIR/Core/Inc"
  -I"$AMS_DIR/Core/Inc/ext_drivers"
  -I"$AMS_DIR/Core/Inc/tasks"
  -I"$AMS_DIR/Core/Inc/estimator"
  -I"$AMS_DIR/Drivers/STM32F7xx_HAL_Driver/Inc"
  -I"$AMS_DIR/Drivers/STM32F7xx_HAL_Driver/Inc/Legacy"
  -I"$AMS_DIR/Drivers/CMSIS/Device/ST/STM32F7xx/Include"
  -I"$AMS_DIR/Drivers/CMSIS/Include"
  -I"$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/include"
  -I"$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2"
  -I"$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1"
)

SOURCES=()

while IFS= read -r src; do
  SOURCES+=("$src")
done < <(find "$AMS_DIR/Core/Src" -name "*.c" -print | sort)

while IFS= read -r src; do
  SOURCES+=("$src")
done < <(find "$AMS_DIR/Drivers/STM32F7xx_HAL_Driver/Src" -name "*.c" -print | sort)

FREERTOS_SOURCES=(
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/list.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/queue.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/timers.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS_V2/cmsis_os2.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM7/r0p1/port.c"
  "$AMS_DIR/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
)

for src in "${FREERTOS_SOURCES[@]}"; do
  if [[ -f "$src" ]]; then
    SOURCES+=("$src")
  else
    echo "Missing expected FreeRTOS source: $src"
    exit 1
  fi
done

OBJECTS=()

echo "Compiling ${#SOURCES[@]} C files..."

for src in "${SOURCES[@]}"; do
  rel="${src#$AMS_DIR/}"
  obj="$BUILD_DIR/${rel//\//_}.o"
  PROJECT_FLAGS=()
  if [[ "$AMS_WARNINGS_AS_ERRORS" == 1 && "$src" == "$AMS_DIR/Core/Src/"* ]]; then
    PROJECT_FLAGS=(-Werror)
  fi
  arm-none-eabi-gcc "${CFLAGS[@]}" "${PROJECT_FLAGS[@]}" "${INCLUDES[@]}" -c "$src" -o "$obj"
  OBJECTS+=("$obj")
done

STARTUP="$AMS_DIR/Core/Startup/startup_stm32f767zitx.s"
STARTUP_OBJ="$BUILD_DIR/startup_stm32f767zitx.o"

arm-none-eabi-gcc "${CFLAGS[@]}" "${INCLUDES[@]}" -x assembler-with-cpp -c "$STARTUP" -o "$STARTUP_OBJ"
OBJECTS+=("$STARTUP_OBJ")

LDFLAGS=(
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -T"$AMS_DIR/STM32F767ZITX_FLASH.ld"
  -Wl,-Map="$BUILD_DIR/DER26-AMS.map"
  -Wl,--gc-sections
  -specs=nano.specs
  -specs=nosys.specs
  -lc
  -lm
  -lnosys
)

echo "Linking DER26-AMS.elf..."

arm-none-eabi-gcc "${OBJECTS[@]}" "${LDFLAGS[@]}" -o "$BUILD_DIR/DER26-AMS.elf"

arm-none-eabi-size "$BUILD_DIR/DER26-AMS.elf"
arm-none-eabi-objcopy -O ihex "$BUILD_DIR/DER26-AMS.elf" "$BUILD_DIR/DER26-AMS.hex"
arm-none-eabi-objcopy -O binary "$BUILD_DIR/DER26-AMS.elf" "$BUILD_DIR/DER26-AMS.bin"
arm-none-eabi-objdump -h -S "$BUILD_DIR/DER26-AMS.elf" > "$BUILD_DIR/DER26-AMS.list"

(
  cd "$BUILD_DIR"
  sha256sum DER26-AMS.elf DER26-AMS.map DER26-AMS.hex DER26-AMS.bin > SHA256SUMS.txt
)

# Keep the unique build directory as the immutable evidence location, while
# publishing stable links for CI/reporting steps that consume AMS/build/*.
# This avoids coupling downstream jobs to mktemp's Debug.* / Release.* suffix.
build_leaf="$(basename "$BUILD_DIR")"
for artifact in DER26-AMS.elf DER26-AMS.map DER26-AMS.hex DER26-AMS.bin DER26-AMS.list BUILD_PROVENANCE.txt SHA256SUMS.txt; do
  ln -sfn "$build_leaf/$artifact" "$AMS_DIR/build/$artifact"
done

echo "Headless STM32 ARM-GCC build complete."
