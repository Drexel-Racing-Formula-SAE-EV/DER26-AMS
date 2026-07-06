#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(pwd)"
AMS_DIR="$ROOT_DIR/AMS"
BUILD_DIR="$AMS_DIR/build"

rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

AMS_RENODE_VALUE="${AMS_RENODE:-0}"
AMS_RENODE_FAKE_ADBMS_VALUE="${AMS_RENODE_FAKE_ADBMS:-$AMS_RENODE_VALUE}"
AMS_RENODE_FAKE_CURRENT_VALUE="${AMS_RENODE_FAKE_CURRENT:-$AMS_RENODE_VALUE}"
AMS_RENODE_FAKE_CHARGER_VALUE="${AMS_RENODE_FAKE_CHARGER:-$AMS_RENODE_VALUE}"
AMS_RENODE_FAKE_TEMP_VALUE="${AMS_RENODE_FAKE_TEMP:-$AMS_RENODE_VALUE}"
AMS_RENODE_CAN_CAPTURE_VALUE="${AMS_RENODE_CAN_CAPTURE:-$AMS_RENODE_VALUE}"

CFLAGS=(
  -mcpu=cortex-m7
  -mthumb
  -mfpu=fpv5-d16
  -mfloat-abi=hard
  -std=gnu11
  -O0
  -g3
  -ffunction-sections
  -fdata-sections
  -Wall
  -Wextra
  -Wno-unused-parameter
  -Wno-missing-field-initializers
  -DDEBUG
  -DUSE_HAL_DRIVER
  -DSTM32F767xx
  -DAMS_RENODE="$AMS_RENODE_VALUE"
  -DAMS_RENODE_FAKE_ADBMS="$AMS_RENODE_FAKE_ADBMS_VALUE"
  -DAMS_RENODE_FAKE_CURRENT="$AMS_RENODE_FAKE_CURRENT_VALUE"
  -DAMS_RENODE_FAKE_CHARGER="$AMS_RENODE_FAKE_CHARGER_VALUE"
  -DAMS_RENODE_FAKE_TEMP="$AMS_RENODE_FAKE_TEMP_VALUE"
  -DAMS_RENODE_CAN_CAPTURE="$AMS_RENODE_CAN_CAPTURE_VALUE"
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
  arm-none-eabi-gcc "${CFLAGS[@]}" "${INCLUDES[@]}" -c "$src" -o "$obj"
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

echo "Headless STM32 ARM-GCC build complete."
