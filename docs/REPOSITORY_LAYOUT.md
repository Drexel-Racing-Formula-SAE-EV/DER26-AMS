# Repository Layout

The repository is intentionally split into the target firmware project and support infrastructure rather than flattening everything into one directory.

## `AMS/`

The STM32CubeIDE project. Keeping the target project self-contained preserves CubeIDE/CubeMX paths and makes it possible to import the directory directly.

Important subdirectories:

- `Core/Src/` and `Core/Inc/` — DER26 application code and interfaces.
- `Drivers/` — ST HAL/CMSIS vendor code.
- `Middlewares/` — FreeRTOS/CMSIS-RTOS middleware.
- `docs/` — active firmware contracts and module documentation.
- `host_tests/` — host unit, SIL, stress, sanitizer, and source-contract tests.

## `HiL/`

Hardware-in-the-loop plant/support assets. HIL code is kept outside the STM32 project so target firmware and test-plant dependencies remain separated.

## `Tools/`

Reference models and offline utilities used to validate production algorithms or inspect data. These are support tools, not target firmware dependencies unless a test target explicitly references them.

## `ci/`

Repository hygiene checks and the headless ARM-GCC target build.

## `docs/`

Repository-level documentation. Detailed firmware contracts stay under `AMS/docs/` so they version with the code they describe.

## Paths intentionally not renamed

`AMS/Core/Src/ext_drivers/` contains more than literal peripheral drivers, but it is referenced by CubeIDE, host tests, CI, and source-contract checks. A cosmetic mass rename would create a large nonfunctional diff and unnecessary build risk. If it is ever migrated, it should be done incrementally with the build/test paths changed in the same commit.
