# Host Test Quickstart

From `AMS/host_tests`:

```bash
make clean
make test
```

Focused validation:

```bash
make unit
make system-sil
make apm-sil
make fuse-oracle
make production-gates
make static-allocation-gate
```

Sanitizer/static-analysis targets, when supported by the host toolchain:

```bash
make asan
make analyze
```

Clean generated host artifacts before packaging:

```bash
make clean
```

These tests exercise firmware logic with fake hardware/RTOS adapters. They do not replace STM32 target builds or bench/HIL validation. See `LIMITATIONS.md` and `TEST_MATRIX.md`.
