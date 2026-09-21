# Bundled FreeRTOS provenance

The current AMS tree contains inconsistent upstream version labels:

- `Source/include/task.h` reports `tskKERNEL_VERSION_NUMBER` = `V10.2.0`.
- The bundled FreeRTOS source/header banners report `FreeRTOS Kernel V10.2.1`.

The firmware does **not** rewrite those vendor files to make the labels agree.
Doing that would manufacture provenance rather than establish it.

For the current baseline, the exact bundled `Middlewares/Third_Party/FreeRTOS/Source`
tree is pinned by the repository gate to:

`SHA-256 4393c390c3939c1ce11c713c9b862ec1b27cd9a3448e73a7a48c9c11dbdc3824`

The hash is computed over every file in that directory in sorted relative-path
order, including each relative path and file bytes. Run:

```sh
python3 AMS/host_tests/tools/check_freertos_provenance.py
```

Any vendor-source change intentionally invalidates the gate and requires a
reviewed provenance update. Target build evidence must additionally record the
compiler/toolchain version, build type, source revision and deterministic build
epoch; the headless build script writes those fields into its build directory.
