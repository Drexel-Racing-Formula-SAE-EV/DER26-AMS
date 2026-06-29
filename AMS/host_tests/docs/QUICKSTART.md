# Quickstart

## 1. Put the harness in the AMS repo

```bash
cd DER26-AMS-feature-canbus_charger
cp -r /path/to/DER26-AMS-host-test-harness host_tests
```

## 2. Run normal tests

```bash
cd host_tests
make test
```

## 3. Run sanitizer tests

```bash
make asan
```

## 4. Run static analyzer

```bash
make analyze
```

## 5. Expected pass line

```text
ALL COMPREHENSIVE HOST INJECTION TESTS PASSED
```

## 6. Interpreting results

Pass means host-exercised firmware logic passed fake-signal and fault-injection checks.

Pass does not mean hardware is validated. You still need CubeIDE build, board flashing, ADBMS chain testing, charger CAN sniffing, ECU/dashboard test, and safe low-voltage bench validation.
