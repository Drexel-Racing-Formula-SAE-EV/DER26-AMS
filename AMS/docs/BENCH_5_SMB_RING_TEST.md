# Five-SMB Passive Ring Bench Check

Flash the `BENCH_VALIDATION` five-SMB target built from AMS v0.5.19. Connect
the five SMBs on String A in their final isoSPI order, but keep the cell stack
electrically unloaded. Do not connect a charger or load while the passive-ring
assumption is enabled.

Keep the boards and cells near 25 C. Automatic thermistor scanning is
deliberately disabled on Rev5 hardware because the GPIO4/GPIO5 100-ohm pull-up
network is not validated.

Run these CLI commands after boot:

```text
ver
status
spi lifecycle
spi ages
volt
```

Confirm:

- firmware is `0.5.19`, build is `bench_validation`;
- the passive-ring observer banner is present;
- BMS_OK is 0, output inhibit is 1 and balance inhibit is 1;
- the physical chain is five SMBs on String A;
- the last voltage scan succeeds;
- `Voltage valid:1`, `Cells usable:75`, `updated:75`, `stale:0`, `pec:0`;
- every cell voltage is plausible before trusting the estimator output.

Leave the ring undisturbed for at least 25 seconds, then run:

```text
estimator
power
```

The expected estimator state is five segment lines with `valid:1`, `acq:2`
and `reason:10`. Compare the five segment SoCs; a materially different segment
should prompt a cell-voltage review rather than averaging it away.

`power` is expected to report SoP authority false and both SoH validity fields
false. That is correct for this setup. A passive voltage-only session can
estimate rested charge level, but it cannot measure capacity SoH, resistance
SoH or loaded power capability.

Stop and inspect the ring if any SMB is absent, any usable mask is not
`0x7FFF`, any cell reads unavailable, PEC/stale counts are nonzero, or the
five estimators do not complete acquisition after a second 25-second rest.

