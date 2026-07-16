# ADBMS6830 Five-SMB Reference Alignment

This package uses the DER26 final-ring topology and behavior:

- five SMB ADBMS6830B devices;
- one ADBMS2950B APM device at the opposite end of the ring;
- fifteen populated cell channels per SMB;
- the existing ADBMS6822/SPI6 electrical mode, with String A owning the
  five-SMB subset and String B owning the one-APM subset;
- the existing thermistor-mux, configuration, balancing and safety policies.

The production implementation was independently checked against two vendor
reference implementations and the applicable device timing requirements.
No vendor source or confidential document is included in this package.

The review confirmed the existing ADCV command encoding, redundant continuous
measurement selection, PEC/counter validation, six individual cell-register
group reads and cell-voltage scaling. The individual group commands are used
instead of the mixed-chain-incompatible read-all shortcut. It resulted in these
timing corrections:

- each isoSPI wake low/high interval is 1 ms instead of 250/100 us;
- cold wake uses the same checked 1 ms timing contract;
- the code waits the complete 8 ms redundant C-ADC/S-ADC conversion interval
  before publishing cell data;
- compile-time constants and host regressions guard the wake, reference and
  conversion timing assumptions.

No separate eval-board branch is included. The final firmware retains the full
five-SMB thermistor/balancing behavior and adds one advisory APM path without
giving APM measurements authority over BMS_OK.

Host CI, safety profiles, static analysis, sanitizers and stress testing do not
replace an ARM target build or current-limited physical validation.
