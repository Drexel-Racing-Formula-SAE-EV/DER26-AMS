# EAC14-80-SCT manufacturer data request

**Purpose:** Close the evidence gap before the DER26 AMS curve-based fuse
observer can become an authoritative State-of-Power constraint.

**Installed part under review:** Eaton Bussmann EAC14-80-SCT, 80 A,
500 Vdc / 420 Vac, bolt-down single-cap terminal.

## Data requested from Eaton application engineering

1. Guaranteed-minimum pre-arcing time-current curve for the EAC14-80-SCT,
   including tolerance bands rather than only a typical centerline.
2. Guaranteed-minimum pre-arcing I²t versus prospective current and time.
3. Maximum total-clearing time/current and I²t data where relevant to system
   protection coordination.
4. Clarification of whether the page-4 and page-5 plotted curves are typical,
   minimum, maximum, pre-arcing, or total-clearing values.
5. Applicable production, lot, and environmental tolerances.
6. Recommended method for evaluating non-rectangular and repeated traction
   current pulses with cooling intervals.
7. Repetitive-pulse endurance or aging data, including cumulative weakening
   below the one-shot melting curve.
8. Derating guidance based on measured fuse body, terminal, holder, and ambient
   temperature, and which temperature the published derating curve assumes.
9. Thermal time constant or validated lumped thermal model, if available.
10. Impact of bolt-down mounting, busbar cross-section, terminal torque,
    enclosure airflow, vibration, and adjacent heat sources.
11. Guidance for a 75s6p Formula SAE accumulator with an 80 A EAC14-80-SCT,
    including short acceleration pulses and repeated endurance duty.
12. Confirmation that 8020 A²s is a typical melting value measured at 800 A,
    and whether a guaranteed-minimum value is available.

## Vehicle evidence still required even with manufacturer data

- Confirm exact installed fuse marking and part revision.
- Verify M5 terminal torque and mounting arrangement.
- Measure calibrated pack current and fuse/terminal temperatures.
- Replay acceleration, autocross, and endurance traces.
- Test hot starts, repeated pulses, cooling, and MCU reset behavior.
- Coordinate the software model with independent overcurrent trips, contactor,
  busbar, cable, and inverter limits.

## Release rule

`AMS_FUSE_MODEL_VALIDATED` remains zero until:

- the guaranteed-minimum manufacturer basis is documented;
- a conservative software margin is justified;
- installed-path tests pass; and
- the revision identifiers and evidence are frozen in the vehicle build.
