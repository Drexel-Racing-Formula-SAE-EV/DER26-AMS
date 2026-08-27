# Validation artifacts

- `baselines/`: frozen host-C behavior plus source/archive hashes.
- `generated/`: machine-generated summaries and future validation output.
- `Table1_ECM_Parameters.*`: representative values from generated model 1.67.
- `Figure_US06_25C_600s_Matched_SoC_Check.pdf`: retained historical figure; its
  provenance predates the reusable report generator.

The old Table 1 values did not match the checked-in generated model. They were
replaced with values reconstructed from the actual C oracle; the exact table
source is also in `generated/p42a_legacy_parameter_summary.csv`.
