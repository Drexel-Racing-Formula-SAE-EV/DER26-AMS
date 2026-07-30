# Model sources

`der_accumulator_2rc_thermal_plant.slx` is the clean reusable template. It is
intentionally identical to the validated model 1.67 at rest in the repository.
`hil.configure_model` copies it into the generated-output directory and changes
only the copy:

- model-workspace parameter package;
- Ns/Np gains;
- initial conditions and sample time;
- LUT temperature clamp;
- fixed-size output-expansion chart and signal dimensions;
- solver and artifact metadata.

The long `drev_..._validated.slx` file is retained as the frozen source
reference. Do not hand-edit either source to build a new cell/pack; change
reviewed configuration and parameter artifacts instead.
