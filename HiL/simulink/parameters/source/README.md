# Reviewed parameter sources

`p42a_legacy_codegen_snapshot.mat` is an exact reconstruction of the constants
in the qualified generated-C model. It preserves behavior and makes the
constants loadable without treating C source as the parameter database.

It is migration evidence, not raw fitting evidence. A defensible refit still
requires the external P42A CDT/HCGT dataset through `HIL_DATA_ROOT`.
