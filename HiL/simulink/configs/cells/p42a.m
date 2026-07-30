function cfg = p42a()
%P42A Molicel P42A cell identity and fitting contract.

cfg = struct();
cfg.schema_version = 1;
cfg.kind = 'cell';
cfg.id = 'p42a';
cfg.is_template = false;
cfg.manufacturer = 'Molicel';
cfg.model = 'INR-21700-P42A';
cfg.nominal_capacity_Ah = 4.2;
cfg.nominal_voltage_V = 3.6;
cfg.minimum_voltage_V = 2.5;
cfg.maximum_voltage_V = 4.2;
cfg.maximum_charge_current_A = 8.4;
cfg.maximum_discharge_current_A = 45.0;

cfg.soc_breakpoints = linspace(0.10, 1.00, 12).';
cfg.ocv_soc_breakpoints = (0.00:0.01:1.00).';
cfg.temperature_breakpoints_C = [5, 25, 40].';
cfg.temperature_model_limits_C = [5, 40];
cfg.current_sign = 'positive_discharge';
cfg.dataset_adapter = 'load_p42a_hcgt';
cfg.dataset_configuration = 'p42a_published_hcgt';

fit = struct();
fit.cell_ids = { ...
    'A2', 'A4', 'A6', 'A8', 'A9', 'A11', 'A12', 'A13'};
fit.holdout_cell_ids = {'A15', 'A19', 'A21', 'A37'};
fit.ocv_cell_id = 'A2';
fit.hcgt_soc_regions = [0.95, 0.85, 0.75, 0.65, 0.50, 0.35, 0.25, 0.15];
fit.r0_soc_regions = [0.95, 0.85, 0.75, 0.65, 0.50, 0.35, 0.25, 0.15, 0.05];
fit.current_tolerance_fraction = 0.65;
fit.rest_current_threshold_A = 0.20;
fit.minimum_valid_fast_fits = 3;
fit.minimum_valid_slow_fits = 10;
fit.ocv_voltage_bounds_V = [2.50, 4.25];
fit.r0_bounds_ohm = [0.006, 0.050];
fit.r1_bounds_ohm = [0.0003, 0.030];
fit.c1_bounds_F = [300, 100000];
fit.tau1_bounds_s = [0.5, 300];
fit.r2_bounds_ohm = [0.0002, 0.030];
fit.c2_bounds_F = [1000, 1000000];
fit.tau2_bounds_s = [20, 500];
fit.fallback_R1_ohm = 0.0035;
fit.fallback_C1_F = 3000.0;
fit.fallback_R2_ohm = 0.0040;
fit.fallback_C2_F = 12000.0;
cfg.parameter_fit_options = fit;

thermal = struct();
thermal.Cc_J_per_K = 55.0;
thermal.Cs_J_per_K = 15.0;
thermal.Rcs_K_per_W = 1.5;
thermal.Rsa_K_per_W = 8.0;
thermal.status = 'first-pass prior; pack installation not calibrated';
cfg.thermal_prior = thermal;

paths = hil.project_paths();
cfg.parameter_source = struct( ...
    'mode', 'legacy_snapshot_or_refit', ...
    'legacy_snapshot_file', fullfile(paths.parameters_source, ...
        'p42a_legacy_codegen_snapshot.mat'));

cfg.provenance = struct( ...
    'dataset', 'Published Molicel P42A CDT/HCGT data', ...
    'dataset_owner', 'Kollmeyer battery dataset', ...
    'parameter_status', 'electrical LUTs fitted; thermal values provisional', ...
    'known_limitations', {{ ...
        'Raw published data is external and must be supplied through HIL_DATA_ROOT.', ...
        'Electrical fitting and validation use disjoint P42A cell IDs.', ...
        'Published HCGT chamber temperature is not cell-surface evidence.', ...
        'Legacy snapshot was reconstructed from the checked-in generated C oracle.', ...
        'Thermal installation parameters are not a fanless-pack qualification.'}});
end
