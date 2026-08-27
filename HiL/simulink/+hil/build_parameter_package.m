function params = build_parameter_package(cell_cfg, values, source_manifest)
%BUILD_PARAMETER_PACKAGE Create the reviewed parameter artifact contract.

if nargin < 3
    source_manifest = struct();
end
[cell_cfg, ~] = hil.validate_configuration(cell_cfg);

required = {'OCV', 'R0', 'R1', 'C1', 'R2', 'C2', 'Cc', 'Cs', 'Rcs', 'Rsa'};
for index = 1:numel(required)
    if ~isfield(values, required{index})
        error('hil:params:MissingField', ...
            'Parameter values are missing "%s".', required{index});
    end
end

params = struct();
params.schema_version = 1;
params.cell_id = cell_cfg.id;
params.Q_nom = single(cell_cfg.nominal_capacity_Ah);
params.SoC_init = single(1.0);
params.T_init = single(25.0);
params.soc_common = single(cell_cfg.soc_breakpoints(:));
params.soc_ocv_common = single(cell_cfg.ocv_soc_breakpoints(:));
params.temp_bp = single(cell_cfg.temperature_breakpoints_C(:).');
params.temp_bp_ocv = params.temp_bp;

params.OCV = single(values.OCV);
params.R0 = single(values.R0);
params.R1 = single(values.R1);
params.C1 = single(values.C1);
params.R2 = single(values.R2);
params.C2 = single(values.C2);
params.Cc = single(values.Cc);
params.Cs = single(values.Cs);
params.Rcs = single(values.Rcs);
params.Rsa = single(values.Rsa);

% Compatibility aliases consumed by the existing Simulink model.
params.OCV_2d = params.OCV;
params.R0_2d_fix = params.R0;
params.R1_2d = params.R1;
params.inv_C1_2d_fixed = single(1.0 ./ double(params.C1));
params.neg_inv_R1C1_2d_fixed = single( ...
    -1.0 ./ (double(params.R1) .* double(params.C1)));

if isfield(values, 'fit_quality')
    params.fit_quality = values.fit_quality;
else
    params.fit_quality = struct();
end
params.validity_domain = struct( ...
    'soc', [min(double(params.soc_ocv_common)), ...
            max(double(params.soc_ocv_common))], ...
    'temperature_C', [min(double(params.temp_bp)), ...
                      max(double(params.temp_bp))], ...
    'current_sign', 'positive_discharge');
params.source_manifest = source_manifest;
params.generation_timestamp_utc = char(datetime('now', ...
    'TimeZone', 'UTC', 'Format', 'yyyy-MM-dd''T''HH:mm:ssXXX'));
params.configuration_hash = hil.configuration_hash(cell_cfg, ...
    rmfield_if_present(values, 'fit_quality'), source_manifest);
end

function value = rmfield_if_present(value, field_name)
if isfield(value, field_name)
    value = rmfield(value, field_name);
end
end
