function report = validate_codegen_parity(reference, candidate, varargin)
%VALIDATE_CODEGEN_PARITY Compare generated-C/Simulink output to a reference.

parser = inputParser();
parser.addParameter('ScalarTolerance', 2e-4, @(x) isnumeric(x) && isscalar(x));
parser.addParameter('ArrayTolerance', 5e-4, @(x) isnumeric(x) && isscalar(x));
parser.parse(varargin{:});
options = parser.Results;

if ischar(reference) || isstring(reference)
    reference = csv_result(char(reference));
end
if ischar(candidate) || isstring(candidate)
    candidate = csv_result(char(candidate));
end

fields = { ...
    'V_pack', 'T_core', 'T_surf', 'SoC_true', ...
    'V_min', 'V_max', 'T_max', 'T_avg'};
array_fields = {'V_group', 'V_segment', 'T_sensor', 'SoC_group'};
metrics = struct();
checks = struct();

for index = 1:numel(fields)
    name = fields{index};
    error_values = aligned_error(reference, candidate, name);
    metrics.(name) = hil.metrics(error_values);
    checks.(name) = metrics.(name).maximum_absolute <= ...
        double(options.ScalarTolerance);
end
for index = 1:numel(array_fields)
    name = array_fields{index};
    if isfield(reference, name) && isfield(candidate, name)
        error_values = aligned_error(reference, candidate, name);
        metrics.(name) = hil.metrics(error_values);
        checks.(name) = metrics.(name).maximum_absolute <= ...
            double(options.ArrayTolerance);
    end
end

names = fieldnames(checks);
failed = names(~cellfun(@(name) checks.(name), names));
report = struct( ...
    'passed', isempty(failed), ...
    'checks', checks, ...
    'failed_outputs', {failed}, ...
    'metrics', metrics, ...
    'scalar_tolerance', options.ScalarTolerance, ...
    'array_tolerance', options.ArrayTolerance);
end

function error_values = aligned_error(reference, candidate, name)
reference_values = double(reference.(name));
candidate_values = double(candidate.(name));
if isequal(reference.time_s(:), candidate.time_s(:))
    aligned = candidate_values;
else
    aligned = interp1(double(candidate.time_s(:)), candidate_values, ...
        double(reference.time_s(:)), 'linear', NaN);
end
if ~isequal(size(reference_values), size(aligned))
    error('hil:validation:ParityDimensions', ...
        'Parity output "%s" has incompatible dimensions.', name);
end
error_values = aligned - reference_values;
end

function result = csv_result(file_path)
data = readtable(file_path, 'VariableNamingRule', 'preserve');
result = struct();
result.time_s = data.time_s;
names = data.Properties.VariableNames;
array_names = {'V_group', 'V_segment', 'T_sensor', 'SoC_group'};
for array_index = 1:numel(array_names)
    prefix = [array_names{array_index}, '_'];
    selected = names(startsWith(names, prefix));
    if ~isempty(selected)
        indices = cellfun(@(name) str2double(extractAfter(name, prefix)), selected);
        [~, order] = sort(indices);
        result.(array_names{array_index}) = ...
            table2array(data(:, selected(order)));
    end
end
for index = 1:numel(names)
    is_array = any(cellfun(@(prefix) startsWith( ...
        names{index}, [prefix, '_']), array_names));
    if ~strcmp(names{index}, 'time_s') && ~is_array
        result.(names{index}) = data.(names{index});
    end
end
end
