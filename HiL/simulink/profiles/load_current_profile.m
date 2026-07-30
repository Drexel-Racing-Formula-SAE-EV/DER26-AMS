function profile = load_current_profile(profile_cfg, sim_cfg, cell_cfg, pack_cfg)
%LOAD_CURRENT_PROFILE Load, normalize, resample, repeat, and explicitly scale.

kind = lower(char(profile_cfg.kind));
switch kind
    case 'synthetic_hppc'
        profile = synthetic_hppc(sim_cfg);

    case 'embedded_c_header'
        values = parse_c_array(profile_cfg.source_file, profile_cfg.array_name);
        source_time = (0:(numel(values) - 1)).' * ...
            double(profile_cfg.source_sample_time_s);
        source_current = values(:) * ...
            double(profile_cfg.current_scale_A_per_count);
        profile = base_profile(source_time, source_current, sim_cfg, ...
            profile_cfg.name, profile_cfg.source_file, profile_cfg);

    case 'csv'
        table_data = readtable(profile_cfg.source_file, ...
            'VariableNamingRule', 'preserve');
        time = table_data.(profile_cfg.time_column);
        if isfield(profile_cfg, 'power_column') && ...
                ~isempty(profile_cfg.power_column)
            power = table_data.(profile_cfg.power_column);
            profile = base_profile(time, power, sim_cfg, ...
                profile_cfg.name, profile_cfg.source_file, profile_cfg);
            profile.pack_power_W = profile.pack_current_A;
            profile.pack_current_A = zeros(size(profile.pack_power_W));
        else
            current = table_data.(profile_cfg.current_column);
            profile = base_profile(time, current, sim_cfg, ...
                profile_cfg.name, profile_cfg.source_file, profile_cfg);
        end

    case 'mat'
        loaded = load(profile_cfg.source_file);
        time = loaded.(profile_cfg.time_variable);
        current = loaded.(profile_cfg.current_variable);
        profile = base_profile(time, current, sim_cfg, ...
            profile_cfg.name, profile_cfg.source_file, profile_cfg);

    case 'constant_current'
        time = (0:sim_cfg.sample_time_s:double(sim_cfg.stop_time_s)).';
        current = repmat(double(profile_cfg.current_A), size(time));
        profile = base_profile(time, current, sim_cfg, ...
            profile_cfg.name, 'configuration', profile_cfg);

    case 'constant_power'
        time = (0:sim_cfg.sample_time_s:double(sim_cfg.stop_time_s)).';
        profile = base_profile(time, zeros(size(time)), sim_cfg, ...
            profile_cfg.name, 'configuration', profile_cfg);
        profile.pack_power_W = repmat(double(profile_cfg.power_W), size(time));

    otherwise
        error('hil:profile:UnknownKind', 'Unsupported profile kind "%s".', kind);
end

profile = apply_repeat_policy(profile, profile_cfg, sim_cfg);
profile = resample_profile(profile, double(sim_cfg.sample_time_s));
profile = apply_scaling(profile, profile_cfg, cell_cfg, pack_cfg);

if isnumeric(sim_cfg.stop_time_s)
    keep = profile.time_s <= double(sim_cfg.stop_time_s) + 1e-9;
    profile = subset_profile(profile, keep);
end
end

function profile = base_profile(time, current, sim_cfg, name, source, profile_cfg)
time = double(time(:));
current = double(current(:));
if numel(time) ~= numel(current)
    error('hil:profile:LengthMismatch', ...
        'Profile time and current vectors have different lengths.');
end
if numel(time) < 2
    error('hil:profile:TooShort', 'A profile requires at least two samples.');
end
valid = isfinite(time) & isfinite(current);
time = time(valid);
current = current(valid);
[time, order] = sort(time);
current = current(order);
[time, unique_index] = unique(time, 'stable');
current = current(unique_index);
time = time - time(1);

profile = struct( ...
    'time_s', time, ...
    'pack_current_A', current, ...
    'ambient_temperature_C', ...
        repmat(double(sim_cfg.ambient_temperature_C), size(time)), ...
    'name', char(name), ...
    'source', char(source), ...
    'sample_time_s', median(diff(time)), ...
    'repeat_policy', char(profile_cfg.repeat_policy), ...
    'scaling_mode', char(profile_cfg.scaling_mode));
end

function values = parse_c_array(file_path, array_name)
text = fileread(file_path);
pattern = sprintf( ...
    'static\\s+const\\s+int16_t\\s+%s\\s*\\[\\]\\s*=\\s*\\{(.*?)\\}\\s*;', ...
    regexptranslate('escape', array_name));
match = regexp(text, pattern, 'tokens', 'once');
if isempty(match)
    error('hil:profile:CArrayMissing', ...
        'Could not find int16 array "%s" in %s.', array_name, file_path);
end
tokens = regexp(match{1}, '[-+]?\\d+', 'match');
values = cellfun(@str2double, tokens(:));
if isempty(values) || any(~isfinite(values))
    error('hil:profile:CArrayInvalid', ...
        'Array "%s" in %s contains no valid values.', array_name, file_path);
end
end

function profile = apply_repeat_policy(profile, cfg, sim_cfg)
policy = lower(char(cfg.repeat_policy));
switch policy
    case 'one_shot'
        return;
    case 'repeat'
        if isnumeric(sim_cfg.stop_time_s)
            period = profile.time_s(end) + profile.sample_time_s;
            repeat_count = max(1, ceil( ...
                (double(sim_cfg.stop_time_s) + profile.sample_time_s) / period));
            profile = repeat_profile(profile, repeat_count);
        end
    case 'repeat_count'
        repeat_count = double(cfg.repeat_count);
        if repeat_count < 1 || mod(repeat_count, 1) ~= 0
            error('hil:profile:RepeatCount', 'repeat_count must be a positive integer.');
        end
        profile = repeat_profile(profile, repeat_count);
    otherwise
        error('hil:profile:RepeatPolicy', 'Unsupported repeat policy "%s".', policy);
end
end

function profile = repeat_profile(profile, repeat_count)
original = profile;
period = original.time_s(end) + original.sample_time_s;
sample_count = numel(original.time_s);
total_count = sample_count * repeat_count;
time = zeros(total_count, 1);
current = zeros(total_count, 1);
ambient = zeros(total_count, 1);
if isfield(original, 'pack_power_W')
    power = zeros(total_count, 1);
end
for index = 0:(repeat_count - 1)
    destination = index * sample_count + (1:sample_count);
    time(destination) = original.time_s + index * period;
    current(destination) = original.pack_current_A;
    ambient(destination) = original.ambient_temperature_C;
    if isfield(original, 'pack_power_W')
        power(destination) = original.pack_power_W;
    end
end
profile.time_s = time;
profile.pack_current_A = current;
profile.ambient_temperature_C = ambient;
if isfield(original, 'pack_power_W')
    profile.pack_power_W = power;
end
end

function profile = resample_profile(profile, target_sample_time)
target_time = (profile.time_s(1):target_sample_time:profile.time_s(end)).';
profile.pack_current_A = interp1(profile.time_s, profile.pack_current_A, ...
    target_time, 'previous', 'extrap');
profile.ambient_temperature_C = interp1( ...
    profile.time_s, profile.ambient_temperature_C, ...
    target_time, 'linear', 'extrap');
if isfield(profile, 'pack_power_W')
    profile.pack_power_W = interp1(profile.time_s, profile.pack_power_W, ...
        target_time, 'previous', 'extrap');
end
profile.time_s = target_time;
profile.sample_time_s = target_sample_time;
end

function profile = apply_scaling(profile, cfg, cell_cfg, pack_cfg)
mode = lower(char(cfg.scaling_mode));
switch mode
    case {'no_scaling', 'vehicle_current_replay'}
        scale = 1.0;
    case 'constant_c_rate'
        require_reference(cfg);
        source_capacity = double(cfg.reference_parallel_cells) * ...
            double(cfg.reference_cell_capacity_Ah);
        target_capacity = double(pack_cfg.parallel_cells) * ...
            double(cell_cfg.nominal_capacity_Ah);
        scale = target_capacity / source_capacity;
    case 'constant_cell_current'
        require_reference(cfg);
        scale = double(pack_cfg.parallel_cells) / ...
            double(cfg.reference_parallel_cells);
    case 'constant_pack_power'
        scale = 1.0;
        if ~isfield(profile, 'pack_power_W')
            if ~isfield(cfg, 'reference_pack_voltage_V')
                error('hil:profile:PowerReference', ...
                    'constant_pack_power requires power data or reference_pack_voltage_V.');
            end
            profile.pack_power_W = profile.pack_current_A * ...
                double(cfg.reference_pack_voltage_V);
        end
    otherwise
        error('hil:profile:ScalingMode', ...
            'Unsupported profile scaling mode "%s".', mode);
end
profile.pack_current_A = profile.pack_current_A * scale;
profile.current_scale = scale;
end

function require_reference(cfg)
required = {'reference_parallel_cells', 'reference_cell_capacity_Ah'};
for index = 1:numel(required)
    if ~isfield(cfg, required{index})
        error('hil:profile:ScalingReference', ...
            'Scaling mode requires profile field "%s".', required{index});
    end
end
end

function profile = subset_profile(profile, keep)
profile.time_s = profile.time_s(keep);
profile.pack_current_A = profile.pack_current_A(keep);
profile.ambient_temperature_C = profile.ambient_temperature_C(keep);
if isfield(profile, 'pack_power_W')
    profile.pack_power_W = profile.pack_power_W(keep);
end
end
