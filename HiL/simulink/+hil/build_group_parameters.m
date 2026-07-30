function group = build_group_parameters(cell_cfg, pack_cfg)
%BUILD_GROUP_PARAMETERS Create repeatable bounded per-group variation.

Ns = double(pack_cfg.series_groups);
mode = lower(char(pack_cfg.imbalance_model.mode));
group = struct();
group.capacity_multiplier = ones(Ns, 1);
group.r0_multiplier = ones(Ns, 1);
group.r1_multiplier = ones(Ns, 1);
group.c1_multiplier = ones(Ns, 1);
group.r2_multiplier = ones(Ns, 1);
group.c2_multiplier = ones(Ns, 1);
group.rsa_multiplier = ones(Ns, 1);
group.initial_soc_offset = zeros(Ns, 1);

if ~strcmp(mode, 'parameter_distributed')
    group.initial_soc = repmat(double(pack_cfg.initial_soc), Ns, 1);
    return;
end

settings = pack_cfg.imbalance_model;
stream = RandStream('mt19937ar', 'Seed', double(settings.seed));
group.capacity_multiplier = bounded_multiplier(stream, Ns, ...
    settings.capacity_sigma_fraction, settings.capacity_bounds_fraction);
group.r0_multiplier = bounded_multiplier(stream, Ns, ...
    settings.r0_sigma_fraction, settings.r0_bounds_fraction);
group.r1_multiplier = bounded_multiplier(stream, Ns, ...
    settings.r1_sigma_fraction, settings.r1_bounds_fraction);
group.c1_multiplier = bounded_multiplier(stream, Ns, ...
    settings.c1_sigma_fraction, settings.c1_bounds_fraction);
group.r2_multiplier = bounded_multiplier(stream, Ns, ...
    settings.r2_sigma_fraction, settings.r2_bounds_fraction);
group.c2_multiplier = bounded_multiplier(stream, Ns, ...
    settings.c2_sigma_fraction, settings.c2_bounds_fraction);
group.rsa_multiplier = bounded_multiplier(stream, Ns, ...
    settings.thermal_resistance_sigma_fraction, ...
    settings.thermal_resistance_bounds_fraction);

offset = settings.initial_soc_sigma * randn(stream, Ns, 1);
offset = min(max(offset, settings.initial_soc_bounds(1)), ...
    settings.initial_soc_bounds(2));
offset = offset - mean(offset);
group.initial_soc_offset = offset;
if isfield(settings, 'overrides') && ~isempty(settings.overrides)
    group = apply_overrides(group, settings.overrides, Ns);
end
group.initial_soc = min(max(double(pack_cfg.initial_soc) + ...
    group.initial_soc_offset, 0), 1);
group.configuration_hash = hil.configuration_hash( ...
    cell_cfg.id, pack_cfg.id, settings, group);
end

function values = bounded_multiplier(stream, count, sigma, bounds)
values = 1.0 + double(sigma) * randn(stream, count, 1);
values = min(max(values, double(bounds(1))), double(bounds(2)));
values = values ./ mean(values);
values = min(max(values, double(bounds(1))), double(bounds(2)));
end

function group = apply_overrides(group, overrides, Ns)
if ~isstruct(overrides)
    error('hil:pack:OverrideType', ...
        'imbalance_model.overrides must be a struct or struct array.');
end
fields = { ...
    'capacity_multiplier', 'r0_multiplier', 'r1_multiplier', ...
    'c1_multiplier', 'r2_multiplier', 'c2_multiplier', ...
    'rsa_multiplier', 'initial_soc_offset'};
for override_index = 1:numel(overrides)
    override = overrides(override_index);
    if ~isfield(override, 'group_index')
        error('hil:pack:OverrideIndex', ...
            'Every group override requires group_index.');
    end
    group_index = double(override.group_index);
    if ~isscalar(group_index) || ~isfinite(group_index) || ...
            group_index < 1 || group_index > Ns || mod(group_index, 1) ~= 0
        error('hil:pack:OverrideIndex', ...
            'Override group_index must identify one configured series group.');
    end
    for field_index = 1:numel(fields)
        name = fields{field_index};
        if isfield(override, name)
            value = double(override.(name));
            if ~isscalar(value) || ~isfinite(value)
                error('hil:pack:OverrideValue', ...
                    'Override %s must be one finite scalar.', name);
            end
            if ~strcmp(name, 'initial_soc_offset') && value <= 0
                error('hil:pack:OverrideValue', ...
                    'Override %s must be positive.', name);
            end
            group.(name)(group_index) = value;
        end
    end
end
end
