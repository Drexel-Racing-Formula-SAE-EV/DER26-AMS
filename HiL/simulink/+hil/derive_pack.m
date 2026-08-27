function pack_cfg = derive_pack(cell_cfg, pack_cfg)
%DERIVE_PACK Add cross-checked values derived from cell and topology inputs.

[cell_cfg, ~] = hil.validate_configuration(cell_cfg);
[pack_cfg, ~] = hil.validate_configuration(pack_cfg);
if isfield(cell_cfg, 'is_template') && cell_cfg.is_template
    error('hil:config:TemplateCell', ...
        'Cell configuration "%s" is a non-buildable template.', cell_cfg.id);
end
if isfield(pack_cfg, 'is_template') && pack_cfg.is_template
    error('hil:config:TemplatePack', ...
        'Pack configuration "%s" is a non-buildable template.', pack_cfg.id);
end

pack_cfg.total_cells = pack_cfg.series_groups * pack_cfg.parallel_cells;
pack_cfg.nominal_capacity_Ah = ...
    pack_cfg.parallel_cells * cell_cfg.nominal_capacity_Ah;
pack_cfg.nominal_voltage_V = ...
    pack_cfg.series_groups * cell_cfg.nominal_voltage_V;
pack_cfg.nominal_energy_Wh = ...
    pack_cfg.nominal_capacity_Ah * pack_cfg.nominal_voltage_V;
pack_cfg.minimum_pack_voltage_V = ...
    pack_cfg.series_groups * cell_cfg.minimum_voltage_V;
pack_cfg.maximum_pack_voltage_V = ...
    pack_cfg.series_groups * cell_cfg.maximum_voltage_V;
pack_cfg.maximum_discharge_current_A = ...
    pack_cfg.parallel_cells * cell_cfg.maximum_discharge_current_A;
pack_cfg.maximum_charge_current_A = ...
    pack_cfg.parallel_cells * cell_cfg.maximum_charge_current_A;

if abs(pack_cfg.minimum_group_voltage_V - cell_cfg.minimum_voltage_V) > 1e-9
    error('hil:config:MinimumVoltageMismatch', ...
        'Pack and cell minimum group voltage limits disagree.');
end
if abs(pack_cfg.maximum_group_voltage_V - cell_cfg.maximum_voltage_V) > 1e-9
    error('hil:config:MaximumVoltageMismatch', ...
        'Pack and cell maximum group voltage limits disagree.');
end
end
