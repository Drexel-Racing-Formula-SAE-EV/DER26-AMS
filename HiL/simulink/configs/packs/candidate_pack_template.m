function cfg = candidate_pack_template()
%CANDIDATE_PACK_TEMPLATE Copy and replace topology/mapping placeholders.

cfg = structural_12s2p();
cfg.id = 'candidate_pack_template';
cfg.is_template = true;
cfg.thermal_layout.geometry_status = 'REPLACE_ME';
cfg.cooling_boundary.assumption = 'REPLACE_ME';
end
