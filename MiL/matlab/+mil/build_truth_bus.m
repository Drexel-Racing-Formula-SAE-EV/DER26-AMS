function truth = build_truth_bus(plant, params)
%BUILD_TRUTH_BUS Construct a hidden scoring-only bus from the distributed plant.
%
% Nothing downstream of the AMS measurement adapter is permitted to consume
% this bus. It exists only for MiL scoring/oracle calculations.
required = {'Vp1_group','Vp2_group','T_core_group','T_surf_group'};
for k = 1:numel(required)
    if ~isfield(plant, required{k})
        error('mil:truth:MissingGroupState', ...
            'Plant run must enable store_group_truth; missing %s.', required{k});
    end
end
pack = plant.pack_configuration;
Ns = double(pack.series_groups);
Nseg = double(pack.segment_count);
N = numel(plant.time_s);
if size(plant.V_group,2) ~= Ns
    error('mil:truth:Topology', 'Plant group-voltage topology mismatch.');
end

truth = struct();
truth.tag = 'SCORING_ONLY_DO_NOT_FEED_ESTIMATOR';
truth.time_s = double(plant.time_s(:));
truth.sequence = uint32((1:N).');
truth.pack_current_A = double(plant.I_pack(:));
truth.ambient_C = double(plant.T_ambient(:));
truth.pack_voltage_V = double(plant.V_pack(:));
truth.group_voltage_V = double(plant.V_group);
truth.group_soc = double(plant.SoC_group);
truth.group_vp1_V = double(plant.Vp1_group);
truth.group_vp2_V = double(plant.Vp2_group);
truth.group_core_C = double(plant.T_core_group);
truth.group_surface_C = double(plant.T_surf_group);
truth.temp_sensor_C = double(plant.T_sensor);
truth.group_capacity_multiplier = double(plant.group_parameters.capacity_multiplier(:));
truth.group_r0_multiplier = double(plant.group_parameters.r0_multiplier(:));
truth.group_r1_multiplier = double(plant.group_parameters.r1_multiplier(:));
truth.group_c1_multiplier = double(plant.group_parameters.c1_multiplier(:));
truth.group_r2_multiplier = double(plant.group_parameters.r2_multiplier(:));
truth.group_c2_multiplier = double(plant.group_parameters.c2_multiplier(:));
truth.group_rsa_multiplier = double(plant.group_parameters.rsa_multiplier(:));

truth.group_r0_ohm = zeros(N,Ns);
for n = 1:N
    truth.group_r0_ohm(n,:) = mil.util.r0(params, ...
        truth.group_soc(n,:), truth.group_core_C(n,:)) .* ...
        truth.group_r0_multiplier.';
end

truth.segment_voltage_V = zeros(N,Nseg);
truth.segment_soc = zeros(N,Nseg);
truth.segment_vp1_V = zeros(N,Nseg);
truth.segment_vp2_V = zeros(N,Nseg);
truth.segment_r0_ohm = zeros(N,Nseg);
truth.segment_core_C = zeros(N,Nseg);
truth.segment_surface_max_C = zeros(N,Nseg);
truth.segment_min_cell_V = zeros(N,Nseg);
truth.segment_max_cell_V = zeros(N,Nseg);
for s = 1:Nseg
    mask = double(pack.group_to_segment(:)) == s;
    truth.segment_voltage_V(:,s) = sum(truth.group_voltage_V(:,mask),2);
    capacity_weights=truth.group_capacity_multiplier(mask).';
    truth.segment_soc(:,s) = sum(truth.group_soc(:,mask).*capacity_weights,2)./ ...
        sum(capacity_weights);
    truth.segment_vp1_V(:,s) = mean(truth.group_vp1_V(:,mask),2);
    truth.segment_vp2_V(:,s) = mean(truth.group_vp2_V(:,mask),2);
    truth.segment_r0_ohm(:,s) = mean(truth.group_r0_ohm(:,mask),2);
    truth.segment_core_C(:,s) = mean(truth.group_core_C(:,mask),2);
    truth.segment_surface_max_C(:,s) = max(truth.group_surface_C(:,mask),[],2);
    truth.segment_min_cell_V(:,s) = min(truth.group_voltage_V(:,mask),[],2);
    truth.segment_max_cell_V(:,s) = max(truth.group_voltage_V(:,mask),[],2);
end
pack_capacity_weights=truth.group_capacity_multiplier.';
truth.pack_soc=sum(truth.group_soc.*pack_capacity_weights,2)./sum(pack_capacity_weights);
truth.minimum_cell_V = min(truth.group_voltage_V,[],2);
truth.maximum_cell_V = max(truth.group_voltage_V,[],2);
truth.maximum_core_C = max(truth.group_core_C,[],2);
truth.maximum_surface_C = max(truth.group_surface_C,[],2);
truth.parameter_hash = plant.parameter_hash;
truth.configuration_hash = plant.configuration_hash;
end
