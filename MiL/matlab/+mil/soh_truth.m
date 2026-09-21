function truth_soh = soh_truth(truth, pack_cfg)
%SOH_TRUTH Derive explicit plant-defined capacity/resistance health targets.
%
% Capacity of a series pack is ultimately limited by its weakest series group;
% mean values are retained only as diagnostics. Resistance growth uses the
% configured multiplicative plant parameter variation, independent of the
% estimator's online R0 state.
S = double(pack_cfg.segment_count);
truth_soh = struct();
truth_soh.capacity_group = truth.group_capacity_multiplier(:);
truth_soh.resistance_group = truth.group_r0_multiplier(:);
truth_soh.capacity_pack_weakest = min(truth_soh.capacity_group);
truth_soh.capacity_pack_mean = mean(truth_soh.capacity_group);
truth_soh.resistance_pack_worst = max(truth_soh.resistance_group);
truth_soh.resistance_pack_mean = mean(truth_soh.resistance_group);
% Architecture-observable production SoH targets. The capacity observer sees
% aggregate capacity through capacity-weighted pack SoC. The five production
% R0 observers each see a 15-group segment-equivalent resistance.
truth_soh.capacity_pack_observable = truth_soh.capacity_pack_mean;
truth_soh.capacity_segment_weakest = zeros(S,1);
truth_soh.capacity_segment_mean = zeros(S,1);
truth_soh.resistance_segment_worst = zeros(S,1);
truth_soh.resistance_segment_mean = zeros(S,1);
for s = 1:S
    mask = double(pack_cfg.group_to_segment(:)) == s;
    truth_soh.capacity_segment_weakest(s) = min(truth_soh.capacity_group(mask));
    truth_soh.capacity_segment_mean(s) = mean(truth_soh.capacity_group(mask));
    truth_soh.resistance_segment_worst(s) = max(truth_soh.resistance_group(mask));
    truth_soh.resistance_segment_mean(s) = mean(truth_soh.resistance_group(mask));
end
truth_soh.resistance_pack_observable = max(truth_soh.resistance_segment_mean);
end
