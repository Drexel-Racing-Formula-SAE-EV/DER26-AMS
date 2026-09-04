function campaign = sop_campaign(truth,params,pack_cfg,cfg,fuse_replay)
%SOP_CAMPAIGN Evaluate distributed truth-based SoP at configured checkpoints.
if nargin < 5,fuse_replay=struct([]);end
times = double(cfg.checkpoint_times_s(:));
if isempty(times)
    times = truth.time_s(round(linspace(1,numel(truth.time_s),min(10,numel(truth.time_s)))))';
end
valid_times = times(times >= truth.time_s(1) & times <= truth.time_s(end));
items = cell(numel(valid_times),1);
for k = 1:numel(valid_times)
    [~,idx] = min(abs(truth.time_s-valid_times(k)));
    item = mil.oracle.sop_snapshot(truth,idx,params,pack_cfg,cfg);
    item.electrothermal_discharge_current_A=item.discharge_current_A;
    item.electrothermal_charge_current_A=item.charge_current_A;
    item.fuse_discharge_current_cap_A=inf(size(item.discharge_current_A));
    item.fuse_charge_current_cap_A=inf(size(item.charge_current_A));
    item.combined_discharge_current_A=item.discharge_current_A;
    item.combined_charge_current_A=item.charge_current_A;
    item.combined_discharge_binding=repmat({'electrothermal'}, ...
        size(item.discharge_current_A));
    item.combined_charge_binding=repmat({'electrothermal'}, ...
        size(item.charge_current_A));
    if ~isempty(fuse_replay)
        [~,fi]=min(abs(fuse_replay.time_s-truth.time_s(idx)));
        fuse_cap=double(fuse_replay.reference_cap_A(fi,:));
        fuse_charge_cap=double(fuse_replay.reference_charge_cap_A(fi,:));
        item.fuse_discharge_current_cap_A=fuse_cap;
        item.fuse_charge_current_cap_A=fuse_charge_cap;
        item.combined_discharge_current_A=min(item.discharge_current_A,fuse_cap);
        item.combined_charge_current_A=-min(abs(item.charge_current_A), ...
            fuse_charge_cap);
        bound=fuse_cap<item.discharge_current_A-1e-9;
        item.combined_discharge_binding(bound)={'fuse_reference'};
        charge_bound=fuse_charge_cap<abs(item.charge_current_A)-1e-9;
        item.combined_charge_binding(charge_bound)={'fuse_reference'};
    end
    items{k}=item;
end
if isempty(items)
    campaign=struct([]);
else
    % Build the structure array only after real records exist. MATLAB does
    % not permit indexed assignment of a populated struct into struct([]).
    % The cell staging also keeps checkpoint construction independent of
    % structure field ordering.
    campaign=vertcat(items{:});
end
end
