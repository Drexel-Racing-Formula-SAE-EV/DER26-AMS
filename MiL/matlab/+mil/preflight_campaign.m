function report = preflight_campaign(names,label)
%PREFLIGHT_CAMPAIGN Resolve/validate profiles and structural observability.
%
% This is intentionally a necessary-condition preflight. It catches campaign
% definitions that cannot possibly exercise the production acquisition or SoH
% contracts before expensive plant/production-C execution starts. Runtime
% estimator confidence, innovation, polarization, temperature and measurement
% validity remain authoritative; preflight never substitutes for those gates.
if nargin<2||strlength(string(label))==0,label='campaign';end
if isstring(names),names=cellstr(names);end

paths=mil.project_paths();
if isempty(which('hil.load_profile'))
    addpath(paths.hil_simulink);addpath(paths.hil_profiles);addpath(paths.hil_adapters);
end

items=repmat(struct( ...
    'scenario_id','','profile_name','','samples',0,'duration_s',0, ...
    'minimum_current_A',0,'maximum_current_A',0, ...
    'estimator_precondition_rest_s',0,'longest_acquisition_rest_s',0, ...
    'r0_observable_step_count',0,'capacity_qualified_rest_count',0, ...
    'capacity_structural_excursion_count',0,'capacity_effective_Ah',NaN, ...
    'capacity_anchor_soc_min',NaN,'capacity_anchor_soc_max',NaN, ...
    'capacity_confidence_anchor_count',0, ...
    'capacity_confidence_excursion_count',0),numel(names),1);

for k=1:numel(names)
    cfg=mil.load_scenario(string(names{k}));
    cell_cfg=hil.load_configuration('cell',cfg.cell_id);
    pack_cfg=hil.load_configuration('pack',cfg.pack_id);
    sim_cfg=hil.load_configuration('simulation',cfg.simulation_id);
    sim_cfg.initial_soc=double(cfg.initial_soc);
    sim_cfg.ambient_temperature_C=double(cfg.ambient_temperature_C);
    if isfield(cfg,'sample_time_s')&&~isempty(cfg.sample_time_s),sim_cfg.sample_time_s=double(cfg.sample_time_s);end
    if isnumeric(cfg.stop_time_s),sim_cfg.stop_time_s=double(cfg.stop_time_s);else,sim_cfg.stop_time_s=cfg.stop_time_s;end

    profile=mil.resolve_profile(cfg,sim_cfg,cell_cfg,pack_cfg);
    t=double(profile.time_s(:));i=double(profile.pack_current_A(:));a=double(profile.ambient_temperature_C(:));
    if numel(t)<2||numel(t)~=numel(i)||numel(t)~=numel(a)
        error('mil:preflight:ProfileShape','Scenario %s resolved an invalid profile shape.',cfg.id);
    end
    if any(~isfinite(t))||any(~isfinite(i))||any(~isfinite(a))||any(diff(t)<=0)
        error('mil:preflight:ProfileNumeric','Scenario %s resolved non-finite or non-monotonic profile data.',cfg.id);
    end
    dt=double(sim_cfg.sample_time_s);
    if max(abs(diff(t)-dt))>1e-8
        error('mil:preflight:ProfileSampleTime', ...
            'Scenario %s profile does not match %.9g s simulation sample time.',cfg.id,dt);
    end

    items(k).scenario_id=char(cfg.id);items(k).profile_name=char(profile.name);
    items(k).samples=numel(t);items(k).duration_s=t(end)-t(1);
    items(k).minimum_current_A=min(i);items(k).maximum_current_A=max(i);

    pre_s=0.0;
    if isfield(cfg.production.estimator,'precondition_rest_s'),pre_s=double(cfg.production.estimator.precondition_rest_s);end
    if pre_s>0&&pre_s<20.0
        error('mil:preflight:EstimatorPrecondition', ...
            'Scenario %s estimator precondition %.3g s is shorter than the 20 s production acquisition window.',cfg.id,pre_s);
    end
    items(k).estimator_precondition_rest_s=pre_s;

    acq_runs=logical_runs(abs(i)<=0.5,dt);
    if isempty(acq_runs),longest_acq=0;else,longest_acq=max([acq_runs.duration_s]);end
    items(k).longest_acquisition_rest_s=longest_acq;
    ekf_gate=isfield(cfg.gates,'ekf')&&cfg.gates.ekf;
    if cfg.production.estimator.enabled&&ekf_gate&&pre_s==0&&longest_acq<20.0-1e-9
        error('mil:preflight:EstimatorAcquisitionUnobservable', ...
            ['Scenario %s gates the production EKF but has no >=20 s |I|<=0.5 A ', ...
             'acquisition window and no estimator precondition.'],cfg.id);
    end

    % Production resistance observer: each accepted R0 update needs |I|>=20 A
    % and a >=5 A current transition; 50 accepted observations are required
    % before the estimator publishes ADVISORY_VALID resistance SoH.
    di=[0;abs(diff(i))];r0_steps=nnz(abs(i)>=20.0&di>=5.0);
    items(k).r0_observable_step_count=r0_steps;
    r0_accuracy_required=isfield(cfg.acceptance,'ekf')&& ...
        isfield(cfg.acceptance.ekf,'r0_accuracy_required')&& ...
        logical(cfg.acceptance.ekf.r0_accuracy_required);
    r0_min_observations=1;
    if isfield(cfg.acceptance,'ekf')&&isfield(cfg.acceptance.ekf,'r0_min_observations')
        r0_min_observations=double(cfg.acceptance.ekf.r0_min_observations);
    end
    if r0_accuracy_required&&r0_steps<r0_min_observations
        error('mil:preflight:EkfR0Unobservable', ...
            ['Scenario %s has only %d structurally observable raw R0 steps; ', ...
             'EKF-R0 requires at least %d.'],cfg.id,r0_steps,r0_min_observations);
    end

    resistance_gate=isfield(cfg.gates,'soh')&&cfg.gates.soh&& ...
        isfield(cfg.gates,'soh_resistance')&&cfg.gates.soh_resistance;
    if resistance_gate&&r0_steps<50
        error('mil:preflight:SohResistanceUnobservable', ...
            'Scenario %s has only %d structurally observable R0 steps; production SoH requires at least 50.',cfg.id,r0_steps);
    end

    % Capacity observer needs three qualified rest anchors to produce two
    % accepted rest-to-rest windows. At profile level, exact-zero current is
    % the necessary current condition because run_soh supplies 0.5 A current
    % uncertainty against the production 0.5 A rest-current bound.
    %
    % v2.6.7 also checks nominal anchor SoC when a scenario supplies
    % preflight.capacity_confidence_soc_windows. This is deliberately a MiL
    % qualification contract, not a production firmware threshold. The current
    % C5/release capacity cases use bands demonstrated by a licensed 25 C
    % production-C stationary covariance sweep (5/10/98 %% passed the 1.5 %%
    % SoC-sigma rest gate; 15-95 %% did not).
    rest_runs=logical_runs(abs(i)<=1e-12,dt);
    qualified=rest_runs([rest_runs.duration_s]>=60.0-1e-9);
    items(k).capacity_qualified_rest_count=numel(qualified);

    effective_ah=effective_pack_capacity_ah(cfg,cell_cfg,pack_cfg);
    items(k).capacity_effective_Ah=effective_ah;
    charge_before_as=[0;cumsum(i(1:end-1))*dt];
    nominal_soc=double(cfg.initial_soc)-charge_before_as/(3600.0*effective_ah);
    anchor_soc=zeros(numel(qualified),1);
    for r=1:numel(qualified)
        anchor_soc(r)=nominal_soc(qualified(r).start_index);
    end
    if ~isempty(anchor_soc)
        items(k).capacity_anchor_soc_min=min(anchor_soc);
        items(k).capacity_anchor_soc_max=max(anchor_soc);
    end

    excursion_count=0;
    structural_excursion=false(max(0,numel(qualified)-1),1);
    for r=1:max(0,numel(qualified)-1)
        idx0=qualified(r).end_index;idx1=qualified(r+1).start_index;
        if idx1>idx0
            net_ah=abs(sum(i(idx0:idx1-1))*dt)/3600.0;
        else
            net_ah=0.0;
        end
        delta_soc=abs(anchor_soc(r+1)-anchor_soc(r));
        structural_excursion(r)=net_ah>=3.0-1e-9&&delta_soc>=0.15-1e-9;
        if structural_excursion(r),excursion_count=excursion_count+1;end
    end
    items(k).capacity_structural_excursion_count=excursion_count;

    confidence_anchor=false(numel(qualified),1);
    confidence_excursion_count=0;
    windows=[];
    if isfield(cfg,'preflight')&&isfield(cfg.preflight,'capacity_confidence_soc_windows')
        windows=double(cfg.preflight.capacity_confidence_soc_windows);
        if size(windows,2)~=2||isempty(windows)||any(~isfinite(windows(:)))|| ...
                any(windows(:,1)<0)||any(windows(:,2)>1)||any(windows(:,1)>windows(:,2))
            error('mil:preflight:SohCapacityConfidenceWindows', ...
                'Scenario %s has invalid capacity confidence SoC windows.',cfg.id);
        end
        for r=1:numel(anchor_soc)
            confidence_anchor(r)=any(anchor_soc(r)>=windows(:,1)-1e-9& ...
                anchor_soc(r)<=windows(:,2)+1e-9);
        end
        for r=1:numel(structural_excursion)
            if structural_excursion(r)&&confidence_anchor(r)&&confidence_anchor(r+1)
                confidence_excursion_count=confidence_excursion_count+1;
            end
        end
    end
    items(k).capacity_confidence_anchor_count=nnz(confidence_anchor);
    items(k).capacity_confidence_excursion_count=confidence_excursion_count;

    capacity_gate=isfield(cfg.gates,'soh')&&cfg.gates.soh&& ...
        isfield(cfg.gates,'soh_capacity')&&cfg.gates.soh_capacity;
    if capacity_gate&&(numel(qualified)<3||excursion_count<2)
        error('mil:preflight:SohCapacityUnobservable', ...
            ['Scenario %s cannot structurally produce two capacity windows: ', ...
             '%d >=60 s rest anchors, %d >=3 Ah / >=15 %%SoC rest-to-rest excursions.'], ...
            cfg.id,numel(qualified),excursion_count);
    end
    if capacity_gate&&~isempty(windows)&& ...
            (nnz(confidence_anchor)<3||confidence_excursion_count<2)
        error('mil:preflight:SohCapacityConfidenceUnobservable', ...
            ['Scenario %s has structurally valid capacity rests/excursions but ', ...
             'its nominal rest anchors do not provide two transitions between ', ...
             'configured production-confidence SoC windows: %d confidence anchors, ', ...
             '%d confidence excursions.'],cfg.id,nnz(confidence_anchor),confidence_excursion_count);
    end
end

report=struct('pass',true,'scenario_count',numel(names),'items',items);
fprintf('[MiL] %s preflight: %d/%d profiles resolved and validated\n', ...
    char(label),numel(names),numel(names));
end

function runs=logical_runs(mask,dt)
mask=logical(mask(:));edges=diff([false;mask;false]);starts=find(edges==1);ends=find(edges==-1)-1;
runs=repmat(struct('start_index',0,'end_index',0,'duration_s',0),numel(starts),1);
for k=1:numel(starts)
    runs(k).start_index=starts(k);runs(k).end_index=ends(k);
    runs(k).duration_s=(ends(k)-starts(k)+1)*dt;
end
end

function capacity_ah=effective_pack_capacity_ah(cfg,cell_cfg,pack_cfg)
% Qualification-only aggregate capacity estimate used for nominal anchor SoC.
% Explicit per-group capacity overrides are included; zero-mean distributed
% Monte Carlo variation is intentionally not treated as known preflight truth.
capacity_ah=double(cell_cfg.nominal_capacity_Ah)*double(pack_cfg.parallel_cells);
mult=ones(double(pack_cfg.series_groups),1);
if isfield(cfg,'plant')&&isfield(cfg.plant,'overrides')&&~isempty(cfg.plant.overrides)
    overrides=cfg.plant.overrides;
    for q=1:numel(overrides)
        if ~isfield(overrides(q),'capacity_multiplier')|| ...
                isempty(overrides(q).capacity_multiplier)
            continue;
        end
        idx=double(overrides(q).group_index);
        value=double(overrides(q).capacity_multiplier);
        if ~(isscalar(idx)&&isfinite(idx)&&idx>=1&&idx<=numel(mult)&&idx==floor(idx)&& ...
                isscalar(value)&&isfinite(value)&&value>0)
            error('mil:preflight:SohCapacityOverride', ...
                'Scenario %s has an invalid capacity override.',cfg.id);
        end
        mult(idx)=value;
    end
end
capacity_ah=capacity_ah*mean(mult);
if ~(isscalar(capacity_ah)&&isfinite(capacity_ah)&&capacity_ah>0)
    error('mil:preflight:SohCapacityEffective', ...
        'Scenario %s produced an invalid effective pack capacity.',cfg.id);
end
end
