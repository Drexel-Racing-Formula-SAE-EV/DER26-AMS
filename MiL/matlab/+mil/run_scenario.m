function result = run_scenario(scenario,varargin)
%RUN_SCENARIO Execute one deterministic DER26 distributed-plant MiL case.
%
% This runner deliberately reuses HiL/simulink only as the checked-in battery
% plant/parameter library. It does not invoke the real-time HIL target.
p=inputParser();
p.addParameter('OutputDirectory','',@(x)ischar(x)||isstring(x));
p.addParameter('Export',true,@(x)islogical(x)&&isscalar(x));
p.addParameter('RunSoP',[],@(x)islogical(x)&&isscalar(x)||isempty(x));
p.parse(varargin{:}); opt=p.Results;

if ischar(scenario)||isstring(scenario)
    cfg=mil.load_scenario(string(scenario));
else
    cfg=scenario; mil.validate_config(cfg);
end
paths=mil.project_paths();
if isempty(which('hil.run_reference'))
    addpath(paths.hil_simulink); addpath(paths.hil_profiles); addpath(paths.hil_adapters);
end

cell_cfg=hil.load_configuration('cell',cfg.cell_id);
pack_cfg=hil.load_configuration('pack',cfg.pack_id);
sim_cfg=hil.load_configuration('simulation',cfg.simulation_id);

% Scenario owns physical variation. Legacy plant-side sensor/fault injection
% is disabled so there is exactly one AMS measurement bus in the MiL.
pack_cfg.imbalance_model.mode=char(cfg.plant.variation_mode);
pack_cfg.imbalance_model.seed=double(cfg.plant.seed);
pack_cfg.imbalance_model.overrides=cfg.plant.overrides;
pack_cfg.initial_soc=double(cfg.initial_soc);
pack_cfg.initial_temperature_C=double(cfg.initial_temperature_C);
pack_cfg.cooling_boundary.Rsa_multiplier=double(cfg.plant.cooling_Rsa_multiplier);
sim_cfg.initial_soc=double(cfg.initial_soc);
sim_cfg.ambient_temperature_C=double(cfg.ambient_temperature_C);
sim_cfg.output_decimation=1;
sim_cfg.measurement_noise.voltage_std_V=0;
sim_cfg.measurement_noise.current_std_A=0;
sim_cfg.measurement_noise.temperature_std_C=0;
sim_cfg.current_bias_A=0;
sim_cfg.fault_injection=struct('enabled',false);
sim_cfg.store_group_truth=true;
sim_cfg.engine='reference';
if isfield(cfg,'sample_time_s')&&~isempty(cfg.sample_time_s),sim_cfg.sample_time_s=double(cfg.sample_time_s);end
if isnumeric(cfg.stop_time_s),sim_cfg.stop_time_s=double(cfg.stop_time_s);else,sim_cfg.stop_time_s=cfg.stop_time_s;end

[params,param_report]=hil.build_parameters(cell_cfg,'Save',false);
profile=mil.resolve_profile(cfg,sim_cfg,cell_cfg,pack_cfg);
plant=hil.run_reference(cell_cfg,pack_cfg,sim_cfg,params,profile);
truth=mil.build_truth_bus(plant,params);
meas=mil.build_measurement_bus(truth,cfg.sensor,plant.pack_configuration);

result=struct();
result.schema_version=1;
result.scenario=cfg;
result.scenario_id=cfg.id;
result.seed=cfg.seed;
result.parameter_hash=plant.parameter_hash;
result.plant_configuration_hash=plant.configuration_hash;
result.mil_configuration_hash=hil.configuration_hash(cfg,plant.parameter_hash);
result.parameter_source=param_report.mode;
result.plant=plant;
result.truth=truth;
result.measurement=meas;
result.reference=struct();
result.metrics=struct();
result.truth.soh=mil.soh_truth(truth,plant.pack_configuration);

initial_soc_for_estimators=truth.segment_soc(1,:).' + ...
    double(cfg.reference_ekf.initial_soc_offset);
if cfg.reference_ekf.enabled
    ref_cfg=cfg.reference_ekf;
    ref_cfg.initial_soc=initial_soc_for_estimators;
    result.reference.ekf=mil.reference.run_segment_ekf( ...
        meas,params,plant.pack_configuration,ref_cfg);
    result.metrics.ekf=mil.metrics.ekf(result.reference.ekf,truth,cfg.acceptance.ekf);
    result.reference.soh_capacity=mil.reference.soh_capacity(meas,result.reference.ekf, ...
        plant.pack_configuration,double(params.Q_nom)*double(plant.pack_configuration.parallel_cells));
    result.metrics.soh=mil.metrics.soh(result.reference.soh_capacity,result.truth.soh);
end

result.production=struct();
if isfield(cfg,'production') && cfg.production.estimator.enabled
    precondition_rest_s=0.0;
    if isfield(cfg.production.estimator,'precondition_rest_s')
        precondition_rest_s=double(cfg.production.estimator.precondition_rest_s);
    end
    result.production.ekf=mil.production.run_estimator( ...
        meas,initial_soc_for_estimators,'PreconditionRestS',precondition_rest_s);
    result.metrics.production_ekf=mil.metrics.production_ekf( ...
        result.production.ekf,truth,cfg.acceptance.ekf);
    if isfield(cfg.acceptance,'acquisition_fault_recovery') && ...
            cfg.acceptance.acquisition_fault_recovery.enabled
        afr=cfg.acceptance.acquisition_fault_recovery;
        result.metrics.acquisition_fault_recovery= ...
            mil.metrics.production_acquisition_fault_recovery( ...
                result.production.ekf,truth,result.metrics.production_ekf, ...
                afr.target_segment,afr.fault_end_s,cfg.acceptance.ekf);
    end
    if cfg.production.soh.enabled
        result.production.soh=mil.production.run_soh(meas,result.production.ekf);
        result.metrics.production_soh=mil.metrics.production_soh( ...
            result.production.soh,result.truth.soh,cfg.acceptance.soh);
    end
end

result.fuse_replay=struct([]);
if isfield(cfg,'fuse') && cfg.fuse.enabled
    result.fuse_replay=mil.fuse.run_replay(truth,cfg.fuse);
    result.metrics.fuse=mil.metrics.fuse(result.fuse_replay,cfg.acceptance.fuse);
end

run_sop=cfg.sop_oracle.enabled;
if ~isempty(opt.RunSoP),run_sop=opt.RunSoP;end
if run_sop
    result.oracle.sop=mil.oracle.sop_campaign(truth,params,plant.pack_configuration, ...
        cfg.sop_oracle,result.fuse_replay);
else
    result.oracle.sop=struct([]);
end
if run_sop && isfield(result.production,'ekf') && ...
        isfield(cfg,'production') && cfg.production.sop.enabled
    result.production.sop=mil.production.run_sop( ...
        meas,result.production.ekf,plant.pack_configuration,cfg.sop_oracle);
    oracle_discharge=vertcat(result.oracle.sop.discharge_current_A);
    oracle_charge=abs(vertcat(result.oracle.sop.charge_current_A));
    result.metrics.sop_discharge=mil.metrics.sop( ...
        result.production.sop.model_discharge_current_A,oracle_discharge,cfg.acceptance.sop);
    result.metrics.sop_charge=mil.metrics.sop( ...
        abs(result.production.sop.model_charge_current_A),oracle_charge,cfg.acceptance.sop);
end
result.metrics.faults=mil.metrics.fault_behavior(meas,cfg);
result.metrics.numeric=mil.metrics.numeric_health(result);
result.summary=mil.summarize_result(result);

if opt.Export
    out=char(opt.OutputDirectory);
    if isempty(out),out=fullfile(paths.output_root,cfg.id);end
    result.artifacts=mil.export_result(result,out);
else
    result.artifacts=struct();
end
end
