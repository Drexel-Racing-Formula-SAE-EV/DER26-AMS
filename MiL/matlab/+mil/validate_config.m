function validate_config(cfg)
%VALIDATE_CONFIG Fail early on malformed or unsafe MiL configuration.
required = {'schema_version','id','seed','cell_id','pack_id','simulation_id', ...
    'plant','sensor','reference_ekf','sop_oracle','fuse','acceptance'};
for k = 1:numel(required)
    if ~isfield(cfg, required{k})
        error('mil:config:MissingField', 'Missing scenario field %s.', required{k});
    end
end
if cfg.schema_version ~= 1
    error('mil:config:Schema', 'Unsupported MiL schema version.');
end
if ~strcmpi(char(cfg.plant.variation_mode), 'parameter_distributed')
    error('mil:config:PlantMode', ...
        'MiL qualification requires parameter_distributed plant mode.');
end
if ~(isfinite(cfg.initial_soc) && cfg.initial_soc >= 0 && cfg.initial_soc <= 1)
    error('mil:config:InitialSoc', 'initial_soc must be in [0,1].');
end
if ~(isfinite(cfg.initial_temperature_C) && isfinite(cfg.ambient_temperature_C))
    error('mil:config:Temperature', 'Initial/ambient temperatures must be finite.');
end
if isfield(cfg,'production') && isfield(cfg.production,'estimator') && ...
        isfield(cfg.production.estimator,'precondition_rest_s')
    pre_s=double(cfg.production.estimator.precondition_rest_s);
    if ~(isscalar(pre_s) && isfinite(pre_s) && pre_s>=0)
        error('mil:config:EstimatorPrecondition', ...
            'production.estimator.precondition_rest_s must be finite and nonnegative.');
    end
end
if isfield(cfg.acceptance,'ekf')
    applicability={'convergence_required','r0_accuracy_required','r0_unobservable_drift_required'};
    for q=1:numel(applicability)
        field=applicability{q};
        if isfield(cfg.acceptance.ekf,field)
            value=cfg.acceptance.ekf.(field);
            if ~(isscalar(value) && (islogical(value) || ...
                    (isnumeric(value) && isfinite(value) && ...
                     (value==0 || value==1))))
                error('mil:config:EkfApplicability', ...
                    'acceptance.ekf.%s must be scalar logical/0/1.',field);
            end
        end
    end
    if isfield(cfg.acceptance.ekf,'r0_min_observations')
        value=cfg.acceptance.ekf.r0_min_observations;
        if ~(isscalar(value) && isnumeric(value) && isfinite(value) && ...
                value>=1 && value==floor(value))
            error('mil:config:EkfR0MinimumObservations', ...
                'acceptance.ekf.r0_min_observations must be a positive integer.');
        end
    end
end

if cfg.sop_oracle.enabled
    h = double(cfg.sop_oracle.horizons_s(:));
    if numel(h) ~= 4 || any(~isfinite(h)) || any(diff(h) <= 0)
        error('mil:config:SoPHorizons', 'SoP oracle requires four increasing horizons.');
    end
end
if ~(isfinite(cfg.sensor.current.sample_time_s) && ...
        cfg.sensor.current.sample_time_s > 0 && ...
        cfg.sensor.current.sample_time_s <= 0.1)
    error('mil:config:CurrentSampleTime', ...
        'Hall-current sample time must be in (0, 0.1] s.');
end
if cfg.fuse.enabled && ~ismember(lower(string(cfg.fuse.startup_policy)), ...
        ["cold-soak","known-cold"])
    error('mil:config:FuseStartup','Unsupported fuse startup policy.');
end
if isfield(cfg.acceptance,'acquisition_fault_recovery') && ...
        cfg.acceptance.acquisition_fault_recovery.enabled
    afr=cfg.acceptance.acquisition_fault_recovery;
    if ~(isscalar(afr.target_segment) && isfinite(afr.target_segment) && ...
            afr.target_segment>=1 && afr.target_segment==floor(afr.target_segment))
        error('mil:config:AcquisitionFaultRecoverySegment', ...
            'acquisition fault-recovery target_segment must be a positive integer.');
    end
    if ~(isscalar(afr.fault_end_s) && isfinite(afr.fault_end_s) && afr.fault_end_s>=0)
        error('mil:config:AcquisitionFaultRecoveryTime', ...
            'acquisition fault-recovery fault_end_s must be finite and nonnegative.');
    end
    if ~isfield(cfg,'production') || ~cfg.production.estimator.enabled
        error('mil:config:AcquisitionFaultRecoveryProduction', ...
            'acquisition fault-recovery scoring requires production estimator execution.');
    end
end
end
