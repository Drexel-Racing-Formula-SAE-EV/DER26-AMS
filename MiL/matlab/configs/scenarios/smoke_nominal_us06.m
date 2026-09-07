function cfg = smoke_nominal_us06()
%SMOKE_NOMINAL_US06 DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='smoke_nominal_us06'; cfg.description='Nominal 25C distributed US06 smoke/regression'; cfg.production.estimator.precondition_rest_s=30.0; cfg.requirements={'EKF-NOMINAL','NUMERIC'}; cfg.tier='pr';
end
