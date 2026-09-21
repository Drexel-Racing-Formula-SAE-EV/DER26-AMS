function cfg = c1_hppc_bad_init()
%C1_HPPC_BAD_INIT HPPC estimator convergence/R0 canonical case.
cfg=mil.default_config();cfg.id='c1_hppc_bad_init';
cfg.description='C1: 120 s HPPC with +20 percentage-point initial SoC error and fixed-basis relaxation acquisition';
cfg.simulation_id='hppc_validation';cfg.initial_soc=0.60;cfg.stop_time_s=120;
cfg.production.estimator.precondition_rest_s=0.0;
cfg.reference_ekf.initial_soc_offset=0.20;
cfg.reference_ekf.acquisition.enabled=true;
cfg.acceptance.ekf.acquisition_mode=true;
cfg.acceptance.ekf.r0_accuracy_required=true;
cfg.acceptance.ekf.r0_unobservable_drift_required=true;
cfg.sop_oracle.enabled=false;
cfg.requirements={'C1-HPPC','EKF-ACQUISITION','EKF-COVARIANCE','EKF-CONVERGENCE','EKF-R0','EKF-NIS','NUMERIC'};
cfg.tier='pr';
end
