function cfg = ekf_init_minus20()
%EKF_INIT_MINUS20 DER26 MiL canonical scenario.
cfg=mil.default_config(); cfg.id='ekf_init_minus20'; cfg.description='EKF convergence from -20 percentage-point SoC initialization error with fixed-basis relaxation acquisition'; cfg.simulation_id='hppc_validation'; cfg.stop_time_s=120; cfg.production.estimator.precondition_rest_s=0.0; cfg.reference_ekf.initial_soc_offset=-0.20; cfg.reference_ekf.acquisition.enabled=true;
cfg.acceptance.ekf.acquisition_mode=true; cfg.requirements={'EKF-CONVERGENCE'}; cfg.tier='pr';
end
