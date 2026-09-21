REPO_ROOT := $(abspath ..)
.PHONY: help static profile-assets soh-profile-contracts acquisition-math acquisition-policy production-acquisition-policy covariance-sop-policy traceability audit host-runners estimator-runner sop-runner soh-runner ams-unit fuse-oracle fuse-smoke check clean
help:
	@echo "DER26 AMS MiL verification tooling"
	@echo "  make static        Architecture/source integrity checks"
	@echo "  make host-runners  Build and self-test production-C adapters"
	@echo "  make check         Static + runners + focused AMS/fuse regressions"
	@echo "  MATLAB scenarios are executed separately with matlab/scripts/setup_mil.m"
static:
	python3 tests/run_static_checks.py
profile-assets:
	python3 tests/test_profile_assets.py
soh-profile-contracts:
	python3 tests/test_soh_capacity_profiles.py
acquisition-math:
	python3 tests/test_fixed_basis_acquisition.py
acquisition-policy:
	python3 tests/test_acquisition_shadow_policy.py
production-acquisition-policy:
	python3 tests/test_production_acquisition_policy.py
covariance-sop-policy:
	python3 tests/test_covariance_sop_safety_policy.py
traceability:
	python3 tools/generate_traceability.py --check
audit:
	python3 tools/audit_release_inputs.py --check
estimator-runner:
	$(MAKE) -C host/production_estimator_runner clean all test
sop-runner:
	$(MAKE) -C host/production_sop_runner clean all
soh-runner:
	$(MAKE) -C host/production_soh_runner clean all test
host-runners: estimator-runner sop-runner soh-runner
ams-unit:
	$(MAKE) -C $(REPO_ROOT)/AMS/host_tests unit
fuse-oracle:
	$(MAKE) -C $(REPO_ROOT)/AMS/host_tests fuse-oracle
fuse-smoke:
	$(MAKE) -C $(REPO_ROOT)/Tools/fuse_replay smoke-ci
check: static profile-assets soh-profile-contracts acquisition-math acquisition-policy production-acquisition-policy covariance-sop-policy traceability audit host-runners ams-unit fuse-oracle fuse-smoke
clean:
	$(MAKE) -C host/production_estimator_runner clean
	$(MAKE) -C host/production_sop_runner clean
	$(MAKE) -C host/production_soh_runner clean
