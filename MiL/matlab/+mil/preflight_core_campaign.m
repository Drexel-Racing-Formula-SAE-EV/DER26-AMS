function report = preflight_core_campaign()
%PREFLIGHT_CORE_CAMPAIGN Resolve and structurally validate C0-C8 up front.
report=mil.preflight_campaign(mil.core_campaign(),'core');
end
