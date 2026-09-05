function campaign = run_core_campaign(varargin)
%RUN_CORE_CAMPAIGN Execute the frozen compact C0-C8 deterministic suite.
mil.preflight_core_campaign();
campaign=mil.run_campaign(mil.core_campaign(),varargin{:});
end
