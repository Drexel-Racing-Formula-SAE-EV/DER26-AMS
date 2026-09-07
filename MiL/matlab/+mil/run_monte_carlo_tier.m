function campaign = run_monte_carlo_tier(base_scenario,tier,varargin)
%RUN_MONTE_CARLO_TIER Run the compressed 32/256/1000 seeded campaign sizes.
switch lower(string(tier))
    case "pr",count=32;
    case "nightly",count=256;
    case {"release","qualification"},count=1000;
    otherwise,error('mil:mc:Tier','Unknown Monte Carlo tier %s.',tier);
end
campaign=mil.monte_carlo(base_scenario,count,varargin{:});
campaign.tier=char(tier);
end
