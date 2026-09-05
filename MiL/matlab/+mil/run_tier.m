function campaign = run_tier(tier,varargin)
%RUN_TIER Execute cumulative PR/nightly/release deterministic inventory.
tier=lower(string(tier));ranks=struct('pr',1,'nightly',2,'release',3);
if ~isfield(ranks,char(tier)),error('mil:campaign:Tier','Unknown campaign tier %s.',tier);end
catalog=mil.scenario_catalog();names={};
for k=1:numel(catalog)
    item=lower(string(catalog(k).tier));
    if isfield(ranks,char(item)) && ranks.(char(item))<=ranks.(char(tier))
        names{end+1}=catalog(k).id; %#ok<AGROW>
    end
end
mil.preflight_campaign(names,char(tier));
campaign=mil.run_campaign(names,varargin{:});
end
