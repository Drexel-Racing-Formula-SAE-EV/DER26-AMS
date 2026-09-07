function catalog = scenario_catalog()
%SCENARIO_CATALOG Load deterministic scenario metadata without running plants.
paths=mil.project_paths();files=dir(fullfile(paths.scenarios,'*.m'));
catalog=struct('id',{},'tier',{},'description',{},'requirements',{},'file',{});
for k=1:numel(files)
    [~,name]=fileparts(files(k).name);cfg=mil.load_scenario(string(name));
    tier='development';if isfield(cfg,'tier'),tier=char(cfg.tier);end
    catalog(end+1)=struct('id',char(cfg.id),'tier',tier, ... %#ok<AGROW>
        'description',char(cfg.description),'requirements',{cfg.requirements}, ...
        'file',fullfile(files(k).folder,files(k).name));
end
[~,order]=sort(string({catalog.id}));catalog=catalog(order);
end
