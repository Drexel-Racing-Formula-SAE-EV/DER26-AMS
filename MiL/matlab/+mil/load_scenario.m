function cfg = load_scenario(name)
%LOAD_SCENARIO Load and validate one named MiL scenario function.
arguments
    name (1,1) string
end
paths = mil.project_paths();
scenario_file = fullfile(paths.scenarios, [char(name) '.m']);
if ~isfile(scenario_file)
    error('mil:scenario:Missing', 'Unknown MiL scenario "%s".', name);
end
if isempty(which(char(name)))
    addpath(paths.scenarios);
end
builder = str2func(char(name));
cfg = builder();
mil.validate_config(cfg);
end
