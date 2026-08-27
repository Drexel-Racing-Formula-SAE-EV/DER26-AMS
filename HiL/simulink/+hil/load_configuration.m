function cfg = load_configuration(kind, name)
%LOAD_CONFIGURATION Load one named project configuration.

kind = lower(char(kind));
name = lower(char(name));

valid_kinds = {'cell', 'pack', 'simulation', 'dataset', 'acceptance'};
if ~ismember(kind, valid_kinds)
    error('hil:config:UnknownKind', 'Unknown configuration kind "%s".', kind);
end

if isempty(regexp(name, '^[a-z][a-z0-9_]*$', 'once'))
    error('hil:config:InvalidName', ...
        'Configuration name "%s" is not a valid MATLAB identifier.', name);
end

paths = hil.project_paths();
folder = fullfile(paths.configs, [kind 's']);
config_file = fullfile(folder, [name '.m']);

if exist(config_file, 'file') ~= 2
    error('hil:config:NotFound', ...
        'Could not find %s configuration "%s" at %s.', ...
        kind, name, config_file);
end

old_path = path;
cleanup = onCleanup(@() path(old_path)); %#ok<NASGU>
addpath(folder, '-begin');

factory = str2func(name);
cfg = factory();

if ~isstruct(cfg) || ~isscalar(cfg)
    error('hil:config:InvalidFactory', ...
        'Configuration function %s must return one scalar struct.', config_file);
end

cfg.kind = kind;
cfg.config_name = name;
cfg.config_file = config_file;
[cfg, ~] = hil.validate_configuration(cfg);
end
