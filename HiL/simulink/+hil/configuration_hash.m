function digest = configuration_hash(varargin)
%CONFIGURATION_HASH Return a deterministic SHA-256 hash for MATLAB values.

canonical = cell(size(varargin));
for index = 1:nargin
    canonical{index} = local_canonicalize(varargin{index});
end

payload = jsonencode(canonical);
digest = local_sha256(uint8(unicode2native(payload, 'UTF-8')));
end

function value = local_canonicalize(value)
if isstruct(value)
    names = sort(fieldnames(value));
    runtime_only = { ...
        'config_file', 'output_directory', ...
        'legacy_snapshot_file', 'root'};
    ordered = struct();
    for element_index = 1:numel(value)
        item = struct();
        for field_index = 1:numel(names)
            name = names{field_index};
            if ismember(name, runtime_only)
                continue;
            end
            field_value = value(element_index).(name);
            if strcmp(name, 'source_file') && ...
                    (ischar(field_value) || isstring(field_value)) && ...
                    isfile(field_value)
                [~, base, extension] = fileparts(char(field_value));
                item.(name) = struct( ...
                    'name', [base, extension], ...
                    'sha256', hil.file_sha256(field_value));
            else
                item.(name) = local_canonicalize(field_value);
            end
        end
        if element_index == 1
            ordered = item;
        else
            ordered(element_index) = item; %#ok<AGROW>
        end
    end
    value = ordered;
elseif iscell(value)
    for index = 1:numel(value)
        value{index} = local_canonicalize(value{index});
    end
elseif isa(value, 'function_handle')
    value = func2str(value);
elseif isdatetime(value)
    value = char(value, 'yyyy-MM-dd''T''HH:mm:ss.SSSXXX');
elseif isstring(value)
    value = cellstr(value);
end
end

function digest = local_sha256(bytes)
if ~usejava('jvm')
    error('hil:hash:NoJVM', 'SHA-256 hashing requires the MATLAB JVM.');
end
engine = java.security.MessageDigest.getInstance('SHA-256');
engine.update(typecast(uint8(bytes(:)), 'int8'));
raw = typecast(int8(engine.digest()), 'uint8');
digest = lower(reshape(dec2hex(raw, 2).', 1, []));
end
