function ensure_windows_local_runner(exe,build_script,dependencies,label)
%ENSURE_WINDOWS_LOCAL_RUNNER Rebuild a local MSYS2 runner when sources change.
%
% The first use in every MATLAB function session rebuilds the local runner.
% Subsequent uses are free unless one of the checked-in dependency files
% changes. This prevents overlay-extracted source updates from silently using
% a stale .exe whose CSV schema happens to remain compatible.
if ~ispc
    return;
end
exe=char(exe);build_script=char(build_script);
if nargin<4||strlength(string(label))==0,label='production';end
signature=mil.production.host_runner_signature(dependencies);
persistent built_signatures
if isempty(built_signatures)
    built_signatures=containers.Map('KeyType','char','ValueType','char');
end
key=lower(strrep(exe,'/','\'));
needs_build=~isfile(exe) || ~isKey(built_signatures,key) || ...
    ~strcmp(built_signatures(key),signature);
if ~needs_build
    return;
end
if ~isfile(build_script)
    error('mil:production:RunnerBuildHelperMissing', ...
        'Windows %s host-runner build helper missing: %s',label,build_script);
end
[status,text]=system(shell_quote(build_script));
if status~=0 || ~isfile(exe)
    error('mil:production:RunnerBuild', ...
        ['Failed to rebuild the Windows %s host runner.\n', ...
         'A current local executable is required for production-C parity.\n', ...
         'Build helper: %s\nOutput:\n%s'],label,build_script,text);
end
% Recompute after the build in case a generator touched any dependency.
built_signatures(key)=mil.production.host_runner_signature(dependencies);
end

function q=shell_quote(path)
path=char(path);
q=['"' strrep(path,'"','""') '"'];
end
