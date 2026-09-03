function exe = estimator_runner_path(varargin)
%ESTIMATOR_RUNNER_PATH Locate/build the host executable linked to production C.
%
% On Windows the repo is typically built with MSYS2/UCRT64. A stale host
% executable is dangerous here because MATLAB can otherwise read an older CSV
% schema while the wrapper expects newer production telemetry. Therefore the
% checked-in local runner is rebuilt on first use and whenever its source signature changes.
% Set DER26_MIL_ESTIMATOR_RUNNER to use an explicitly managed external runner.
p=inputParser();
p.addParameter('BuildIfMissing',true,@(x)islogical(x)&&isscalar(x));
p.parse(varargin{:});
paths=mil.project_paths();
override=getenv('DER26_MIL_ESTIMATOR_RUNNER');
using_override=~isempty(override);
if using_override
    exe=string(override);
    hostdir='';
else
    hostdir=fullfile(paths.mil_root,'host','production_estimator_runner');
    if ispc
        candidate=fullfile(hostdir,'build','production_estimator_runner.exe');
    else
        candidate=fullfile(hostdir,'build','production_estimator_runner');
    end
    exe=string(candidate);
end

if ispc && ~using_override && p.Results.BuildIfMissing
    build_script=fullfile(hostdir,'build_windows_msys2.cmd');
    deps={ ...
        fullfile(hostdir,'main.c'), ...
        fullfile(hostdir,'Makefile'), ...
        build_script, ...
        fullfile(paths.repo_root,'AMS','Core','Src','estimator','ams_soc_ekf.c'), ...
        fullfile(paths.repo_root,'AMS','Core','Inc','estimator','ams_soc_ekf.h'), ...
        fullfile(paths.repo_root,'AMS','Core','Src','estimator','ams_estimator_lut.c'), ...
        fullfile(paths.repo_root,'AMS','Core','Inc','estimator','ams_estimator_lut.h')};
    mil.production.ensure_windows_local_runner( ...
        exe,build_script,deps,'production estimator');
    return;
end

if isfile(exe),return;end
if ~p.Results.BuildIfMissing
    error('mil:production:RunnerMissing','Production estimator runner not found: %s',exe);
end
if ispc
    error('mil:production:RunnerMissing', ...
        ['Production estimator runner is not built. Build MiL/host/', ...
         'production_estimator_runner with MSYS2/UCRT64, or set ', ...
         'DER26_MIL_ESTIMATOR_RUNNER to the executable path.']);
end
hostdir=fileparts(fileparts(char(exe)));
cmd=sprintf('make -C %s all',shell_quote(hostdir));
[status,text]=system(cmd);
if status~=0 || ~isfile(exe)
    error('mil:production:RunnerBuild','Failed to build production estimator runner:\n%s',text);
end
end

function q=shell_quote(path)
%SHELL_QUOTE Quote one filesystem path for MATLAB's platform shell.
% MATLAB system() uses cmd.exe on Windows, where POSIX single-quote quoting is
% not recognized. Use double quotes there; retain POSIX quoting elsewhere.
path=char(path);
if ispc
    q=['"' strrep(path,'"','""') '"'];
else
    q=['''' strrep(path,'''','''"''"''') ''''];
end
end
