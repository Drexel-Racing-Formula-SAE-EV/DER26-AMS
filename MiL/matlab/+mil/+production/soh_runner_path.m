function exe = soh_runner_path(varargin)
%SOH_RUNNER_PATH Locate/build direct checked-in ams_soh.c host runner.
p=inputParser();
p.addParameter('BuildIfMissing',true,@(x)islogical(x)&&isscalar(x));
p.parse(varargin{:});
paths=mil.project_paths();
override=getenv('DER26_MIL_SOH_RUNNER');
using_override=~isempty(override);
if using_override
    exe=string(override);hostdir='';
else
    hostdir=fullfile(paths.mil_root,'host','production_soh_runner');
    if ispc
        candidate=fullfile(hostdir,'build','production_soh_runner.exe');
    else
        candidate=fullfile(hostdir,'build','production_soh_runner');
    end
    exe=string(candidate);
end

if ispc && ~using_override && p.Results.BuildIfMissing
    build_script=fullfile(hostdir,'build_windows_msys2.cmd');
    deps={ ...
        fullfile(hostdir,'main.c'), ...
        fullfile(hostdir,'Makefile'), ...
        build_script, ...
        fullfile(paths.repo_root,'AMS','Core','Src','soh','ams_soh.c'), ...
        fullfile(paths.repo_root,'AMS','Core','Inc','soh','ams_soh.h')};
    mil.production.ensure_windows_local_runner(exe,build_script,deps,'production SoH');
    return;
end

if isfile(exe),return;end
if ~p.Results.BuildIfMissing
    error('mil:production:SohRunnerMissing','production SoH runner not found: %s',exe);
end
if ispc
    error('mil:production:SohRunnerMissing', ...
        ['production SoH runner is not built. Use the checked-in MSYS2 helper or set ', ...
         'DER26_MIL_SOH_RUNNER to an explicitly managed executable.']);
end
hostdir=fileparts(fileparts(char(exe)));
[status,text]=system(sprintf('make -C %s all',shell_quote(hostdir)));
if status~=0||~isfile(exe)
    error('mil:production:SohRunnerBuild','Failed to build production SoH runner:\n%s',text);
end
end

function q=shell_quote(path)
path=char(path);
if ispc
    q=['"' strrep(path,'"','""') '"'];
else
    q=['''' strrep(path,'''','''"''"''') ''''];
end
end
