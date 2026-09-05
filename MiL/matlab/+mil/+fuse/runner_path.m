function exe = runner_path()
%RUNNER_PATH Build/locate the checked-in production/reference fuse replay tool.
paths=mil.project_paths();
exe=fullfile(paths.fuse_replay,'fuse_replay');
if ispc,exe=[exe '.exe'];end
if ~isfile(exe)
    [status,text]=system(sprintf('make -C %s all',shell_quote(paths.fuse_replay)));
    if status~=0,error('mil:fuse:Build','Fuse replay build failed:\n%s',text);end
end
if ~isfile(exe),error('mil:fuse:Missing','Fuse replay executable missing: %s.',exe);end
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
