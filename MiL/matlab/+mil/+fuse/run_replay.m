function result = run_replay(truth,cfg,varargin)
%RUN_REPLAY Compare production and independent fuse models on a MiL truth trace.
%
% The trace uses hidden plant current only because this result is an oracle/
% model-consistency evidence track. It does not feed any estimator.
p=inputParser();
p.addParameter('KeepFiles',false,@(x)islogical(x)&&isscalar(x));
p.addParameter('WorkingDirectory','',@(x)ischar(x)||isstring(x));
p.parse(varargin{:});opt=p.Results;
if strlength(string(opt.WorkingDirectory))==0
    work=tempname();mkdir(work);cleanup_dir=onCleanup(@()cleanup_work(work,opt.KeepFiles)); %#ok<NASGU>
else
    work=char(opt.WorkingDirectory);if ~isfolder(work),mkdir(work);end
end
trace_path=fullfile(work,'mil_fuse_trace.csv');
detail_path=fullfile(work,'mil_fuse_replay.csv');
summary_path=fullfile(work,'mil_fuse_summary.csv');
N=numel(truth.time_s);
temperature=max(double(truth.group_surface_C),[],2);
T=table(round(1000*double(truth.time_s(:))),double(truth.pack_current_A(:)), ...
    repmat(double(cfg.current_uncertainty_A),N,1),temperature,ones(N,1), ...
    ones(N,1),ones(N,1),repmat(double(cfg.temperature_measured_at_fuse),N,1), ...
    repmat(double(cfg.model_validated),N,1),strings(N,1), ...
    'VariableNames',{'timestamp_ms','current_a','current_uncertainty_a', ...
    'temperature_proxy_c','measurement_valid','current_calibrated', ...
    'current_polarity_validated','temperature_measured_at_fuse', ...
    'model_validated','event'});
writetable(T,trace_path);
exe=mil.fuse.runner_path();
cmd=sprintf('%s --trace %s --output %s --summary %s --startup %s --strict', ...
    shell_quote(exe),shell_quote(trace_path),shell_quote(detail_path), ...
    shell_quote(summary_path),shell_quote(cfg.startup_policy));
[status,text]=system(cmd);
if status~=0,error('mil:fuse:Replay','Fuse replay failed (%d):\n%s',status,text);end
D=readtable(detail_path,'VariableNamingRule','preserve');
S=readtable(summary_path,'VariableNamingRule','preserve');
result=struct();
result.description='production fuse observer vs independent long-double exact-ZOH oracle';
result.source='Tools/fuse_replay/fuse_reference_oracle.c';
result.time_s=double(D.timestamp_ms)/1000;
result.production_utilization=double(D.prod_utilization);
result.reference_utilization=double(D.ref_utilization);
result.production_authority=logical(D.prod_authority);
result.reference_authority=logical(D.ref_authority);
result.production_cap_A=[double(D.prod_cap_0p1s_a),double(D.prod_cap_1s_a), ...
    double(D.prod_cap_10s_a),double(D.prod_cap_30s_a)];
result.reference_cap_A=[double(D.ref_cap_0p1s_a),double(D.ref_cap_1s_a), ...
    double(D.ref_cap_10s_a),double(D.ref_cap_30s_a)];
result.strict_pass=logical(S.strict_pass(1));
result.summary_table=S;
result.host_output=text;
if opt.KeepFiles
    result.host_artifacts=struct('trace_csv',string(trace_path), ...
        'detail_csv',string(detail_path),'summary_csv',string(summary_path));
else
    result.host_artifacts=struct();
end
end
function cleanup_work(work,keep),if ~keep&&isfolder(work),rmdir(work,'s');end,end
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
