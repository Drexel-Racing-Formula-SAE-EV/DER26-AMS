function result = run_estimator(meas, initial_soc, varargin)
%RUN_ESTIMATOR Run the checked-in production AMS five-segment EKF C source.
%
% This is parity execution, not an independent oracle. The host executable
% links ams_soc_ekf.c and ams_estimator_lut.c directly from AMS/Core/Src.
p=inputParser();
p.addParameter('KeepFiles',false,@(x)islogical(x)&&isscalar(x));
p.addParameter('WorkingDirectory','',@(x)ischar(x)||isstring(x));
p.addParameter('PreconditionRestS',0,@(x)isnumeric(x)&&isscalar(x)&&isfinite(x)&&x>=0);
p.parse(varargin{:});opt=p.Results;

S=size(meas.segment_voltage_V,2);
if S~=5,error('mil:production:Topology','Production runner expects five DER26 segments.');end
soc0=double(initial_soc(:));
if isscalar(soc0),soc0=repmat(soc0,S,1);end
if numel(soc0)~=S || any(~isfinite(soc0)) || any(soc0<0|soc0>1)
    error('mil:production:InitialSoc','Initial SoC must be scalar or five finite values in [0,1].');
end

if strlength(string(opt.WorkingDirectory))==0
    work=tempname();mkdir(work);cleanup_dir=onCleanup(@()cleanup_work(work,opt.KeepFiles));
else
    work=char(opt.WorkingDirectory);if ~exist(work,'dir'),mkdir(work);end
    cleanup_dir=[]; %#ok<NASGU>
end
input_path=fullfile(work,'production_estimator_input.csv');
output_path=fullfile(work,'production_estimator_output.csv');

N=numel(meas.time_s);
% The production task gates voltage/temperature per segment. Do not use the
% aggregate measurement_valid bit here because one PEC-invalid segment would
% incorrectly suppress all five host EKFs. This common field represents the
% shared current/epoch prerequisite; segment validity is written below.
common_valid = logical(meas.current_valid(:)) & isfinite(meas.pack_current_A(:));
T=table(double(meas.timestamp_s(:)),double(meas.sequence(:)),double(meas.pack_current_A(:)), ...
    double(common_valid),double(meas.current_calibrated(:)), ...
    double(meas.current_polarity_validated(:)),ones(N,1), ...
    'VariableNames',{'time_s','sequence','current_A','measurement_valid', ...
    'current_calibrated','current_polarity_validated','balance_recovered'});
for s=1:S
    T.(sprintf('s%d_v',s-1))=double(meas.segment_voltage_V(:,s));
    T.(sprintf('s%d_t',s-1))=double(meas.segment_surface_max_C(:,s));
    T.(sprintf('s%d_valid',s-1))=double(isfinite(meas.segment_voltage_V(:,s)) & ...
        isfinite(meas.segment_surface_max_C(:,s)));
end

% Dynamic qualification scenarios can start from an already-qualified BMS
% without changing the scored drive profile. Precondition only the production
% estimator with a stationary copy of the first measured electrical/thermal
% state, then discard those host rows. This is deliberately distinct from C0/
% C1 startup cases, which exercise acquisition inside scored scenario time.
pre_count=0;pre_duration_s=0.0;
if opt.PreconditionRestS>0
    dt=median_positive(diff(double(meas.timestamp_s(:))));
    if ~isfinite(dt)||dt<=0,dt=0.1;end
    pre_count=max(1,ceil(double(opt.PreconditionRestS)/dt));
    pre_duration_s=pre_count*dt;
    first_segment_valid=true;
    for s=1:S
        valid_column=T.(sprintf('s%d_valid',s-1));
        first_segment_valid=first_segment_valid && logical(valid_column(1));
    end
    if ~logical(T.measurement_valid(1)) || ...
            ~logical(T.current_calibrated(1)) || ...
            ~logical(T.current_polarity_validated(1)) || ~first_segment_valid
        error('mil:production:EstimatorPreconditionInput', ...
            ['Production estimator preconditioning requires a valid calibrated ', ...
             'first measurement on all five segments.']);
    end
    preT=T(ones(pre_count,1),:);
    preT.time_s=(0:pre_count-1).'*dt;
    preT.sequence=(1:pre_count).';
    preT.current_A=zeros(pre_count,1);
    preT.balance_recovered=ones(pre_count,1);

    T.time_s=T.time_s+pre_duration_s;
    T.sequence=T.sequence+pre_count;
    T=[preT;T]; %#ok<AGROW>
end
writetable(T,input_path);

exe=mil.production.estimator_runner_path();
args=strjoin(arrayfun(@(x)sprintf('%.9g',x),soc0(:).','UniformOutput',false),' ');
cmd=sprintf('%s %s %s %s',shell_quote(exe),shell_quote(input_path), ...
    shell_quote(output_path),args);
[status,text]=system(cmd);
if status~=0
    error('mil:production:EstimatorRun','Production estimator runner failed (%d):\n%s',status,text);
end
O=readtable(output_path,'VariableNamingRule','preserve');
expected_rows=N+pre_count;
if height(O)~=expected_rows
    error('mil:production:EstimatorRows', ...
        'Production runner returned %d rows for %d inputs.',height(O),expected_rows);
end
if pre_count>0
    O=O(pre_count+1:end,:);
    % Restore scored scenario coordinates. Internal estimator state retains the
    % completed precondition, but all MiL truth/metric alignment remains t=0..
    O.time_s=double(meas.timestamp_s(:));
    O.sequence=double(meas.sequence(:));
end
required_schema={'s0_acq_state','s0_acq_reason','s0_acq_candidate_soc', ...
    's0_acq_dynamic_step_count','s0_acq_dynamic_update_count', ...
    's0_p_soc_vp1','s0_p_soc_vp2','s0_p_vp1_vp2', ...
    's0_covariance_repair_count','s0_innovation_variance_V2', ...
    's0_pre_soc','s0_pre_p_soc_vp1'};
missing_schema=required_schema(~ismember(required_schema,O.Properties.VariableNames));
if ~isempty(missing_schema)
    error('mil:production:EstimatorSchema', ...
        ['Production estimator host runner CSV schema is stale. Missing: %s. ', ...
         'Rebuild MiL/host/production_estimator_runner from the current tree ', ...
         '(Windows: run build_windows_msys2.cmd) and rerun MATLAB.'], ...
        strjoin(missing_schema,', '));
end

result=struct();
result.description='checked-in production AMS five-segment EKF host execution';
result.source='AMS/Core/Src/estimator/ams_soc_ekf.c';
result.precondition=struct('applied',pre_count>0, ...
    'requested_rest_s',double(opt.PreconditionRestS), ...
    'executed_rest_s',pre_duration_s,'samples',pre_count);
result.time_s=double(O.time_s);
result.sequence=uint32(O.sequence);
result.soc=nan(N,S);result.vp1_V=nan(N,S);result.vp2_V=nan(N,S);
result.r0_ohm=nan(N,S);result.core_temp_C=nan(N,S);
result.P_diag=nan(N,S,4);
result.P_full=nan(N,S,3,3);
result.covariance_repair_count=zeros(N,S,'uint32');
result.r_meas_V2=nan(N,S);
result.pre_update=struct();
result.pre_update.soc=nan(N,S);
result.pre_update.vp1_V=nan(N,S);
result.pre_update.vp2_V=nan(N,S);
result.pre_update.r0_ohm=nan(N,S);
result.pre_update.P_full=nan(N,S,3,3);
result.predicted_voltage_V=nan(N,S);result.innovation_V=nan(N,S);
result.innovation_variance_V2=nan(N,S);
result.accepted=false(N,S);result.measurement_used=false(N,S);
result.step_ok=false(N,S);result.valid=false(N,S);
result.fault_flags=zeros(N,S,'uint32');result.model_domain_flags=zeros(N,S,'uint8');
result.r0_update_result=zeros(N,S,'uint8');result.soh_reject_flags=zeros(N,S,'uint32');
result.soh_accepted_count=zeros(N,S,'uint32');result.soh_rejected_count=zeros(N,S,'uint32');
result.resistance_growth_ratio=nan(N,S);result.resistance_confidence_pct=zeros(N,S,'uint8');result.resistance_status_flags=zeros(N,S,'uint8');
result.acquisition_state=zeros(N,S,'uint8');
result.acquisition_reason=zeros(N,S,'uint8');
result.acquisition_sample_count=zeros(N,S,'uint8');
result.acquisition_reject_count=zeros(N,S,'uint8');
result.acquisition_candidate_ready=false(N,S);
result.acquisition_anchor_count=zeros(N,S,'uint32');
result.acquisition_candidate_soc=nan(N,S);
result.acquisition_ocv_cell_V=nan(N,S);
result.acquisition_vp1_finish_V=nan(N,S);
result.acquisition_vp2_finish_V=nan(N,S);
result.acquisition_fit_rmse_mV_cell=nan(N,S);
result.acquisition_fit_rcond=nan(N,S);
result.acquisition_consensus_soc=nan(N,S);
result.acquisition_dynamic_step_count=zeros(N,S,'uint32');
result.acquisition_dynamic_update_count=zeros(N,S,'uint32');
for s=1:S
    p=sprintf('s%d_',s-1);
    result.step_ok(:,s)=logical(O.([p 'step_ok']));
    result.measurement_used(:,s)=logical(O.([p 'measurement_used']));
    result.accepted(:,s)=logical(O.([p 'measurement_accepted']));
    result.soc(:,s)=double(O.([p 'soc']));
    result.vp1_V(:,s)=double(O.([p 'vp1_V']));
    result.vp2_V(:,s)=double(O.([p 'vp2_V']));
    result.r0_ohm(:,s)=double(O.([p 'r0_ohm']));
    result.core_temp_C(:,s)=double(O.([p 'tcore_C']));
    result.P_diag(:,s,1)=double(O.([p 'p_soc']));
    result.P_diag(:,s,2)=double(O.([p 'p_vp1']));
    result.P_diag(:,s,3)=double(O.([p 'p_vp2']));
    result.P_diag(:,s,4)=double(O.([p 'p_r0']));
    result.P_full(:,s,1,1)=result.P_diag(:,s,1);
    result.P_full(:,s,1,2)=double(O.([p 'p_soc_vp1']));
    result.P_full(:,s,2,1)=result.P_full(:,s,1,2);
    result.P_full(:,s,1,3)=double(O.([p 'p_soc_vp2']));
    result.P_full(:,s,3,1)=result.P_full(:,s,1,3);
    result.P_full(:,s,2,2)=result.P_diag(:,s,2);
    result.P_full(:,s,2,3)=double(O.([p 'p_vp1_vp2']));
    result.P_full(:,s,3,2)=result.P_full(:,s,2,3);
    result.P_full(:,s,3,3)=result.P_diag(:,s,3);
    result.covariance_repair_count(:,s)=uint32(O.([p 'covariance_repair_count']));
    result.pre_update.soc(:,s)=double(O.([p 'pre_soc']));
    result.pre_update.vp1_V(:,s)=double(O.([p 'pre_vp1_V']));
    result.pre_update.vp2_V(:,s)=double(O.([p 'pre_vp2_V']));
    result.pre_update.r0_ohm(:,s)=double(O.([p 'pre_r0_ohm']));
    result.pre_update.P_full(:,s,1,1)=double(O.([p 'pre_p_soc']));
    result.pre_update.P_full(:,s,1,2)=double(O.([p 'pre_p_soc_vp1']));
    result.pre_update.P_full(:,s,2,1)=result.pre_update.P_full(:,s,1,2);
    result.pre_update.P_full(:,s,1,3)=double(O.([p 'pre_p_soc_vp2']));
    result.pre_update.P_full(:,s,3,1)=result.pre_update.P_full(:,s,1,3);
    result.pre_update.P_full(:,s,2,2)=double(O.([p 'pre_p_vp1']));
    result.pre_update.P_full(:,s,2,3)=double(O.([p 'pre_p_vp1_vp2']));
    result.pre_update.P_full(:,s,3,2)=result.pre_update.P_full(:,s,2,3);
    result.pre_update.P_full(:,s,3,3)=double(O.([p 'pre_p_vp2']));
    result.r_meas_V2(:,s)=double(O.([p 'R_V2']));
    result.predicted_voltage_V(:,s)=double(O.([p 'vpred_V']));
    result.innovation_V(:,s)=double(O.([p 'innovation_V']));
    result.innovation_variance_V2(:,s)=double(O.([p 'innovation_variance_V2']));
    result.valid(:,s)=logical(O.([p 'valid']));
    result.fault_flags(:,s)=uint32(O.([p 'fault_flags']));
    result.model_domain_flags(:,s)=uint8(O.([p 'model_domain_flags']));
    result.r0_update_result(:,s)=uint8(O.([p 'r0_update_result']));
    result.soh_reject_flags(:,s)=uint32(O.([p 'soh_reject_flags']));
    result.soh_accepted_count(:,s)=uint32(O.([p 'soh_accepted_count']));
    result.soh_rejected_count(:,s)=uint32(O.([p 'soh_rejected_count']));
    result.resistance_growth_ratio(:,s)=double(O.([p 'resistance_growth_ratio']));
    result.resistance_confidence_pct(:,s)=uint8(O.([p 'resistance_confidence_pct']));
    result.resistance_status_flags(:,s)=uint8(O.([p 'resistance_status_flags']));
    result.acquisition_state(:,s)=uint8(O.([p 'acq_state']));
    result.acquisition_reason(:,s)=uint8(O.([p 'acq_reason']));
    result.acquisition_sample_count(:,s)=uint8(O.([p 'acq_sample_count']));
    result.acquisition_reject_count(:,s)=uint8(O.([p 'acq_reject_count']));
    result.acquisition_candidate_ready(:,s)=logical(O.([p 'acq_candidate_ready']));
    result.acquisition_anchor_count(:,s)=uint32(O.([p 'acq_anchor_count']));
    result.acquisition_candidate_soc(:,s)=double(O.([p 'acq_candidate_soc']));
    result.acquisition_ocv_cell_V(:,s)=double(O.([p 'acq_ocv_cell_V']));
    result.acquisition_vp1_finish_V(:,s)=double(O.([p 'acq_vp1_finish_V']));
    result.acquisition_vp2_finish_V(:,s)=double(O.([p 'acq_vp2_finish_V']));
    result.acquisition_fit_rmse_mV_cell(:,s)=double(O.([p 'acq_fit_rmse_mV_cell']));
    result.acquisition_fit_rcond(:,s)=double(O.([p 'acq_fit_rcond']));
    result.acquisition_consensus_soc(:,s)=double(O.([p 'acq_consensus_soc']));
    result.acquisition_dynamic_step_count(:,s)=uint32(O.([p 'acq_dynamic_step_count']));
    result.acquisition_dynamic_update_count(:,s)=uint32(O.([p 'acq_dynamic_update_count']));
end
result.initial_state=struct( ...
    'soc',soc0(:).', ...
    'vp1_V',zeros(1,S), ...
    'vp2_V',zeros(1,S), ...
    'r0_ohm',repmat(0.0147,1,S), ...
    'p_soc',repmat(1.0e-2,1,S), ...
    'p_soc_vp1',zeros(1,S), ...
    'p_soc_vp2',zeros(1,S), ...
    'p_vp1',repmat(1.0e-3,1,S), ...
    'p_vp1_vp2',zeros(1,S), ...
    'p_vp2',repmat(1.0e-3,1,S), ...
    'p_r0',repmat(1.0e-4,1,S));
result.host_artifacts=struct('input_csv',string(input_path),'output_csv',string(output_path));
if ~opt.KeepFiles
    result.host_artifacts=struct();
end
end

function m=median_positive(x)
x=double(x(:));x=x(isfinite(x)&x>0);
if isempty(x),m=NaN;else,m=median(x);end
end

function cleanup_work(work,keep)
if ~keep && exist(work,'dir'),rmdir(work,'s');end
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
