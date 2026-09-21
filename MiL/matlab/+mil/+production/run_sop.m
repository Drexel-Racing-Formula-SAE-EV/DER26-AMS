function result = run_sop(meas,prod_ekf,pack_cfg,cfg,varargin)
%RUN_SOP Execute production ams_sop.c at configured MiL checkpoints.
%
% The production solver consumes production EKF state plus the simulated AMS
% measurement image. Capacity/resistance SoH start from production conservative
% priors unless independently available; hidden plant truth is never fed here.
p=inputParser();p.addParameter('KeepFiles',false,@(x)islogical(x)&&isscalar(x));
p.addParameter('WorkingDirectory','',@(x)ischar(x)||isstring(x));p.parse(varargin{:});opt=p.Results;
S=size(prod_ekf.soc,2);if S~=5,error('mil:production:SopTopology','Expected five segment estimators.');end
checkpoint_times=double(cfg.checkpoint_times_s(:));
if isempty(checkpoint_times)
    checkpoint_times=meas.time_s(round(linspace(1,numel(meas.time_s),min(10,numel(meas.time_s)))));
end
indices=zeros(numel(checkpoint_times),1);keep=false(size(indices));
for k=1:numel(checkpoint_times)
    if checkpoint_times(k)>=meas.time_s(1)&&checkpoint_times(k)<=meas.time_s(end)
        [~,indices(k)]=min(abs(meas.time_s-checkpoint_times(k)));keep(k)=true;
    end
end
indices=indices(keep);checkpoint_times=checkpoint_times(keep);
if isempty(indices),result=struct([]);return;end

if strlength(string(opt.WorkingDirectory))==0
    work=tempname();mkdir(work);cleanup_dir=onCleanup(@()cleanup_work(work,opt.KeepFiles)); %#ok<NASGU>
else
    work=char(opt.WorkingDirectory);if ~exist(work,'dir'),mkdir(work);end
end
input_path=fullfile(work,'production_sop_input.csv');output_path=fullfile(work,'production_sop_output.csv');
K=numel(indices);
T=table();
T.time_s=double(meas.time_s(indices));T.sequence=double(meas.sequence(indices));
T.timestamp_ms=round(1000*double(meas.timestamp_s(indices)));T.now_ms=T.timestamp_ms;
T.current_A=finite_or(double(meas.pack_current_A(indices)),0);
T.current_uncertainty_A=0.5*ones(K,1); % production default floor; replace when HW calibration uncertainty is frozen
T.ambient_C=max(double(meas.segment_surface_max_C(indices,:)),[],2,'omitnan');
T.ambient_C=finite_or(T.ambient_C,25);
T.measurement_valid=double(meas.measurement_valid(indices));
T.estimator_valid=double(all(prod_ekf.valid(indices,:),2));
if isfield(prod_ekf,'acquisition_state') && isfield(prod_ekf,'acquisition_anchor_count')
    acquired=all(prod_ekf.acquisition_state(indices,:)==uint8(2) & ...
        prod_ekf.acquisition_anchor_count(indices,:)>0,2);
else
    acquired=false(K,1);
end
T.estimator_acquired=double(acquired);
T.segment_topology=ones(K,1);
T.current_calibrated=double(meas.current_calibrated(indices));
T.polarity_validated=double(meas.current_polarity_validated(indices));
T.ambient_measured=zeros(K,1); % mirrors current production hottest-surface proxy
T.balance_recovered=ones(K,1);
for s=1:S
    pfx=sprintf('s%d_',s-1);
    T.([pfx 'soc'])=finite_or(prod_ekf.soc(indices,s),0.5);
    T.([pfx 'vp1'])=finite_or(prod_ekf.vp1_V(indices,s),0);
    T.([pfx 'vp2'])=finite_or(prod_ekf.vp2_V(indices,s),0);
    T.([pfx 'r0'])=finite_or(prod_ekf.r0_ohm(indices,s),0.0147);
    T.([pfx 'core'])=finite_or(prod_ekf.core_temp_C(indices,s),25);
    T.([pfx 'surface'])=finite_or(meas.segment_surface_max_C(indices,s),25);
    T.([pfx 'p_soc'])=finite_or(prod_ekf.P_diag(indices,s,1),1e-2);
    T.([pfx 'p_vp1'])=finite_or(prod_ekf.P_diag(indices,s,2),1e-3);
    T.([pfx 'p_vp2'])=finite_or(prod_ekf.P_diag(indices,s,3),1e-3);
    T.([pfx 'p_r0'])=finite_or(prod_ekf.P_diag(indices,s,4),1e-4);
    T.([pfx 'innovation'])=finite_or(prod_ekf.innovation_V(indices,s),0);
    T.([pfx 'cap_lower'])=0.80*ones(K,1);T.([pfx 'res_upper'])=1.25*ones(K,1);
    T.([pfx 'max_age'])=double(meas.segment_max_cell_age_ms(indices,s));
    T.([pfx 'mask'])=double(meas.segment_cell_usable_mask(indices,s));
    T.([pfx 'est_valid'])=double(prod_ekf.valid(indices,s));
    T.([pfx 'model_flags'])=double(prod_ekf.model_domain_flags(indices,s));
    T.([pfx 'cap_valid'])=zeros(K,1);T.([pfx 'res_valid'])=zeros(K,1);
    group_indices=find(double(pack_cfg.group_to_segment(:))==s);
    if numel(group_indices)~=15,error('mil:production:SopCells','Segment %d does not contain 15 groups.',s);end
    for c=1:15
        T.(sprintf('%scell%d',pfx,c-1))=finite_or(meas.cell_voltage_V(indices,group_indices(c)),0);
    end
end
writetable(T,input_path);
exe=mil.production.sop_runner_path();cmd=sprintf('%s %s %s',shell_quote(exe),shell_quote(input_path),shell_quote(output_path));
[status,text]=system(cmd);if status~=0,error('mil:production:SopRun','Production SoP runner failed (%d):\n%s',status,text);end
O=readtable(output_path,'VariableNamingRule','preserve');if height(O)~=K,error('mil:production:SopRows','Expected %d SoP rows, got %d.',K,height(O));end
H=4;result=struct();result.description='checked-in production ams_sop.c host execution';result.source='AMS/Core/Src/sop/ams_sop.c';
result.time_s=double(O.time_s);result.index=indices;result.sequence=uint32(O.sequence);result.status=uint8(O.status);
result.valid=logical(O.valid);result.authority_valid=logical(O.authority_valid);result.fallback_active=logical(O.fallback_active);result.reason_flags=uint32(O.reason_flags);
result.horizons_s=nan(K,H);result.model_discharge_current_A=nan(K,H);result.model_charge_current_A=nan(K,H);result.discharge_current_A=nan(K,H);result.charge_current_A=nan(K,H);
result.discharge_binding=zeros(K,H,'uint8');result.charge_binding=zeros(K,H,'uint8');
for h=1:H
    pfx=sprintf('h%d_',h-1);result.horizons_s(:,h)=double(O.([pfx 's']));
    result.model_discharge_current_A(:,h)=double(O.([pfx 'model_discharge_A']));result.model_charge_current_A(:,h)=double(O.([pfx 'model_charge_A']));
    result.discharge_current_A(:,h)=double(O.([pfx 'discharge_A']));result.charge_current_A(:,h)=double(O.([pfx 'charge_A']));
    result.discharge_binding(:,h)=uint8(O.([pfx 'd_bind']));result.charge_binding(:,h)=uint8(O.([pfx 'c_bind']));
end
result.host_artifacts=struct('input_csv',string(input_path),'output_csv',string(output_path));if ~opt.KeepFiles,result.host_artifacts=struct();end
end
function y=finite_or(x,fallback),y=double(x);y(~isfinite(y))=fallback;end
function cleanup_work(work,keep),if ~keep&&exist(work,'dir'),rmdir(work,'s');end,end
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
