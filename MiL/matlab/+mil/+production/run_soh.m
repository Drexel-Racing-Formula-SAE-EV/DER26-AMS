function result = run_soh(meas,prod_ekf,varargin)
%RUN_SOH Execute production ams_soh.c against the MiL measurement/estimator stream.
% Hidden plant truth is not consumed by this function.
p=inputParser();p.addParameter('KeepFiles',false,@(x)islogical(x)&&isscalar(x));p.addParameter('WorkingDirectory','',@(x)ischar(x)||isstring(x));p.parse(varargin{:});opt=p.Results;
N=numel(meas.time_s);S=size(prod_ekf.soc,2);if S~=5,error('mil:production:SohTopology','Expected five segments.');end
if strlength(string(opt.WorkingDirectory))==0,work=tempname();mkdir(work);cleanup_dir=onCleanup(@()cleanup_work(work,opt.KeepFiles)); %#ok<NASGU>
else,work=char(opt.WorkingDirectory);if ~exist(work,'dir'),mkdir(work);end,end
input_path=fullfile(work,'production_soh_input.csv');output_path=fullfile(work,'production_soh_output.csv');
t=double(meas.timestamp_s(:));dt=[median_positive(diff(t));diff(t)];dt(~isfinite(dt)|dt<=0)=0.1;dt=min(max(dt,0.001),1.0);
current=finite_or(meas.pack_current_A(:),0);total_charge_as=cumsum(current.*dt);
cell=double(meas.cell_voltage_V);cell(~meas.cell_fresh)=NaN;
spread=row_max(cell)-row_min(cell);spread(~isfinite(spread))=0;
temp=double(meas.temperature_C);temp(~meas.temperature_fresh)=NaN;avgtemp=row_mean(temp);avgtemp(~isfinite(avgtemp))=25;
pack_soc=row_mean(prod_ekf.soc);max_sigma=sqrt(max(double(prod_ekf.P_diag(:,:,1)),[],2));
max_innov=max(abs(double(prod_ekf.innovation_V))/15,[],2);max_pol=max(abs(double(prod_ekf.vp1_V))+abs(double(prod_ekf.vp2_V)),[],2);
T=table(double(meas.time_s(:)),double(meas.sequence(:)),round(t*1000),round(t*1000),dt,current,0.5*ones(N,1),total_charge_as, ...
    finite_or(pack_soc,0.5),avgtemp,spread,finite_or(max_sigma,1),finite_or(max_innov,1),finite_or(max_pol,1), ...
    double(meas.measurement_valid(:)),double(all(prod_ekf.valid,2)),double(meas.current_calibrated(:)), ...
    double(meas.current_polarity_validated(:)),ones(N,1), ...
    'VariableNames',{'time_s','sequence','timestamp_ms','now_ms','elapsed_s','current_A','current_uncertainty_A','total_charge_As','pack_soc','avg_cell_temp_C','cell_spread_V','max_soc_sigma','max_innovation_per_cell_V','max_polarization_V','measurement_valid','estimator_valid','current_calibrated','polarity_validated','balance_recovered'});
for s=1:S
    T.(sprintf('s%d_res_growth',s-1))=finite_or(prod_ekf.resistance_growth_ratio(:,s),1);
    T.(sprintf('s%d_res_conf',s-1))=double(prod_ekf.resistance_confidence_pct(:,s));
    flags=prod_ekf.resistance_status_flags(:,s);
    T.(sprintf('s%d_res_valid',s-1))=double( ...
        bitand(flags,uint8(8))~=0 & bitand(flags,uint8(2))~=0);
end
writetable(T,input_path);exe=mil.production.soh_runner_path();[status,text]=system(sprintf('%s %s %s',shell_quote(exe),shell_quote(input_path),shell_quote(output_path)));
if status~=0,error('mil:production:SohRun','Production SoH runner failed (%d):\n%s',status,text);end
O=readtable(output_path,'VariableNamingRule','preserve');if height(O)~=N,error('mil:production:SohRows','Expected %d rows, got %d.',N,height(O));end
result=struct();result.description='checked-in production ams_soh.c host execution';result.source='AMS/Core/Src/soh/ams_soh.c';
fields={'time_s','sequence','update_ok','reason_flags','rest_elapsed_s','anchor_valid','accepted_windows','rejected_windows','capacity_Ah','capacity_sigma_Ah','capacity_soh','capacity_soh_lower','capacity_confidence_pct','capacity_valid','resistance_growth','resistance_growth_upper','resistance_confidence_pct','resistance_valid','combined_soh'};
for k=1:numel(fields),result.(fields{k})=O.(fields{k});end
result.sequence=uint32(result.sequence);result.reason_flags=uint32(result.reason_flags);result.update_ok=logical(result.update_ok);result.anchor_valid=logical(result.anchor_valid);result.capacity_valid=logical(result.capacity_valid);result.resistance_valid=logical(result.resistance_valid);
result.host_artifacts=struct('input_csv',string(input_path),'output_csv',string(output_path));if ~opt.KeepFiles,result.host_artifacts=struct();end
end
function m=median_positive(x),x=x(isfinite(x)&x>0);if isempty(x),m=0.1;else,m=median(x);end,end
function y=finite_or(x,f),y=double(x);y(~isfinite(y))=f;end
function y=row_max(x),y=max(x,[],2,'omitnan');end
function y=row_min(x),y=min(x,[],2,'omitnan');end
function y=row_mean(x),y=mean(x,2,'omitnan');end
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
