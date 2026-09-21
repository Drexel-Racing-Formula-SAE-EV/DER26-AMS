function artifacts = export_result(result,output_directory)
%EXPORT_RESULT Save reproducible MiL evidence without huge hidden-truth CSV dumps.
out=char(output_directory);if ~isfolder(out),mkdir(out);end
artifacts=struct();
artifacts.mat=fullfile(out,'result.mat');save(artifacts.mat,'result','-v7.3');
artifacts.summary_json=fullfile(out,'summary.json');write_text(artifacts.summary_json,jsonencode(result.summary,'PrettyPrint',true));
artifacts.config_json=fullfile(out,'scenario_config.json');write_text(artifacts.config_json,jsonencode(result.scenario,'PrettyPrint',true));
artifacts.summary_csv=fullfile(out,'summary.csv');writetable(struct2table(result.summary,'AsArray',true),artifacts.summary_csv);

if isfield(result.reference,'ekf')
    S=size(result.reference.ekf.soc,2);T=table(result.reference.ekf.time_s,'VariableNames',{'time_s'});
    for s=1:S
        T.(sprintf('soc_est_s%d',s))=result.reference.ekf.soc(:,s);T.(sprintf('soc_truth_s%d',s))=result.truth.segment_soc(:,s);
        T.(sprintf('innovation_V_s%d',s))=result.reference.ekf.innovation_V(:,s);T.(sprintf('nis_s%d',s))=result.reference.ekf.NIS(:,s);
        T.(sprintf('nees_s%d',s))=result.metrics.ekf.NEES(:,s);
    end
    artifacts.reference_ekf_csv=fullfile(out,'reference_ekf_timeseries.csv');writetable(T,artifacts.reference_ekf_csv);
end
if isfield(result,'production')&&isfield(result.production,'ekf')
    P=result.production.ekf;S=size(P.soc,2);T=table(P.time_s,'VariableNames',{'time_s'});
    for s=1:S
        T.(sprintf('soc_prod_s%d',s))=P.soc(:,s);T.(sprintf('soc_truth_s%d',s))=result.truth.segment_soc(:,s);
        T.(sprintf('vp1_prod_s%d',s))=P.vp1_V(:,s);T.(sprintf('vp2_prod_s%d',s))=P.vp2_V(:,s);T.(sprintf('r0_prod_s%d',s))=P.r0_ohm(:,s);
        T.(sprintf('innovation_V_s%d',s))=P.innovation_V(:,s);T.(sprintf('R_V2_s%d',s))=P.r_meas_V2(:,s);
        T.(sprintf('accepted_s%d',s))=P.accepted(:,s);T.(sprintf('valid_s%d',s))=P.valid(:,s);T.(sprintf('fault_flags_s%d',s))=P.fault_flags(:,s);
    end
    artifacts.production_ekf_csv=fullfile(out,'production_ekf_timeseries.csv');writetable(T,artifacts.production_ekf_csv);
end
if isfield(result,'production')&&isfield(result.production,'soh')
    P=result.production.soh;
    T=table(double(P.time_s),double(P.sequence),double(P.update_ok),double(P.reason_flags), ...
        double(P.accepted_windows),double(P.rejected_windows),double(P.capacity_Ah), ...
        double(P.capacity_soh),double(P.capacity_soh_lower),double(P.capacity_valid), ...
        double(P.resistance_growth),double(P.resistance_growth_upper),double(P.resistance_valid), ...
        'VariableNames',{'time_s','sequence','update_ok','reason_flags','accepted_windows','rejected_windows', ...
        'capacity_Ah','capacity_soh','capacity_soh_lower','capacity_valid','resistance_growth','resistance_growth_upper','resistance_valid'});
    artifacts.production_soh_csv=fullfile(out,'production_soh_timeseries.csv');writetable(T,artifacts.production_soh_csv);
end
if isfield(result,'oracle')&&isfield(result.oracle,'sop')&&~isempty(result.oracle.sop)
    rows=[];
    for k=1:numel(result.oracle.sop)
        r=result.oracle.sop(k);
        for h=1:numel(r.horizons_s)
            rows=[rows;r.time_s,r.horizons_s(h),r.discharge_current_A(h), ...
                r.charge_current_A(h),r.fuse_discharge_current_cap_A(h), ...
                r.combined_discharge_current_A(h), ...
                r.fuse_charge_current_cap_A(h), ...
                r.combined_charge_current_A(h)]; %#ok<AGROW>
        end
    end
    artifacts.sop_oracle_csv=fullfile(out,'sop_truth_oracle.csv');
    writetable(array2table(rows,'VariableNames',{'time_s','horizon_s', ...
        'electrothermal_discharge_current_A','charge_current_A', ...
        'fuse_reference_cap_A','combined_discharge_current_A', ...
        'fuse_reference_charge_cap_A','combined_charge_current_A'}), ...
        artifacts.sop_oracle_csv);
end
if isfield(result,'fuse_replay')&&~isempty(result.fuse_replay)
    F=result.fuse_replay;
    T=table(F.time_s,F.production_utilization,F.reference_utilization, ...
        F.production_authority,F.reference_authority, ...
        'VariableNames',{'time_s','production_utilization','reference_utilization', ...
        'production_authority','reference_authority'});
    for h=1:4
        T.(sprintf('production_cap_h%d_A',h))=F.production_cap_A(:,h);
        T.(sprintf('reference_cap_h%d_A',h))=F.reference_cap_A(:,h);
        T.(sprintf('production_charge_cap_h%d_A',h))= ...
            F.production_charge_cap_A(:,h);
        T.(sprintf('reference_charge_cap_h%d_A',h))= ...
            F.reference_charge_cap_A(:,h);
    end
    artifacts.fuse_csv=fullfile(out,'fuse_model_comparison.csv');writetable(T,artifacts.fuse_csv);
end
if isfield(result,'production')&&isfield(result.production,'sop')&&~isempty(result.production.sop)
    P=result.production.sop;rows=[];
    for k=1:numel(P.time_s)
        for h=1:size(P.horizons_s,2)
            rows=[rows;P.time_s(k),P.horizons_s(k,h),P.model_discharge_current_A(k,h), ...
                P.model_charge_current_A(k,h),P.discharge_current_A(k,h),P.charge_current_A(k,h), ...
                double(P.reason_flags(k)),double(P.valid(k)),double(P.authority_valid(k))]; %#ok<AGROW>
        end
    end
    artifacts.production_sop_csv=fullfile(out,'production_sop.csv');
    writetable(array2table(rows,'VariableNames',{'time_s','horizon_s','model_discharge_A','model_charge_A', ...
        'published_discharge_A','published_charge_A','reason_flags','valid','authority_valid'}),artifacts.production_sop_csv);
end
end
function write_text(path,text)
fid=fopen(path,'w');if fid<0,error('mil:export:Open','Cannot write %s.',path);end
c=onCleanup(@()fclose(fid));fprintf(fid,'%s\n',text);
end
