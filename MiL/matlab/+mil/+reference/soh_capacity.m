function result = soh_capacity(meas, ekf, pack_cfg, nominal_capacity_Ah)
%SOH_CAPACITY Independent rest-anchor capacity benchmark.
%
% Unlike production ams_soh.c, this benchmark finds all qualified rest
% anchors first and then fits charge-throughput versus SOC across every
% sufficiently separated anchor pair using robust median statistics.
N = numel(meas.time_s);
dt = [0; diff(meas.time_s)];
current = meas.pack_current_A;
current(~isfinite(current)) = 0;
charge_As = cumsum(current .* dt);
pack_soc = mean(ekf.soc,2,'omitnan');
max_sigma = zeros(N,1);
max_innov_per_cell = zeros(N,1);
max_polarization = zeros(N,1);
for n=1:N
    sigma = zeros(size(ekf.soc,2),1);
    for s=1:size(ekf.soc,2)
        P = squeeze(ekf.P(n,s,:,:));
        sigma(s)=sqrt(max(P(1,1),0));
    end
    max_sigma(n)=max(sigma);
    max_innov_per_cell(n)=max(abs(ekf.innovation_V(n,:)),[],'omitnan')/15;
    max_polarization(n)=max(abs(ekf.vp1_V(n,:))+abs(ekf.vp2_V(n,:)),[],'omitnan');
end

cell_spread = max(meas.cell_voltage_V,[],2,'omitnan') - ...
    min(meas.cell_voltage_V,[],2,'omitnan');
avg_temp = mean(meas.temperature_C,2,'omitnan');
rest_ok = meas.measurement_valid & abs(current) <= 0.5 & ...
    avg_temp >= 10 & avg_temp <= 40 & cell_spread <= 0.050 & ...
    max_sigma <= 0.015 & max_innov_per_cell <= 0.015 & ...
    max_polarization <= 0.020;

anchors = [];
rest_elapsed = 0;
latched = false;
for n=2:N
    if rest_ok(n)
        rest_elapsed = rest_elapsed + dt(n);
        if rest_elapsed >= 60 && ~latched
            anchors(end+1,:) = [n,meas.time_s(n),pack_soc(n),charge_As(n),avg_temp(n)]; %#ok<AGROW>
            latched = true;
        end
    else
        rest_elapsed=0;
        latched=false;
    end
end

candidates=[];
pairs=[];
for a=1:size(anchors,1)-1
    for b=a+1:size(anchors,1)
        dsoc=anchors(b,3)-anchors(a,3);
        dq=anchors(b,4)-anchors(a,4);
        throughput_Ah=abs(dq)/3600;
        if abs(dsoc) >= 0.15 && throughput_Ah >= 3.0 && dsoc*dq < 0
            cap=throughput_Ah/abs(dsoc);
            if isfinite(cap) && cap >= 0.5*nominal_capacity_Ah && cap <= 1.1*nominal_capacity_Ah
                candidates(end+1,1)=cap; %#ok<AGROW>
                pairs(end+1,:)=[a b]; %#ok<AGROW>
            end
        end
    end
end

result=struct();
result.anchors=anchors;
result.candidate_capacity_Ah=candidates;
result.anchor_pairs=pairs;
result.valid=false;
result.capacity_Ah=NaN;
result.capacity_soh=NaN;
result.capacity_sigma_Ah=NaN;
result.capacity_soh_lower=0.80;
result.observation_count=numel(candidates);
if ~isempty(candidates)
    med=median(candidates);
    madv=median(abs(candidates-med));
    if madv > 0
        keep=abs(candidates-med) <= 3*1.4826*madv;
    else
        keep=true(size(candidates));
    end
    used=candidates(keep);
    result.capacity_Ah=mean(used);
    result.capacity_soh=result.capacity_Ah/nominal_capacity_Ah;
    if numel(used)>1
        result.capacity_sigma_Ah=std(used,0);
    else
        result.capacity_sigma_Ah=0.5;
    end
    result.capacity_soh_lower=max(0.5, ...
        (result.capacity_Ah-3*max(result.capacity_sigma_Ah,0.5))/nominal_capacity_Ah);
    result.valid=numel(used)>=2;
    result.used_candidates=used;
else
    result.used_candidates=[];
end
result.note='Independent all-anchor-pairs benchmark; not production algorithm parity.';
end
