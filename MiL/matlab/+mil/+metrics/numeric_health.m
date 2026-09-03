function report = numeric_health(result)
%NUMERIC_HEALTH Detect NaN/Inf/divergence and invalid estimator covariance.
%
% Production-C now exports the full symmetric [SoC,Vp1,Vp2] covariance.
% Numeric qualification therefore checks the actual matrix, not only its
% diagonal compatibility view. Covariance repair counters are reported as a
% diagnostic; a bounded conservative repair is not itself a numeric failure.
report=struct();
report.failures={};
report.production_covariance_repair_count_max=0;
report.production_covariance_psd_checked=0;
report.production_innovation_variance_checked=0;

fields={ ...
    'truth.group_voltage_V',result.truth.group_voltage_V; ...
    'truth.group_soc',result.truth.group_soc; ...
    'truth.group_core_C',result.truth.group_core_C};
if isfield(result.reference,'ekf')
    fields(end+1,:)={'reference.ekf.state',result.reference.ekf.state}; %#ok<AGROW>
    fields(end+1,:)={'reference.ekf.P',result.reference.ekf.P}; %#ok<AGROW>
end

if isfield(result,'production') && isfield(result.production,'ekf')
    prod=result.production.ekf;
    valid=logical(prod.valid);
    fields(end+1,:)={'production.ekf.soc(valid)',prod.soc(valid)}; %#ok<AGROW>
    fields(end+1,:)={'production.ekf.vp1(valid)',prod.vp1_V(valid)}; %#ok<AGROW>
    fields(end+1,:)={'production.ekf.vp2(valid)',prod.vp2_V(valid)}; %#ok<AGROW>
    fields(end+1,:)={'production.ekf.r0(valid)',prod.r0_ohm(valid)}; %#ok<AGROW>

    pdiag=double(prod.P_diag);
    pflat=reshape(pdiag,[],size(pdiag,3));
    if any(pflat(valid(:),:)<0,'all')
        report.failures{end+1}='production EKF contains negative variance'; %#ok<AGROW>
    end

    if ~isfield(prod,'P_full')
        report.failures{end+1}= ...
            'production EKF full [SoC,Vp1,Vp2] covariance is missing'; %#ok<AGROW>
    else
        P=double(prod.P_full);
        for n=1:size(valid,1)
            for s=1:size(valid,2)
                if ~valid(n,s),continue;end
                p=squeeze(P(n,s,:,:));
                report.production_covariance_psd_checked= ...
                    report.production_covariance_psd_checked+1;
                if ~isequal(size(p),[3 3]) || any(~isfinite(p),'all')
                    report.failures{end+1}=sprintf( ...
                        'production EKF invalid covariance at sample %d segment %d',n,s); %#ok<AGROW>
                    break;
                end
                symmetry=max(abs(p-p.'),[],'all');
                scale=max(1.0e-12,max(abs(p),[],'all'));
                if symmetry>(1.0e-5*scale) || ~covariance_psd_3x3(0.5*(p+p.'))
                    report.failures{end+1}=sprintf( ...
                        'production EKF non-PSD covariance at sample %d segment %d',n,s); %#ok<AGROW>
                    break;
                end
            end
            if ~isempty(report.failures),break;end
        end
    end

    if isfield(prod,'innovation_variance_V2') && isfield(prod,'measurement_used')
        mask=logical(prod.measurement_used) & valid;
        sv=double(prod.innovation_variance_V2(mask));
        report.production_innovation_variance_checked=numel(sv);
        if any(~isfinite(sv)) || any(sv<=0)
            report.failures{end+1}= ...
                'production EKF contains invalid prior innovation variance'; %#ok<AGROW>
        end
    end

    if isfield(prod,'covariance_repair_count')
        repairs=double(prod.covariance_repair_count(:));
        repairs=repairs(isfinite(repairs));
        if ~isempty(repairs)
            report.production_covariance_repair_count_max=max(repairs);
        end
    end
end

if isfield(result,'fuse_replay') && ~isempty(result.fuse_replay)
    fields(end+1,:)={'fuse.reference_utilization', ...
        result.fuse_replay.reference_utilization}; %#ok<AGROW>
    fields(end+1,:)={'fuse.reference_cap_A', ...
        result.fuse_replay.reference_cap_A}; %#ok<AGROW>
end
for k=1:size(fields,1)
    x=double(fields{k,2});
    if any(isinf(x),'all')
        report.failures{end+1}=sprintf('%s contains Inf',fields{k,1}); %#ok<AGROW>
    end
    if any(isnan(x),'all')
        report.failures{end+1}=sprintf('%s contains NaN',fields{k,1}); %#ok<AGROW>
    end
end

if isfield(result.reference,'ekf')
    P=double(result.reference.ekf.P);
    bad_cov=false;
    for n=1:size(P,1)
        for s=1:size(P,2)
            p=squeeze(P(n,s,:,:));
            if any(~isfinite(p),'all') || ~covariance_psd_3x3(0.5*(p+p.'))
                report.failures{end+1}=sprintf( ...
                    'reference EKF invalid covariance at sample %d segment %d',n,s); %#ok<AGROW>
                bad_cov=true;break;
            end
        end
        if bad_cov,break;end
    end
end
report.pass=isempty(report.failures);
end

function ok=covariance_psd_3x3(p)
% Same conservative principal-minor test used by the production C guard.
if ~isequal(size(p),[3 3]) || any(~isfinite(p),'all') || any(diag(p)<0)
    ok=false;return;
end
d01=p(1,1)*p(2,2)-p(1,2)^2;
d02=p(1,1)*p(3,3)-p(1,3)^2;
d12=p(2,2)*p(3,3)-p(2,3)^2;
detp=det(p);
scale=max(1.0e-20,p(1,1)*p(2,2)*p(3,3));
tol2=1.0e-5*max([1.0e-20,p(1,1)*p(2,2),p(1,1)*p(3,3),p(2,2)*p(3,3)]);
tol3=1.0e-4*scale;
ok=d01>=-tol2 && d02>=-tol2 && d12>=-tol2 && detp>=-tol3;
end
