function profile = resolve_profile(cfg, sim_cfg, cell_cfg, pack_cfg)
%RESOLVE_PROFILE Resolve an existing plant profile or a MiL-local sequence.
%
% MiL-local profiles are intentionally simple current/ambient vectors. They
% exercise the existing battery plant without adding a second plant model.
if ~isfield(cfg,'profile') || isempty(cfg.profile)
    profile = hil.load_profile(sim_cfg,cell_cfg,pack_cfg);
    return;
end
p = cfg.profile;
kind = lower(string(p.kind));
dt = double(sim_cfg.sample_time_s);
switch kind
    case "segments"
        if ~isfield(p,'duration_s') || ~isfield(p,'current_A')
            error('mil:profile:Segments','segments requires duration_s and current_A.');
        end
        duration = double(p.duration_s(:));
        current = double(p.current_A(:));
        if numel(duration) ~= numel(current) || any(duration <= 0)
            error('mil:profile:Segments','duration/current vectors must match and durations be positive.');
        end
        if isfield(p,'ambient_C')
            ambient_seg = double(p.ambient_C(:));
            if isscalar(ambient_seg), ambient_seg=repmat(ambient_seg,numel(duration),1); end
            if numel(ambient_seg) ~= numel(duration)
                error('mil:profile:Ambient','ambient_C must be scalar or one value per segment.');
            end
        else
            ambient_seg = repmat(double(cfg.ambient_temperature_C),numel(duration),1);
        end
        t=[]; i=[]; a=[]; cursor=0;
        for k=1:numel(duration)
            count=max(1,round(duration(k)/dt));
            tk=cursor+(0:count-1).'*dt;
            t=[t;tk]; %#ok<AGROW>
            i=[i;repmat(current(k),count,1)]; %#ok<AGROW>
            a=[a;repmat(ambient_seg(k),count,1)]; %#ok<AGROW>
            cursor=cursor+count*dt;
        end
        % Include one final sample so plant/profile time contracts remain
        % explicit at the end of the final segment.
        t=[t;cursor]; i=[i;current(end)]; a=[a;ambient_seg(end)];
        profile = make_profile(t,i,a,char(field_or(p,'name',cfg.id)),kind);
    case "array"
        required={'time_s','current_A'};
        for k=1:numel(required)
            if ~isfield(p,required{k}), error('mil:profile:Array','array profile missing %s.',required{k}); end
        end
        t=double(p.time_s(:)); i=double(p.current_A(:));
        if isfield(p,'ambient_C')
            a=double(p.ambient_C(:));
            if isscalar(a),a=repmat(a,numel(t),1);end
        else
            a=repmat(double(cfg.ambient_temperature_C),numel(t),1);
        end
        if numel(t)~=numel(i)||numel(t)~=numel(a)
            error('mil:profile:Array','time/current/ambient lengths must match.');
        end
        % Enforce the same uniformly-sampled contract as run_reference.
        if numel(t)<2 || max(abs(diff(t)-dt))>1e-8
            error('mil:profile:SampleTime','array profile must be uniformly sampled at simulation sample time.');
        end
        profile=make_profile(t,i,a,char(field_or(p,'name',cfg.id)),kind);
    case "csv"
        if ~isfield(p,'path')
            error('mil:profile:Csv','csv profile requires path.');
        end
        paths=mil.project_paths();source=char(p.path);
        if ~isfile(source),source=fullfile(paths.repo_root,source);end
        if ~isfile(source),error('mil:profile:CsvMissing','CSV profile not found: %s.',source);end
        T=readtable(source,'VariableNamingRule','preserve');names=lower(string(T.Properties.VariableNames));
        if any(names=="time_s")
            t=double(T{:,find(names=="time_s",1)});
        elseif any(names=="timestamp_ms")
            t=double(T{:,find(names=="timestamp_ms",1)})/1000;
        else
            error('mil:profile:CsvTime','CSV requires time_s or timestamp_ms.');
        end
        if any(names=="current_a")
            i=double(T{:,find(names=="current_a",1)});
        elseif any(names=="pack_current_a")
            i=double(T{:,find(names=="pack_current_a",1)});
        else
            error('mil:profile:CsvCurrent','CSV requires current_a or pack_current_A.');
        end
        if any(names=="ambient_temp_c")
            a=double(T{:,find(names=="ambient_temp_c",1)});
        elseif any(names=="temperature_proxy_c")
            a=double(T{:,find(names=="temperature_proxy_c",1)});
        else
            a=repmat(double(cfg.ambient_temperature_C),numel(t),1);
        end
        if numel(t)<2 || any(~isfinite(t)) || any(diff(t)<=0)
            error('mil:profile:CsvTime','CSV time must be finite and strictly increasing.');
        end
        tq=(t(1):dt:t(end)).';
        i=interp1(t,i,tq,'previous','extrap');
        a=interp1(t,a,tq,'linear','extrap');
        profile=make_profile(tq,i,a,char(field_or(p,'name',cfg.id)),kind);
        profile.source=source;
    otherwise
        error('mil:profile:Kind','Unsupported MiL-local profile kind %s.',kind);
end
end

function profile=make_profile(t,i,a,name,kind)
profile=struct('time_s',double(t(:)),'pack_current_A',double(i(:)), ...
    'ambient_temperature_C',double(a(:)),'name',name,'source','MiL scenario', ...
    'sample_time_s',median(diff(t)),'repeat_policy','one_shot', ...
    'scaling_mode','vehicle_current_replay','inputs_resolved',true, ...
    'mil_profile_kind',char(kind));
end

function value=field_or(s,name,default)
if isfield(s,name),value=s.(name);else,value=default;end
end
