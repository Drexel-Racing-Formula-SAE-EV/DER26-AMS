function campaign = monte_carlo(base_scenario,count,varargin)
%MONTE_CARLO Reproducible uncertainty campaign around one named scenario.
p=inputParser();p.addParameter('OutputDirectory','',@(x)ischar(x)||isstring(x));
p.addParameter('RunSoP',[],@(x)isempty(x)||(islogical(x)&&isscalar(x)));p.parse(varargin{:});opt=p.Results;
if count<1||mod(count,1)~=0,error('mil:mc:Count','count must be positive integer.');end
base=mil.load_scenario(string(base_scenario));paths=mil.project_paths();out=char(opt.OutputDirectory);if isempty(out),out=fullfile(paths.output_root,['mc_' char(base_scenario)]);end
if ~isfolder(out),mkdir(out);end
summary_cells=cell(count,1);seeds=zeros(count,1);
for k=1:count
    cfg=base;cfg.id=sprintf('%s_mc_%04d',base.id,k);cfg.seed=base.seed+k*7919;cfg.plant.seed=cfg.seed;cfg.sensor.seed=cfg.seed+1;
    % Plant variation is already stochastic-but-bounded. Vary sensor bias/noise
    % independently from deterministic draws while preserving replayability.
    st=RandStream('mt19937ar','Seed',cfg.seed+101);
    cfg.initial_soc=min(max(cfg.initial_soc+0.05*(2*rand(st)-1),0.08),0.95);
    cfg.initial_temperature_C=5+35*rand(st);
    cfg.ambient_temperature_C=cfg.initial_temperature_C;
    cfg.sensor.voltage_bias_V=0.0015*randn(st);
    cfg.sensor.temperature_bias_C=0.35*randn(st);
    cfg.sensor.current.bias_50A_A=0.15*randn(st);
    cfg.sensor.current.bias_800A_A=0.75*randn(st);
    cfg.sensor.current.gain_50A=1+0.005*randn(st);
    cfg.sensor.current.gain_800A=1+0.010*randn(st);
    cfg.sensor.timestamp_jitter_std_s=min(abs(0.005*randn(st)),0.020);
    if isempty(cfg.plant.overrides) && rand(st)<0.25
        group=1+floor(75*rand(st));
        cfg.plant.overrides=struct('group_index',group, ...
            'capacity_multiplier',0.85+0.15*rand(st), ...
            'r0_multiplier',1.0+0.35*rand(st));
    end
    if isempty(cfg.sensor.faults) && rand(st)<0.05
        start_s=max(1,0.2*double(profile_duration(cfg))*rand(st));
        cfg.sensor.faults=struct('type','cell_stale','start_s',start_s, ...
            'end_s',start_s+0.2+0.8*rand(st),'target',1+floor(75*rand(st)), ...
            'value',300);
    end
    args={'OutputDirectory',fullfile(out,cfg.id),'Export',false};
    if ~isempty(opt.RunSoP),args=[args,{'RunSoP',opt.RunSoP}];end %#ok<AGROW>
    r=mil.run_scenario(cfg,args{:});
    summary_cells{k}=r.summary;seeds(k)=cfg.seed;
end
% Stage scalar summaries in cells and concatenate only after real records
% exist. This avoids MATLABs dissimilar-structure indexed assignment trap
% and makes Monte Carlo aggregation consistent with deterministic campaigns.
summaries=vertcat(summary_cells{:});
campaign=struct('base_scenario',char(base_scenario),'count',count,'seeds',seeds,'summaries',summaries,'pass_fraction',mean([summaries.pass]));
writetable(struct2table(summaries),fullfile(out,'monte_carlo_summary.csv'));
fid=fopen(fullfile(out,'monte_carlo_manifest.json'),'w');fprintf(fid,'%s\n',jsonencode(rmfield(campaign,'summaries'),'PrettyPrint',true));fclose(fid);
end

function duration=profile_duration(cfg)
if isnumeric(cfg.stop_time_s),duration=double(cfg.stop_time_s);return;end
if isfield(cfg,'profile')&&isfield(cfg.profile,'duration_s')
    duration=sum(double(cfg.profile.duration_s));
else
    duration=600;
end
end
