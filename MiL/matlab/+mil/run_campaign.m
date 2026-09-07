function campaign = run_campaign(scenarios,varargin)
%RUN_CAMPAIGN Run a deterministic set of named MiL scenarios.
p=inputParser();
p.addParameter('OutputDirectory','',@(x)ischar(x)||isstring(x));
p.addParameter('RunSoP',[],@(x)isempty(x)||(islogical(x)&&isscalar(x)));
p.addParameter('StopOnFailure',false,@(x)islogical(x)&&isscalar(x));
p.parse(varargin{:});opt=p.Results;
if ischar(scenarios)||isstring(scenarios),scenarios=cellstr(scenarios);end
paths=mil.project_paths();base=char(opt.OutputDirectory);if isempty(base),base=fullfile(paths.output_root,'campaign');end
if ~isfolder(base),mkdir(base);end
summaries=[];results=cell(numel(scenarios),1);
for k=1:numel(scenarios)
    name=char(scenarios{k});fprintf('[MiL] %d/%d %s\n',k,numel(scenarios),name);
    args={'OutputDirectory',fullfile(base,name),'Export',true};
    if ~isempty(opt.RunSoP),args=[args,{'RunSoP',opt.RunSoP}];end %#ok<AGROW>
    results{k}=mil.run_scenario(name,args{:});
    current_summary=results{k}.summary;
    if k == 1
        % MATLAB does not allow assigning a populated struct into struct([])
        % by indexed assignment because the field schemas are dissimilar.
        % Seed the campaign array from the first real scenario instead.
        summaries=current_summary;
    else
        % summarize_result is intentionally a fixed-schema contract. Guard it
        % explicitly so a future scenario-specific field cannot turn into an
        % opaque "Subscripted assignment between dissimilar structures".
        expected_fields=fieldnames(summaries(1));
        actual_fields=fieldnames(current_summary);
        if ~isequal(sort(expected_fields),sort(actual_fields))
            missing=setdiff(expected_fields,actual_fields);
            extra=setdiff(actual_fields,expected_fields);
            error('mil:run_campaign:SummarySchemaMismatch', ...
                ['Scenario %s returned a summary schema different from the ', ...
                 'campaign schema. Missing: %s. Extra: %s.'], ...
                name,strjoin(missing,', '),strjoin(extra,', '));
        end
        current_summary=orderfields(current_summary,summaries(1));
        summaries(k,1)=current_summary; %#ok<AGROW>
    end
    if opt.StopOnFailure && ~current_summary.pass
        results=results(1:k);summaries=summaries(1:k);scenarios=scenarios(1:k);
        break;
    end
end
campaign=struct('scenarios',{scenarios},'summaries',summaries,'results',{results},'pass',all([summaries.pass]));
writetable(struct2table(summaries),fullfile(base,'campaign_summary.csv'));
fid=fopen(fullfile(base,'campaign_summary.json'),'w');fprintf(fid,'%s\n',jsonencode(summaries,'PrettyPrint',true));fclose(fid);
manifest=struct('schema_version',1,'scenarios',{scenarios},'count',numel(summaries), ...
    'pass',campaign.pass,'run_sop_override',opt.RunSoP, ...
    'mil_configuration_hashes',{string({summaries.mil_configuration_hash})}, ...
    'plant_configuration_hashes',{string({summaries.plant_configuration_hash})}, ...
    'parameter_hashes',{string({summaries.parameter_hash})});
fid=fopen(fullfile(base,'campaign_manifest.json'),'w');fprintf(fid,'%s\n',jsonencode(manifest,'PrettyPrint',true));fclose(fid);
end
