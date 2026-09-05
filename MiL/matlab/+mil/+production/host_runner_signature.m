function signature = host_runner_signature(files)
%HOST_RUNNER_SIGNATURE Lightweight content signature for local host-runner sources.
%
% This is a rebuild/freshness guard, not a cryptographic integrity primitive.
% It intentionally avoids Java/toolbox dependencies so it works in ordinary
% MATLAB installations. Each dependency contributes its normalized path,
% byte count, byte sum and a position-weighted byte sum. A changed checked-in
% runner/source file therefore forces a rebuild even if an old executable has
% a newer filesystem timestamp after an overlay extraction.
if ischar(files)||isstring(files),files=cellstr(files);end
parts=cell(numel(files),1);
for k=1:numel(files)
    path=char(files{k});
    if ~isfile(path)
        error('mil:production:RunnerDependencyMissing', ...
            'Production host-runner dependency missing: %s',path);
    end
    fid=fopen(path,'rb');
    if fid<0
        error('mil:production:RunnerDependencyRead', ...
            'Could not read production host-runner dependency: %s',path);
    end
    cleanup=onCleanup(@()fclose(fid)); %#ok<NASGU>
    bytes=fread(fid,Inf,'*uint8');
    n=numel(bytes);
    values=double(bytes);
    if n==0
        byte_sum=0;
        weighted_sum=0;
    else
        byte_sum=sum(values);
        weights=mod((1:n).',65521)+1;
        weighted_sum=sum(values.*weights);
    end
    parts{k}=sprintf('%s|%d|%.0f|%.0f', ...
        strrep(path,'\','/'),n,byte_sum,weighted_sum);
    clear cleanup
end
signature=strjoin(parts,'||');
end
