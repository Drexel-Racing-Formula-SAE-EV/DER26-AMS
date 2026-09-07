function report = sop(production,oracle,acceptance)
%SOP Compare production limits to truth-based oracle with asymmetric safety cost.
% production and oracle are matrices [checkpoint x horizon]. Positive values
% are discharge magnitudes; pass charge magnitudes separately if desired.
prod = double(production);
ref = double(oracle);
if ~isequal(size(prod),size(ref))
    error('mil:sopmetrics:Dimensions','Production/oracle dimensions differ.');
end
unsafe = max(0,prod-ref);
conservative = max(0,ref-prod);
relative_unsafe = unsafe./max(ref,1.0);
report = struct();
report.max_unsafe_overprediction_A = max(unsafe,[],'all');
report.max_unsafe_overprediction_fraction = max(relative_unsafe,[],'all');
report.mean_conservatism_A = mean(conservative,'all');
report.p95_conservatism_A = percentile(conservative(:),95);
report.unsafe_count = nnz(unsafe > acceptance.numeric_tolerance_A);
report.pass = report.max_unsafe_overprediction_A <= ...
    acceptance.unsafe_overprediction_max_A && ...
    report.max_unsafe_overprediction_fraction <= ...
    acceptance.unsafe_overprediction_max_fraction;
end

function value = percentile(x,p)
x=sort(x(:)); if isempty(x),value=NaN;return;end
q=1+(numel(x)-1)*p/100; lo=floor(q); hi=ceil(q);
if lo==hi,value=x(lo);else,value=x(lo)+(q-lo)*(x(hi)-x(lo));end
end
