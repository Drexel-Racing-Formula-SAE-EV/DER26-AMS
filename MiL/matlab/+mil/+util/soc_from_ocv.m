function [soc,in_range] = soc_from_ocv(params,ocv_V,temperature_C)
%SOC_FROM_OCV Monotonic inverse of the reviewed P42A OCV surface.
%
% This helper is used only by the independent MiL reference acquisition
% path. It consumes measured terminal voltage after current/polarization
% compensation; it never consumes hidden plant SoC truth.

soc_bp = double(params.soc_ocv_common(:));
temp_bp = double(params.temp_bp_ocv(:));
T = min(max(double(temperature_C),min(temp_bp)),max(temp_bp));

curve = zeros(size(soc_bp));
for k = 1:numel(soc_bp)
    curve(k) = interp1(temp_bp,double(params.OCV(k,:)),T,'linear');
end
if any(~isfinite(curve)) || any(diff(curve) <= 0)
    error('mil:ocv:NonMonotonic','P42A OCV curve must be finite and strictly increasing.');
end

v = double(ocv_V);
in_range = isfinite(v) && v >= curve(1) && v <= curve(end);
if ~isfinite(v)
    soc = NaN;
    return;
end
v = min(max(v,curve(1)),curve(end));
soc = interp1(curve,soc_bp,v,'linear');
soc = min(max(soc,0),1);
end
