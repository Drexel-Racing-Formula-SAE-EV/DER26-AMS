function values = lookup2d(table_data, soc_bp, temp_bp, soc, temperature)
%LOOKUP2D Bilinear LUT lookup with explicit domain clamping.
soc_bp = double(soc_bp(:));
temp_bp = double(temp_bp(:));
soc = min(max(double(soc), min(soc_bp)), max(soc_bp));
temperature = min(max(double(temperature), min(temp_bp)), max(temp_bp));
values = interp2(temp_bp.', soc_bp, double(table_data), ...
    temperature, soc, 'linear');
end
