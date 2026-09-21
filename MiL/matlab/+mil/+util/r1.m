function values = r1(params, soc, temperature)
%R1 P42A fast-polarization resistance lookup.
values = mil.util.lookup2d(params.R1, params.soc_common, ...
    params.temp_bp, soc, temperature);
end
