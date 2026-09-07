function values = c1(params, soc, temperature)
%C1 P42A fast-polarization capacitance lookup.
values = mil.util.lookup2d(params.C1, params.soc_common, ...
    params.temp_bp, soc, temperature);
end
