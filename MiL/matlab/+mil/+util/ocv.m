function values = ocv(params, soc, temperature)
%OCV P42A OCV lookup from the existing reviewed plant parameter package.
values = mil.util.lookup2d(params.OCV, params.soc_ocv_common, ...
    params.temp_bp_ocv, soc, temperature);
end
