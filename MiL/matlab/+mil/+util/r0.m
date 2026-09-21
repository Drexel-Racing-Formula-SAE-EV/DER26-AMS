function values = r0(params, soc, temperature)
%R0 P42A ohmic resistance lookup from existing plant parameters.
values = mil.util.lookup2d(params.R0, params.soc_common, ...
    params.temp_bp, soc, temperature);
end
