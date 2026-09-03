function [core_next, surface_next] = thermal_step_tustin(core, surface, ...
    heat_W, ambient_C, dt_s, Cc, Cs, Rcs, Rsa)
%THERMAL_STEP_TUSTIN Independent trapezoidal two-node thermal update.
a = 1 ./ (Rcs .* Cc);
b = 1 ./ (Rcs .* Cs);
c = 1 ./ (Rsa .* Cs);
h = 0.5 .* dt_s;

rhs_core = (1 - h .* a) .* core + h .* a .* surface + ...
    dt_s .* heat_W ./ Cc;
rhs_surface = h .* b .* core + (1 - h .* (b + c)) .* surface + ...
    dt_s .* c .* ambient_C;

m00 = 1 + h .* a;
m01 = -h .* a;
m10 = -h .* b;
m11 = 1 + h .* (b + c);
detm = m00 .* m11 - m01 .* m10;
if any(abs(detm(:)) < 1e-12) || any(~isfinite(detm(:)))
    error('mil:thermal:Singular', 'Thermal Tustin update became singular.');
end
core_next = (m11 .* rhs_core - m01 .* rhs_surface) ./ detm;
surface_next = (-m10 .* rhs_core + m00 .* rhs_surface) ./ detm;
end
