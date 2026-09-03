function x = chi2inv_local(p, dof)
%CHI2INV_LOCAL Chi-square inverse without Statistics Toolbox.
if any(p(:) <= 0 | p(:) >= 1) || any(dof(:) <= 0)
    error('mil:stats:Domain', 'p must be in (0,1), dof > 0.');
end
x = 2 .* gammaincinv(p, dof ./ 2, 'lower');
end
