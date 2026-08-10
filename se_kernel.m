% author : nikhil.sharma.mcmaster@gmail.com
function Y = se_kernel(X1, X2, length_scale, sigma_f)
    
    sqdist = sum(X1.^2, 2) + sum(X2.^2, 2)' - (2*X1)*X2';
    
    Y = sigma_f^2 * exp( (-0.5 .* sqdist) ./ (length_scale^2));
end