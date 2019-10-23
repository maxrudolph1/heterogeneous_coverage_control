center = [1 -1; 0 0];
sigma = .2*eye(2);
detSigma = det(sigma);

gaussC(1,1, sigma, detSigma, center)


function val = gaussC(x, y, sigma, detSigma, center)
xc = center(1, :)
yc = center(2, :)
exponent = ((x-xc).^2/sigma(1,1) + (y-yc).^2/sigma(2,2))./(2)
amplitude = 1 / (sqrt(detSigma) * 2*pi);
val = sum(amplitude  .* exp(-exponent));
end