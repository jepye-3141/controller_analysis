function cmap = ryg_cmap(n_half)
% ryg_cmap  Red-yellow-green diverging colormap used for reachability figures.
%   0 = red, 0.5 = yellow, 1 = green. Default n_half=32 produces 64x3.
if nargin < 1
    n_half = 32;
end
cmap = [[ones(n_half, 1); linspace(1, 0, n_half).'], ...
        [linspace(0, 1, n_half).'; ones(n_half, 1)], ...
        zeros(2*n_half, 1)];
end
