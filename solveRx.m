% alphas, betas are each 3xN matrices
function Rx = solveRx(alphas, betas)

% obtain 3x3 matrix M
[~, N] = size(alphas);
M = zeros(3);
for i = 1:N
    tmp = betas(:,i) * alphas(:,i).' ;
    M = M + tmp;
end

% polar decomposition
Rx = mpower(M.' * M, -1/2) * M.' ;
end