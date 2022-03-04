function X = axxb(e_bh, e_sc)

[alphas, betas, RA, RB, tA, tB] = helper(e_bh, e_sc);

Rx = solveRx(alphas, betas);
tx = solveTx(RA, tA, RB, tB, Rx);

X = [Rx tx; 0 0 0 1];

end