function [alphas, betas, RA, RB, tA, tB] = helper(e_bh, e_sc)

[N,~] = size(e_bh);
alphas = zeros(3, N-1);
betas = zeros(3, N-1);
RA = [];
RB = [];
tA = zeros(3, N-1);
tB = zeros(3, N-1);

quat_i = circshift( e_bh(1, 4:7), 1);
E_i = quat2tform(quat_i);
E_i(1:3, 4) = e_bh(1, 1:3).'; % .' is transpose

s_quat_i = circshift( e_sc(1, 4:7), 1);
S_i = quat2tform(s_quat_i);
S_i(1:3, 4) = e_sc(1, 1:3).';

for i=2:N
    quat_i_next = circshift( e_bh(i, 4:7), 1);
    E_i_next = quat2tform(quat_i_next);
    E_i_next(1:3, 4) = e_bh(i, 1:3).'; % .' is transpose
    
    s_quat_i_next = circshift( e_sc(i, 4:7), 1);
    S_i_next = quat2tform(s_quat_i_next);
    S_i_next(1:3, 4) = e_sc(i, 1:3).';

    Ai = inv(E_i) * E_i_next;
    Bi = S_i * inv(S_i_next);
    ai = logm( Ai(1:3,1:3) );
    bi = logm( Bi(1:3,1:3) );

    alpha_i = [ ai(3,2); ai(1,3) ; ai(2,1) ]; % extract from skew symmetric
    beta_i = [ bi(3,2); bi(1,3) ; bi(2,1) ];

    alphas(:,i-1) = alpha_i;
    betas(:,i-1) = beta_i;
    RA(:,:, i-1) = Ai(1:3,1:3) ;
    RB(:,:, i-1) = Bi(1:3,1:3);
    tA(:,i-1) = Ai(1:3,4);
    tB(:,i-1) = Bi(1:3,4);

    E_i = E_i_next;
    S_i = S_i_next;
end


end