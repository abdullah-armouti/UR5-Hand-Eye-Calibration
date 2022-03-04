% RA , RB are 3x3xN matrices
% tA, tB are 3xN
function tx = solveTx(RA, tA, RB, tB, RX)

[~,N] = size(tA);

left = zeros(3*N, 3);
right = zeros(3*N, 1);

for i = 0:N-1
    left( (3*i)+1 : 3*(i+1), :) = eye(3) - RA(:,:,i+1);
    right( (3*i)+1 : 3*(i+1), :) = tA(:,i+1) - RX*tB(:,i+1);
end

tx = left \ right ;

end