% Generate synthetic data to test hand-eye calibration.
% e_bh and e_sc are Nx7 matrices that represent N E_bh and N E_sc
% transformations. Each row is of the form [ tx ty tz qx qy qz qw ]
% were tx, ty and tz denote a translation and qx, qy, qz, qw a
% quaternion.
% X is a randomly generated hand-eye transformation
function [e_bh, e_sc, X] = generatedata(N)
% Assume we know the transformation between the base and the
% checkerboard
E_bc = [ eye(3) [ 1; 0; 0 ]; 0 0 0 1 ];
% Create a random X for generating the data
X = randSE3();
e_bh = zeros(N,7);
e_sc = zeros(N,7);

for i=1:N
    quat = randrot(); % quaternion of E_bh
    [w,x,y,z] = parts(quat);
    t = rand(1,3); % translation of E_bh
    E_bh = quat2tform([w,x,y,z]);
    E_bh(1:3, 4) = t.';
    e_bh(i,:) = [t, x,y,z,w];
    
    E_sc = inv(X) * inv(E_bh) * E_bc;
    s_quat = circshift( tform2quat(E_sc), -1);
    e_sc(i,:) = [ E_sc(1:3,4).' , s_quat];
end

end

% Generate a random SE3 transformation
function Rt = randSE3()
    R = quat2rotm( randrot() );
    t = rand(3,1);
    Rt = [ R t; 0 0 0 1];
end