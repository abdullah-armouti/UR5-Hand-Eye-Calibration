% e_bh is 3x7 matrix [tx,ty,tz, quaternion]
% e_sc is same as ^
function X = axxb_closedform( e_bh, e_sc)
%use input to compute relative transformations
% we need A1, B1, A2, B2

quat_1 = circshift( e_bh(1, 4:7), 1);
E_1 = quat2tform(quat_1);
E_1(1:3, 4) = e_bh(1, 1:3).'; % .' is transpose

s_quat_1 = circshift( e_sc(1, 4:7), 1);
S_1 = quat2tform(s_quat_1);
S_1(1:3, 4) = e_sc(1, 1:3).';

quat_2 = circshift( e_bh(2, 4:7), 1);
E_2 = quat2tform(quat_2);
E_2(1:3, 4) = e_bh(2, 1:3).';

s_quat_2 = circshift( e_sc(2, 4:7), 1);
S_2 = quat2tform(s_quat_2);
S_2(1:3, 4) = e_sc(2, 1:3).';

quat_3 = circshift( e_bh(3, 4:7), 1);
E_3 = quat2tform(quat_3);
E_3(1:3, 4) = e_bh(3, 1:3).';

s_quat_3 = circshift( e_sc(3, 4:7), 1);
S_3 = quat2tform(s_quat_3);
S_3(1:3, 4) = e_sc(3, 1:3).';

%------------
A1 = inv(E_1) * E_2;
B1 = S_1 * inv(S_2);
A2 = inv(E_1) * E_3;
B2 = S_1 * inv(S_3);

a1 = logm( A1(1:3,1:3) );
b1 = logm( B1(1:3,1:3) );
a2 = logm( A2(1:3,1:3) );
b2 = logm( B2(1:3,1:3) );

alpha_1 = [ a1(3,2); a1(1,3) ; a1(2,1) ]; % extract from skew symmetric
beta_1 = [ b1(3,2); b1(1,3) ; b1(2,1) ];
alpha_2 = [ a2(3,2); a2(1,3) ; a2(2,1) ];
beta_2 = [ b2(3,2); b2(1,3) ; b2(2,1) ];

alpha = [alpha_1, alpha_2, cross(alpha_1, alpha_2)];
beta = [beta_1, beta_2, cross(beta_1, beta_2)];
Rx = alpha * inv(beta);

% now solve for translation of X, call it t
left = [A1(1:3,1:3) - eye(3); A2(1:3,1:3) - eye(3)];
right = [Rx*B1(1:3, 4) - A1(1:3, 4) ; Rx*B2(1:3, 4) - A2(1:3, 4)];
t = left \ right;

% assemble transformation matrix X
X = [Rx, t; 0 0 0 1];

end