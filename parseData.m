function [e_bh, e_sc] = parseData()

my_matrix = dlmread('data.txt');
[N,~] = size(my_matrix);
e_bh = my_matrix(1:N/2, :);
e_sc = my_matrix(N/2 + 1 : N, :);

end