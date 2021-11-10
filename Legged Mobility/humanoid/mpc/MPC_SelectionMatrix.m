function S = MPC_MakeSelectionMatrix(i, n, N)

%
% S=SELECTIONMATRIX - Generates selection matrix S_i^(n,N)
%
% N: prediction horizon
% n: dimension of vector to be selected
% i: position to select within prediction horizon [1,N]
%

% initialize selection matrix
S = zeros(n, N*n);

% change i-th component to identity matrix
S(:, (i-1)*n + 1:i*n) = eye(n);

end




