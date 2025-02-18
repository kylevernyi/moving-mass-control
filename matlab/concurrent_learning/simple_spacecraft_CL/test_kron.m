% Define a sample A matrix and b vector
Phi = [1, 2, 3; 4, 5, 6; 7, 8, 9];
Theta = [10; 20; 30];

% Compute vec(A)
PhiVec = Phi(:);

% Construct K(b)
Theta_map = kron(Theta', eye(3));

% Compute A * b using K and vec(A)
result = Theta_map * PhiVec;

% Verify against the standard matrix-vector multiplication
expected = Phi * Theta;

% Compare
disp(result);     % Result using K and vec(A)
disp(expected);   % Standard result
