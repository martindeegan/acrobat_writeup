R1 = SO3.identity();
R2 = SO3.exp((pi / 2) * [1; 0; 0]);
R3 = SO3.exp((pi / 2) * [0; 1; 0]);
R4 = SO3.exp(1e-6 * [0; 1; 0]);

e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];
basis = [e1, e2, e3];

% Run tests

% exp and group action tests
rotated_basis = R1 * basis
correct = [e1, e2, e3]
assert(norm(rotated_basis - correct) < 1e-6);

rotated_basis = R2 * basis
correct = [e1, e3, -e2]
assert(norm(rotated_basis - correct) < 1e-6);

rotated_basis = R3 * basis
correct = [-e3, e2, e1]
assert(norm(rotated_basis - correct) < 1e-6);

rotated_basis = R1 * R2 * R3 * basis
correct = [e2, e3, e1]
assert(norm(rotated_basis - correct) < 1e-6);

% Log tests
assert(norm(R1.log() - [0; 0; 0]) <= 1e-6);
assert(norm(R2.log() - [(pi / 2); 0; 0]) <= 1e-6);
assert(norm(R3.log() - (pi / 2) * [0; 1; 0]) <= 1e-6);

% Inverse tests
id = R2 * R2.inverse();
assert(norm(id.matrix - eye(3)) <= 1e-6);

id = R3 * R3.inverse();
assert(norm(id.matrix - eye(3)) <= 1e-6);

id = R2.inverse() * R2;
assert(norm(id.matrix - eye(3)) <= 1e-6);

% Sophus test
w = [2; -1.5; 2.5];

R_expected = [
-0.307914 -0.190208  0.932207;
-0.733026 -0.577191 -0.359894;
 0.606516 -0.794149 0.0382983];

log_expected = [-1.55431;  1.16573; -1.94288];

Adj_expected = [
-0.307914 -0.190208  0.932207;
-0.733026 -0.577191 -0.359894;
 0.606516 -0.794149 0.0382983  
];

inv_expected = [
-0.307914 -0.733026  0.606516;
-0.190208 -0.577191 -0.794149;
 0.932207 -0.359894 0.0382983   
];

R = SO3.exp(w);
% Test exponential
assert(norm(R.matrix - R_expected) < 1e-6);
% Test group action
assert(norm(R*basis - R_expected) < 1e-6);
% Test logarithm
assert(norm(R.log() - log_expected) < 5e-6);
% Test adjoint
assert(norm(R.adjoint() - Adj_expected) < 1e-6);
% Test inverse
assert(norm(R.inverse().matrix - inv_expected) < 1e-6);



