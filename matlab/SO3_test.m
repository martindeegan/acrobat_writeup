R1 = SO3.identity();
R2 = SO3.exp((pi / 2) * [1; 0; 0]);
R3 = SO3.exp((pi / 2) * [0; 1; 0]);

e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];
basis = [e1, e2, e3];

% Run tests
f1 = R1 * e1;
f2 = R1 * e2;
f3 = R1 * e3;
rotated_basis = [f1, f2, f3]
correct = [e1, e2, e3]
assert(norm(rotated_basis - correct) < 1e-6);

f1 = R2 * e1;
f2 = R2 * e2;
f3 = R2 * e3;
rotated_basis = [f1, f2, f3]
correct = [e1, e3, -e2]
assert(norm(rotated_basis - correct) < 1e-6);

f1 = R3 * e1;
f2 = R3 * e2;
f3 = R3 * e3;
rotated_basis = [f1, f2, f3]
correct = [-e3, e2, e1]
assert(norm(rotated_basis - correct) < 1e-6);

f1 = R1 * R2 * R3 * e1;
f2 = R1 * R2 * R3 * e2;
f3 = R1 * R2 * R3 * e3;
rotated_basis = [f1, f2, f3]
correct = [e2, e3, e1]
assert(norm(rotated_basis - correct) < 1e-6);

R1.log()
assert(norm(R1.log() - [0; 0; 0]) <= 1e-6);
err = norm(R2.log() - [(pi / 2); 0; 0])
assert(norm(R2.log() - [(pi / 2); 0; 0]) <= 1e-6);
assert(norm(R3.log() - (pi / 2) * [0; 1; 0]) <= 1e-6);
