R = SO3.exp((pi / 2) * [1; 0; 0]);
t = [1; 1; 1];
T = SE3(R, t);

e1 = [1; 0; 0];
e2 = [0; 1; 0];
e3 = [0; 0; 1];
basis = [e1, e2, e3];

% Run tests

% exp and group action tests
transformed_basis = T * basis
expected_basis = [e1 + t, e3 + t, -e2 + t]
assert(norm(transformed_basis - expected_basis) <= 1e-6);

% Log tests
assert(norm(SE3.identity().log()) <= 1e-6);

xi = [2;   -1.5;    2.5;    1.5;     -2; 115.25];
T_expected = [ 
 -0.571765  -0.820394 0.00622004  0.0670535;
  0.819684  -0.571558 -0.0379405 -0.0278006;
 0.0346813 -0.0165946   0.999261    2.55071;
         0          0          0          1];
     
expected_basis = [
-0.504711 -0.75334 0.0732735;
0.791884 -0.599358 -0.0657411;
2.58539 2.53411 3.54997];

log_expected = [0.0703902; -0.071793;    2.5499; 0.0283635; -0.037818;   2.17926];

Adj_expected = [  
  -0.571765   -0.820394  0.00622004    -2.09174     1.45834    0.068995;
   0.819684   -0.571558  -0.0379405    -1.46073    -2.09147  -0.0511384;
  0.0346813  -0.0165946    0.999261   0.0390672  -0.0611324 -0.00237112;
          0           0           0   -0.571765   -0.820394  0.00622004;
          0           0           0    0.819684   -0.571558  -0.0379405;
          0           0           0   0.0346813  -0.0165946    0.999261];
      
inv_expected = [
 -0.571765   0.819684  0.0346813 -0.0273351;
 -0.820394  -0.571558 -0.0165946  0.0814485;
0.00622004 -0.0379405   0.999261   -2.55029;
         0          0          0          1];
     
T = SE3.exp(xi);
% Test exponential
assert(norm(T.matrix() - T_expected) <= 5e-6);
% Test group action
assert(norm(T*basis - expected_basis) < 5e-5);
% Test logarithm
assert(norm(T.log() - log_expected) < 5e-6);
% Test adjoint
assert(norm(T.adjoint() - Adj_expected) < 5e-6);
% Test inverse
assert(norm(T.inverse().matrix() - inv_expected) < 5e-6);

