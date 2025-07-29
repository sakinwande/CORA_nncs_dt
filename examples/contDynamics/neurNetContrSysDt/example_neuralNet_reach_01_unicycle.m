function [completed,res,tTotal] = example_neuralNet_reach_01_unicycle
% example_neuralNet_reach_01_unicycle - example of reachability analysis
%    for a neural network controlled system
%
% Syntax:
%    completed = example_neuralNet_reach_01_unicycle()
%
% Inputs:
%    -
%
% Outputs:
%    completed - true/false
%    res - verification result
%    tTotal - total time
%
% Reference:
%   [1] Johnson, Taylor T., et al. "ARCH-COMP21 Category Report:
%       Artificial Intelligence and Neural Network Control Systems (AINNCS)
%       for Continuous and Hybrid Systems Plants."
%       EPiC Series in Computing 80 (2021): 90-119.

% Authors:       Niklas Kochdumper, Tobias Ladner
% Written:       17-September-2021
% Last update:   20-May-2022 (TL, ARCH'22 revisions)
%                30-March-2022 (TL, ARCH'23 revisions)
% Last revision: ---

% ------------------------------ BEGIN CODE -------------------------------

disp("BENCHMARK: Sherlock-Benchmark 10 (Unicycle Car Model)");

% Parameters --------------------------------------------------------------

params.tFinal = 10;
params.R0 = polyZonotope(interval([9.5; -4.5; 2.1; 1.5], [9.55; -4.45; 2.11; 1.51]));

% Reachability Settings ---------------------------------------------------
dt = 0.01;
options.timeStep = dt;
options.alg = 'lin';
options.tensorOrder = 2;
options.taylorTerms = 1;
options.zonotopeOrder = 10;

% Options for NN evaluation -----------------------------------------------
%Options defined as needed to run the code
options.nn = struct();
options.nn.poly_method = "singh";
options.nn.num_generators = [];
options.nn.do_pre_order_reduction = false;
options.nn.sort_exponents = true;
options.nn.reuse_bounds = false;
options.nn.bound_approx = "sample";
options.nn.use_approx_error = false;

% System Dynamics ---------------------------------------------------------

% open-loop system
f = @(x, u) [x(4) * cos(x(3)); x(4) * sin(x(3)); u(2) - 20; u(1) - 20 + u(3)];
sys = nonlinearSysDT(f, options.timeStep);

% load neural network controller
% [4, 500, 2]
load('controllerUnicycle.mat');
nn = neuralNetwork.getFromCellArray(W,b,'ReLU');

% construct neural network controlled system
sys = neurNetContrSysDt(sys, nn, dt);

% Specification -----------------------------------------------------------

goalSet = interval(-[0.6; 0.2; 0.06; 0.3], [0.6; 0.2; 0.06; 0.3]);
spec = specification(goalSet, 'safeSet', interval(params.tFinal));

% Verification ------------------------------------------------------------

t = tic;
[res, R, simRes] = verify(sys, spec, params, options, true);
tTotal = toc(t);
disp(['Result: ' res])

% Visualization -----------------------------------------------------------

disp("Plotting..")
figure; hold on; box on;

% plot goal set
plot(specification(goalSet, 'safeSet'), [1, 2], 'DisplayName', 'Goal set');

% plot reachable set
useCORAcolors("CORA:contDynamics")
plot(R, [1, 2], 'DisplayName', 'Reachable set');

% plot initial set
plot(R(1).R0, [1, 2], 'DisplayName', 'Initial set');

% plot simulations
plot(simRes,[1, 2], 'DisplayName', 'Simulations');

% labels and legend
xlabel('x'); ylabel('y');
legend()


% example completed -------------------------------------------------------

completed = true;

% handling for ARCH competition
if nargout < 2
    clear res;
end
if nargout < 3
    clear tTotal;
end

end

% ------------------------------ END OF CODE ------------------------------
