%% Nonlinear Model Predictive Control (NLMPC)
% This function shows how to design nonlinear MPC controller for KINOVA. 
%
% Copyright 2019 The Mathworks, Inc. 

% Cost weights
Qr = diag([1 1 1 0 0 0]); % running cost weight on desired end-effector pose [x, y, z, phi, theta, psi]
Qt = diag([10 10 10 1 1 1]); % terminal cost weight on desired end-effector pose [x, y, z, phi, theta, psi]
Qu = diag([1 1 1 1 1 1 1])/10; % input cost weight on joint accelerations qDdot
Qv = diag([1 1 1 1 1 1 1])/10;

% Initialize nlmpc object (by modeling joints as double integrators).
nx = numJoints * 2; % [q,qDot]
ny = numJoints; % [q]
nu = numJoints; % [qDdot]
nlobj = nlmpc(nx,ny,nu);

% Set sample time, prediction horizon, and control horizon.
Ts = 1; % units in seconds
p = 5; 
c = 3;
% Solver time step
Ts = mpcTimeStep; % seconds
nlobj.Ts = Ts; 

% Configure NLMPC solver functions
nlobj.Model.StateFcn = @(x,u) nlmpcModel(x,u);  

nlobj.Model.OutputFcn = @(x,u) x(1:numJoints);

nlobj.Optimization.CustomCostFcn = @(X,U,e,data) nlmpcCostFunction(X,U,e,data, poseFinal, robot, endEffector, Qr, Qt, Qu, Qv); 

if ~isempty(world) && avoidCollisions
    nlobj.Optimization.CustomIneqConFcn = @(X,U,e,data) myIneqConFunction(X,U,e,data, collisionHelper , safetyDistance, world);
end

nlobj.Jacobian.OutputFcn = @(x,u) nlmpcJacobianOutputModel(x,u);

nlobj.Jacobian.StateFcn = @(x,u) nlmpcJacobianModel(x,u);

nlobj.Jacobian.CustomCostFcn = @(X,U,e,data) nlmpcJacobianCost(X,U,e,data, poseFinal, robot, endEffector, Qr, Qt, Qu, Qv);

if ~isempty(world) && avoidCollisions
    nlobj.Jacobian.CustomIneqConFcn = @(X,U,e,data) nlmpcJacobianConstraint(X,U,e,data, world, robot, collisionHelper);
end

nlobj.Optimization.SolverOptions.FunctionTolerance = 0.01;

nlobj.Optimization.SolverOptions.StepTolerance = 0.01;

nlobj.Optimization.SolverOptions.MaxIter = 5;

nlobj.Optimization.UseSuboptimalSolution = true;

nlobj.Optimization.ReplaceStandardCost = true;

% nlobj.Optimization.SolverOptions.Display = 'iter-detailed';

nlobj.Optimization.SolverOptions.ConstraintTolerance = 0.01;

% Set constraint on States and MV.
stateMinValues = {-174.53;-2.2000;-174.53;-2.5656;-174.53;-2.0500;-174.53;...
    -0.8727;-0.8727;-0.8727;-0.8727;-0.8727;-0.8727;-0.8727};
stateMaxValues = {174.53;2.2000;174.53;2.5656;174.53;2.0500;174.53;...
    0.8727;0.8727;0.8727;0.8727;0.8727;0.8727;0.8727};

nlobj.States = struct('Min',stateMinValues,...
    'Max',stateMaxValues);
nlobj.MV = struct('Min',{-1;-1;-1;-1;-10;-10;-10},'Max',{1;1;1;1;10;10;10});

% Time horizon in seconds
p = 2; 
nlobj.PredictionHorizon = p; % prediction horizon
nlobj.ControlHorizon = 1; % control horizon
