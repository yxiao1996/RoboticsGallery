p.g = 9.81;
p.m1 = 0.1;
p.m2 = 0.1;
p.m3 = 0.1;
p.L1 = 0.4;
p.L2 = 0.4;
p.L3 = 0.8;
t0 = 0;
tF = 2.0;
x0 = [0;0;0];
dx0 = [0;0;0];
xF = [pi;pi;pi];
dxF = [0;0;0];
% User-defined dynamics and objective functions
problem.func.dynamics = @(t,x,u)( acrobot3_dynamics_sym(x,u,p) );
problem.func.pathObj = @(t,x,u)( sum(u.^2) );

% Problem bounds
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low = tF;
problem.bounds.finalTime.upp = tF;

problem.bounds.state.low = [-2*pi; -2*pi; -2*pi; -inf; -inf; -inf];
problem.bounds.state.upp = [2*pi; 2*pi; 2*pi; inf; inf; inf];
problem.bounds.initialState.low = [0;0;0;0;0;0];
problem.bounds.initialState.upp = [0;0;0;0;0;0];
problem.bounds.finalState.low = [xF;dxF];
problem.bounds.finalState.upp = [xF;dxF];

problem.bounds.control.low = [-5; -5; -5]; %-inf;
problem.bounds.control.upp = [5; 5; 5]; %inf;

% Guess at the initial trajectory
tA = t0 + 0.25 * (tF - t0);
tB = t0 + 0.75 * (tF - t0);
xA = [-pi/2;-pi/2;-pi/2];
dxA = [0;0;0];
xB = [pi/2;pi/2;pi/2];
dxB = [0;0;0];
problem.guess.time = [t0,tA,tB,tF];
problem.guess.state = [[x0;dx0], [xA; dxA],[xB; dxB], [xF;dxF]];
problem.guess.control = [0,0,0,0;
                         0,0,0,0;
                         0,0,0,0];

% Select a solver:
problem.options(1).method = 'trapezoid';
problem.options(1).trapezoid.nGrid = 20;
%problem.options(2).method = 'trapezoid';
%problem.options(2).trapezoid.nGrid = 40;
%problem.options(3).method = 'trapezoid';
%problem.options(3).trapezoid.nGrid = 60;

% Solve the problem
soln = optimTraj(problem);
t = soln.grid.time;
q1 = soln.grid.state(1,:);
q2 = soln.grid.state(2,:);
u = soln.grid.control(2,:);

% Plot the solution:
figure(1); clf;

subplot(2,1,1)
plot(t,q1)
ylabel('q')
title('Acrobot Swing-Up');

subplot(2,1,2)
plot(t,q2)
ylabel('q2')
