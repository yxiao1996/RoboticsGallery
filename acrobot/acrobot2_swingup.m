p.g = 9.81;
p.m1 = 0.1;
p.m2 = 0.1;
p.L1 = 0.5;
p.L2 = 1;

% User-defined dynamics and objective functions
problem.func.dynamics = @(t,x,u)( acrobot2_dynamics(x,u,p) );
problem.func.pathObj = @(t,x,u)( sum(u.^2) );

% Problem bounds
problem.bounds.initialTime.low = 0;
problem.bounds.initialTime.upp = 0;
problem.bounds.finalTime.low = 0.5;
problem.bounds.finalTime.upp = 12.5;

problem.bounds.state.low = [-2*pi; -2*pi; -inf; -inf];
problem.bounds.state.upp = [2*pi; 2*pi; inf; inf];
problem.bounds.initialState.low = [0;0;0;0];
problem.bounds.initialState.upp = [0;0;0;0];
problem.bounds.finalState.low = [pi;0;0;0];
problem.bounds.finalState.upp = [pi;0;0;0];

problem.bounds.control.low = [-5; -5]; %-inf;
problem.bounds.control.upp = [5; 5]; %inf;

% Guess at the initial trajectory
problem.guess.time = [0,1];
problem.guess.state = [0, pi;
                       0, 0; 
                       0, 0; 
                       0, 0];
problem.guess.control = [0, 0;
                         0, 0];

% Select a solver:
problem.options.method = 'trapezoid';
problem.options.defaultAccuracy = 'medium';

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
