%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                  Parameters for the dynamics function                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
param.dyn.Ml = 1;  %leg mass
param.dyn.Mb = 20;
param.dyn.I = 1;  %leg inertia about CoM
param.dyn.g = 9.81;  %gravity
param.dyn.Lb = 1;  %leg length
param.dyn.Ll = 1;
param.dyn.d = 0.5;  %Leg CoM distance from hip

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                       Set up function handles                           %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

problem.func.dynamics = @(t,x,u)( dynamics(x,u,param.dyn) );

problem.func.pathObj = @(t,x,u)( sum(u.^2) );

problem.func.bndCst = @(t0,x0,tF,xF)( gait(xF,x0,param.dyn) );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t0 = 0;  tF = 2;
problem.bounds.initialTime.low = t0;
problem.bounds.initialTime.upp = t0;
problem.bounds.finalTime.low = tF;
problem.bounds.finalTime.upp = tF;

% State: [q1;q2;q3;dq1;dq2;dq3];

problem.bounds.state.low = [-pi/2; -3*pi/4; -pi/2; -inf(3,1)];
problem.bounds.state.upp = [ pi/2; 3*pi/4; pi/2;  inf(3,1)];

stepAngle = 0.2;
problem.bounds.initialState.low = [stepAngle; pi/2; -stepAngle; -inf(3,1)];
problem.bounds.initialState.upp = [stepAngle; pi/2; -stepAngle;  inf(3,1)];

problem.bounds.control.low = [-5; -5]; %-inf;
problem.bounds.control.upp = [5; 5]; %inf;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% For now, just assume a linear trajectory between boundary values
tA = (t0+tF)/2;
problem.guess.time = [t0, tA, tF];

stepRate = (2*stepAngle)/(tF-t0);
x0 = [-stepAngle; pi/2; stepAngle; stepRate; -stepRate; -stepRate];
xA = [-2*stepAngle; 2*pi/3; 2*stepAngle; stepRate; -stepRate; -stepRate];
xF = [-3*stepAngle; pi/2; 3*stepAngle; stepRate; -stepRate; -stepRate];
problem.guess.state = [x0, xA, xF];

problem.guess.control = [0 0 0;0 0 0];
method = 'hermiteSimpson';
% First iteration: get a more reasonable guess
problem.options(1).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-3,...
    'MaxFunEvals',1e4);   %options for fmincon
problem.options(1).verbose = 3; % How much to print out?
problem.options(1).method = 'hermiteSimpson'; % Select the transcription method
problem.options(1).hermiteSimpson.nSegment = 10;  %method-specific options


% Second iteration: refine guess to get precise soln
problem.options(2).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-6,...
    'MaxFunEvals',5e4);   %options for fmincon
problem.options(2).verbose = 3; % How much to print out?
problem.options(2).method = 'hermiteSimpson'; % Select the transcription method
problem.options(2).hermiteSimpson.nSegment = 20;  %method-specific options

%%%%% THE KEY LINE:
soln = optimTraj(problem);

% Transcription Grid points:
t = soln(end).grid.time;
q1 = soln(end).grid.state(1,:);
q2 = soln(end).grid.state(2,:);
q3 = soln(end).grid.state(3,:);
dq1 = soln(end).grid.state(4,:);
dq2 = soln(end).grid.state(5,:);
dq3 = soln(end).grid.state(6,:);
u1 = soln(end).grid.control(1);
u2 = soln(end).grid.control(2);

% Interpolated solution:
tInt = linspace(t(1),t(end),10*length(t)+1);
xInt = soln(end).interp.state(tInt);
q1Int = xInt(1,:);
q2Int = xInt(2,:);
q3Int = xInt(3,:);
dq1Int = xInt(4,:);
dq2Int = xInt(5,:);
dq3Int = xInt(6,:);
%uInt = soln(end).interp.control(tInt)(1);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(100); clf;

subplot(3,1,1); hold on;
plot(tInt,q1Int,'r-'); plot(tInt,q2Int,'g-'); plot(tInt,q3Int,'b-');
plot([t(1),t(end)],[0,0],'k--','LineWidth',1);
plot(t,q1,'ro');plot(t,q2,'go');plot(t,q3,'bo');
legend('leg one','body','leg two')
xlabel('time (sec)')
ylabel('angle (rad)')
title('Leg Angles')
