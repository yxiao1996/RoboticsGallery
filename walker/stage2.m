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

problem2.func.dynamics = @(t,x,u)( dynamics(x,u,param.dyn) );

problem2.func.pathObj = @(t,x,u)( sum(u.^2) );

problem2.func.bndCst = @(t0,x0,tF,xF)( gait(xF,x0,param.dyn) );


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%               Set up bounds on time, state, and control                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
t0 = 0;  tF = 2;
problem2.bounds.initialTime.low = t0;
problem2.bounds.initialTime.upp = t0;
problem2.bounds.finalTime.low = tF;
problem2.bounds.finalTime.upp = tF;

% State: [q1;q2;q3;dq1;dq2;dq3];

problem2.bounds.state.low = [-pi/2; -3*pi/4; -pi/2; -inf(3,1)];
problem2.bounds.state.upp = [ pi/2; 3*pi/4; pi/2;  inf(3,1)];

stepAngle = 0.2;
problem2.bounds.initialState.low = [3*stepAngle; -pi/2; -3*stepAngle; -inf(3,1)];
problem2.bounds.initialState.upp = [3*stepAngle; -pi/2; -3*stepAngle;  inf(3,1)];

problem2.bounds.control.low = [-5; -5]; %-inf;
problem2.bounds.control.upp = [5; 5]; %inf;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%              Create an initial guess for the trajectory                 %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

% For now, just assume a linear trajectory between boundary values
tA = (t0+tF)/2;
problem2.guess.time = [t0, tA, tF];

stepRate = (2*stepAngle)/(tF-t0);
x0 = [3*stepAngle; -pi/2; -3*stepAngle; -stepRate; stepRate; stepRate];
xA = [2*stepAngle; -2*pi/3; -2*stepAngle; -stepRate; stepRate; stepRate];
xF = [stepAngle; -pi/2; -stepAngle; -stepRate; stepRate; stepRate];
problem2.guess.state = [x0, xA, xF];

problem2.guess.control = [0 0 0;0 0 0];
method = 'hermiteSimpson';
% First iteration: get a more reasonable guess
problem2.options(1).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-3,...
    'MaxFunEvals',1e4);   %options for fmincon
problem2.options(1).verbose = 3; % How much to print out?
problem2.options(1).method = 'hermiteSimpson'; % Select the transcription method
problem2.options(1).hermiteSimpson.nSegment = 10;  %method-specific options


% Second iteration: refine guess to get precise soln
problem2.options(2).nlpOpt = optimset(...
    'Display','iter',...   % {'iter','final','off'}
    'TolFun',1e-6,...
    'MaxFunEvals',5e4);   %options for fmincon
problem2.options(2).verbose = 3; % How much to print out?
problem2.options(2).method = 'hermiteSimpson'; % Select the transcription method
problem2.options(2).hermiteSimpson.nSegment = 20;  %method-specific options

%%%%% THE KEY LINE:
soln2 = optimTraj(problem2);

% Transcription Grid points:
t_ = soln2(end).grid.time;
q1_ = soln2(end).grid.state(1,:);
q2_ = soln2(end).grid.state(2,:);
q3_ = soln2(end).grid.state(3,:);
dq1_ = soln2(end).grid.state(4,:);
dq2_ = soln2(end).grid.state(5,:);
dq3_ = soln2(end).grid.state(6,:);
u1_ = soln2(end).grid.control(1);
u2_ = soln2(end).grid.control(2);

% Interpolated solution:
tInt_ = linspace(t_(1),t_(end),10*length(t_)+1);
xInt_ = soln2(end).interp.state(tInt);
q1Int_ = xInt_(1,:);
q2Int_ = xInt_(2,:);
q3Int_ = xInt_(3,:);
dq1Int_ = xInt_(4,:);
dq2Int_ = xInt_(5,:);
dq3Int_ = xInt_(6,:);
%uInt = soln(end).interp.control(tInt)(1);

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Plot the solution                                   %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%

figure(100); clf;

subplot(3,1,1); hold on;
plot(tInt,q1Int_,'r-'); plot(tInt,q2Int_,'g-'); plot(tInt,q3Int_,'b-');
plot([t_(1),t_(end)],[0,0],'k--','LineWidth',1);
plot(t_,q1_,'ro');plot(t_,q2_,'go');plot(t_,q3_,'bo');
legend('leg one','body','leg two')
xlabel('time (sec)')
ylabel('angle (rad)')
title('Leg Angles')
