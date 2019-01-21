clc; clear;

syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 'real'; % states
syms u1 u2 'real'; % elbow and hip torque
syms Mb Ml Lb Ll d I g 'real';  % physical parameters

% d = distance along leg from hip/elbow to the center of mass of the leg
% Ml = mass of each leg
% Mb = mass of the body
% I = moment of inertia of each leg about its center of mass
% g = gravity
% Ll = leg length
% Lb = body length

%%%% Unit vectors:
i = sym([1;0]);
j = sym([0;1]);

e1 = cos(q1)*(-j) + sin(q1)*(i);    % hip -> stance foot
e2 = cos(q2)*(-j) + sin(q2)*(i);    % hip -> body
e3 = cos(q3)*(-j) + sin(q3)*(i);    % elbow -> swing foot

%%%% State vectors
x = [q1;q2;q3;dq1;dq2;dq3];
dx = [dq1;dq2;dq3;ddq1;ddq2;ddq3];

%%%% Kinematics
pHip = -Ll*e1;
pElb = pHip + e2*Lb;
p1 = pHip + d*e1; % center of mass for leg onr
p2 = pHip + (Ll/2)*e2; % center of mass for body
p3 = pElb + d*e3; % center of mass for leg two

dp1 = jacobian(p1,x)*dx;
dp2 = jacobian(p2,x)*dx;
dp3 = jacobian(p3,x)*dx;

ddp1 = jacobian(dp1,x)*dx;
ddp2 = jacobian(dp2,x)*dx;
ddp3 = jacobian(dp3,x)*dx;

%%%% Define a function for doing '2d' cross product: dot(a x b, k)
cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

%%%% Angular momentum balance of system about stance foot (origin)
sumTorques1 = cross2d(p1,-Ml*g*j) + cross2d(p2,-Mb*g*j) + cross2d(p3,-Ml*g*j);
sumInertial1 = cross2d(p1,Ml*ddp1) + I*ddq1 ...
             + cross2d(p2,Mb*ddp2) + I*ddq2 ...
             + cross2d(p3,Ml*ddp3) + I*ddq3;
eqn1 = sumTorques1 - sumInertial1;

sumTorques2 = cross2d(p2-p1,-Mb*g*j) + cross2d(p3-p1,-Ml*g*j) + u1;
sumInertial2 = cross2d(p2-p1,Mb*ddp2) + I*ddq2 ...
             + cross2d(p3-p1,Ml*ddp3) + I*ddq3;
eqn2 = sumTorques2 - sumInertial2;

sumTorques3 = cross2d(p3-p2,-Ml*g*j) + u2;
sumInertial3 = cross2d(p3-p2,Ml*ddp3) + I*ddq3;
eqn3 = sumTorques3 - sumInertial3;

%%%% Solve dynamics:
ddq = [ddq1;ddq2;ddq3];
eqns = [eqn1;eqn2;eqn3];
[MM,ff] = equationsToMatrix(eqns,ddq);

%%%% Generate an optimized matlab function for dynamics:
matlabFunction(MM(1,1),MM(1,2),MM(1,3),MM(2,1),MM(2,2),MM(2,3),MM(3,1),MM(3,2),MM(3,3),...
    ff(1),ff(2),ff(3),...
    'file','autoGen_3lwalker_dynamics.m',...
    'vars',{q1,q2,q3,dq1,dq2,dq3,u1,u2,Mb,Ml,Lb,Ll,d,I,g},...
    'outputs',{'M11','M12','M13','M21','M22','M23','M31','M32','M33',...
    'f1','f2','f3'});

%%%% Generate a function for computing the kinematics:
matlabFunction(p1,p2,p3,dp1,dp2,dp3,pHip,pElb,...
    'file','autoGen_kinematics.m',...
    'vars',{q1,q2,q3,dq1,dq2,dq3,Lb,Ll,d},...
    'outputs',{'p1','p2','p3','dp1','dp2','dp3','pHip','pElb'});

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%         Derive heel-strike map and collision mechanics                  %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
disp('*');
pFoot = pElb + Ll*e3; % swing foot position

% Angular momentum of the system about the new stance foot (old swing foot)
hSysBefore = ...
    cross2d(p1-pFoot,Ml*dp1) + I*dq1 + ...   % old stance leg
    cross2d(p2-pFoot,Mb*dp2) + I*dq2 + ...   % old body
    cross2d(p3-pFoot,Ml*dp3) + I*dq3;        % old swing leg

% Anglar momentum of the body about the elbow
hElbBefore = cross2d(p2-pElb,Mb*dp2) + I*dq2 + ...
             cross2d(p1-pElb,Ml*dp1) + I*dq1;

% Angular momentum of the old stance leg about the hip
hLegBefore = cross2d(p1-pHip,Ml*dp1) + I*dq1;  % old stance leg

% Introduce new variables for the state after the collision:
q1New = q3;
q2New = q2;
q3New = q1;
syms dq1New dq2New dq3New 'real'   % angular rates after collision

% Unit vectors after the collision:   (new naming convention)
e1New = cos(q1New)*(-j) + sin(q1New)*(i);    % hip -> stance foot
e2New = cos(q2New)*(-j) + sin(q2New)*(i);    % hip -> elbow
e3New = cos(q3New)*(-j) + sin(q3New)*(i);    % elbow -> swing foot

% Kinematics:
pHipNew = -Ll*e1New;
pElbNew = pHipNew - Lb*e2New;
p1New = pHipNew + d*e1New;
p2New = pHipNew + (Lb/2)*e2New;
p3New = pElbNew + d*e3New;

dp1New = jacobian(p1New,[q1New;q2New;q3New])*[dq1New;dq2New;dq3New];  
dp2New = jacobian(p2New,[q1New;q2New;q3New])*[dq1New;dq2New;dq3New];  
dp3New = jacobian(p3New,[q1New;q2New;q3New])*[dq1New;dq2New;dq3New];

% Angular momentum of the system after collision about new stance foot:
hSysAfter = cross2d(p3New,Ml*dp3New) + I*dq3New + ...
    cross2d(p2New,Mb*dp2New) + I*dq2New + ...
    cross2d(p1New,Ml*dp1New) + I*dq1New;

% Angular momentum of the system about elbow 
hHipAfter = cross2d(p2New-pHipNew,Mb*dp2New) + I*dq2New + ...
            cross2d(p3New-pHipNew,Ml*dp3New) + I*dq3New;
        
% Angular momentum of the new swing leg about the hip
hLegAfter = cross2d(p3New-pElbNew,Ml*dp3New) + I*dq3New;

% solve the dynamics
eqn1 = hSysBefore - hSysAfter;
eqn2 = hElbBefore - hHipAfter;
eqn3 = hLegBefore - hLegAfter;
eqnsHs = [eqn1;eqn2;eqn3];
varsHs = [dq1New; dq2New; dq3New];
[AA,bb] = equationsToMatrix(eqnsHs, varsHs);

% Write the heel-strike map to a file:
matlabFunction(AA(1,1),AA(1,2),AA(1,3),AA(2,1),AA(2,2),AA(2,3),AA(3,1),AA(3,2),AA(3,3),...
    bb(1),bb(2),bb(3),...
    'file','autoGen_heelStrike.m',...
    'vars',{q1,q2,q3,dq1,dq2,dq3,Mb,Ml,Lb,Ll,d,I,g},...
    'outputs',{'A11','A12','A13','A21','A22','A23','A31','A32','A33','b1','b2','b3'});
