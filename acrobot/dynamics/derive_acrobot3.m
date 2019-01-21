syms q1 q2 q3 dq1 dq2 dq3 ddq1 ddq2 ddq3 'real';
syms u1 u2 'real';
syms m1 m2 m3 g l1 l2 l3 'real';

j = sym([0; -1]);
i = sym([1; 0]);

e1 = cos(q1)*(-j) + sin(q1)*(i);
e2 = cos(q1+q2)*(-j) + sin(q1+q2)*(i);
e3 = cos(q1+q2+q3)*(-j) + sin(q1+q2+q3)*(i);

x = [q1; q2; q3; dq1; dq2; dq3];
dx = [dq1; dq2; dq3; ddq1; ddq2; ddq3];

p1 = l1*e1;
p2 = p1 + l2*e2;
p3 = p2 + l3*e3;

dp1 = jacobian(p1, x) * dx;
dp2 = jacobian(p2, x) * dx;
dp3 = jacobian(p3, x) * dx;

ddp1 = jacobian(dp1, x) * dx;
ddp2 = jacobian(dp2, x) * dx;
ddp3 = jacobian(dp3, x) * dx;

cross2d = @(a,b)(a(1)*b(2) - a(2)*b(1));

sumTorques1 = cross2d(p1,-m1*g*j) + cross2d(p2,-m2*g*j) + cross2d(p3,-m3*g*j);
sumInertial1 = cross2d(p1,m1*ddp1) + cross2d(p2,m2*ddp2) + cross2d(p3,m3*ddp3);
eqn1 = sumTorques1 - sumInertial1;

sumTorques2 = cross2d(p2-p1,-m2*g*j) + cross2d(p3-p1,-m3*g*j) + u1;
sumInertial2 = cross2d(p2-p1,m2*ddp2) + cross2d(p3-p1,m3*ddp3);
eqn2 = sumTorques2 - sumInertial2;

sumTorques3 = cross2d(p3-p2,-m3*g*j) + u2;
sumInertial3 = cross2d(p3-p2,m3*ddp3);
eqn3 = sumTorques3 - sumInertial3;

ddq = [ddq1; ddq2; ddq3];
eqns = [eqn1; eqn2; eqn3];
[MM, ff] = equationsToMatrix(eqns, ddq);

%%%% Generate an optimized matlab function for dynamics:
dynamics = matlabFunction(MM(1,1),MM(1,2),MM(1,3),MM(2,1),MM(2,2),MM(2,3),MM(3,1),MM(3,2),MM(3,3),ff(1),ff(2),ff(3),...
    'file','autoGen_acrobot3Dynamics.m',...
    'vars',{q1,q2,q3,dq1,dq2,dq3,u1,u2,m1,m2,m3,g,l1,l2,l3},...
    'outputs',{'M11','M12','M13','M21','M22','M23','M31','M32','M33','f1','f2','f3'});

%%%% Generate a function for computing the kinematics:
matlabFunction(p1,p2, p3,dp1,dp2, dp3,...
    'file','autoGen_acrobot3Kinematics.m',...
    'vars',{q1,q2,q3,dq1,dq2,dq3,l1,l2,l3},...
    'outputs',{'p1','p2','p3','dp1','dp2','dp3'});
