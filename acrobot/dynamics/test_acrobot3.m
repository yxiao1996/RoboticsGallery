function test_acrobot3
m1 = 0.1;
m2 = 0.1;
m3 = 0.1;
L1 = 0.5;
L2 = 0.5;
L3 = 0.5;
g = 9.81;
tspan = linspace(0, 4.00);
q0 = [pi/6, 0, 0, 0, 0, 0]; % upright state
[t,y] = ode45(@f, tspan, q0);
theta1 = y(1, 1);
theta2 = y(1, 2);
theta3 = y(1, 3);
dtheta1 = y(1, 4);
dtheta2 = y(1, 5);
dtheta3 = y(1, 6);
[p1,p2,p3,dp1,dp2,dp3] = ...
    autoGen_acrobot3Kinematics(theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,L1,L2,L3);
%yvals = -[0 p1(1) p2(1) p3(1)];
%xvals = [0 p1(2) p2(2) p3(2)];
yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2) L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)];
xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2) L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)];
plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo', xvals(4), yvals(4), 'yo');
axis([-2 2 -2 2])
for j = 2:length(t)
    %if rand < 0.1
    %    input('enter');
    %end
    theta1 = y(j, 1);
    theta2 = y(j, 2);
    theta3 = y(j, 3);
    dtheta1 = y(j, 4);
    dtheta2 = y(j, 5);
    dtheta3 = y(j, 6);
    [p1,p2,p3,dp1,dp2,dp3] = ...
        autoGen_acrobot3Kinematics(theta1,theta2,theta3,dtheta1,dtheta2,dtheta3,L1,L2,L3);
    %yvals = -[0 p1(1) p2(1) p3(1)];
    %xvals = [0 p1(2) p2(2) p3(2)];
    yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2) L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)];
    xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2) L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)];
    plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo', xvals(4), yvals(4), 'yo');
    axis([-2 2 -2 2])
    pause(0.1);
end

    function dq = f(t, q)
        q1 = q(1); 
        q2 = q(2);
        q3 = q(3);
        dq1 = q(4);
        dq2 = q(5);
        dq3 = q(6);
        u1 = 0;
        u2 = 0;
        [M11,M12,M13,M21,M22,M23,M31,M32,M33,f1,f2,f3] = ...
            autoGen_acrobot3Dynamics(q1,q2,q3,dq1,dq2,dq3,u1,u2,m1,m2,m3,g,L1,L2,L3);
        MM = [
            M11 M12 M13;
            M21 M22 M23;
            M31 M32 M33
            ];
        ff = [f1;f2;f3];
        ddq = MM \ ff;
        dq = [dq1;dq2;dq3;ddq(1);ddq(2);ddq(3)];
    end
end