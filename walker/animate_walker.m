Lb = 1;  %leg length
Ll = 1;
d = 0.5;  %Leg CoM distance from hip

m = [1;0];
n = [0;1];
x_ofs = 0;
for i = 1:10: length(tInt)
    q1 = q1Int(i);
    q2 = q2Int(i);
    q3 = q3Int(i);
    dq1 = dq1Int(i);
    dq2 = dq2Int(i);
    dq3 = dq3Int(i);
    e1 = cos(q1)*(-n) + sin(q1)*(m);    % hip -> stance foot
    e2 = cos(q2)*(-n) + sin(q2)*(m);    % hip -> swing foot
    e3 = cos(q3)*(-n) + sin(q3)*(m);
    [p1,p2,p3,dp1,dp2,dp3,ph,pe] = ...
        autoGen_kinematics(q1,q2,q3,dq1,dq2,dq3,Lb,Ll,d);
    pf2 = pe + e3*Ll;
    xvals = [0 ph(1) pe(1) pf2(1)]+x_ofs;
    yvals = [0 ph(2) pe(2) pf2(2)];
    %yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2) L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)];
    %xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2) L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)];
    plot([-3 3],[0 0]);
    hold on;
    plot(xvals,yvals,p1(1)+x_ofs,p1(2),'ro',p2(1)+x_ofs,p2(2),'go', p3(1)+x_ofs, p3(2), 'bo');
    axis([-1 3 -1 3])
    hold off;
    pause(0.001);
end
x_ofs = x_ofs+pf2(1);
for i = 1:10: length(tInt_)
    q1 = q1Int_(i);
    q2 = q2Int_(i);
    q3 = q3Int_(i);
    dq1 = dq1Int_(i);
    dq2 = dq2Int_(i);
    dq3 = dq3Int_(i);
    e1 = cos(q1)*(-n) + sin(q1)*(m);    % hip -> stance foot
    e2 = cos(q2)*(-n) + sin(q2)*(m);    % hip -> swing foot
    e3 = cos(q3)*(-n) + sin(q3)*(m);
    [p1,p2,p3,dp1,dp2,dp3,ph,pe] = ...
        autoGen_kinematics(q1,q2,q3,dq1,dq2,dq3,Lb,Ll,d);
    pf2 = pe + e3*Ll;
    xvals = [0 ph(1) pe(1) pf2(1)]+x_ofs;
    yvals = [0 ph(2) pe(2) pf2(2)];
    %yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2) L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)];
    %xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2) L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)];
    plot([-3 3],[0 0]);
    hold on;
    plot(xvals,yvals,p1(1)+x_ofs,p1(2),'ro',p2(1)+x_ofs,p2(2),'go', p3(1)+x_ofs, p3(2), 'bo');
    axis([-1 3 -1 3])
    hold off;
    pause(0.001);
end
