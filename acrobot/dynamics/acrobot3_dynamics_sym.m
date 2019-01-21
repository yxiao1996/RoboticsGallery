function dx = acrobot3_dynamics_sym(x,u,p)
    g = p.g;
    m1 = p.m1;
    m2 = p.m2;
    m3 = p.m3;
    l1 = p.L1;
    l2 = p.L2;
    l3 = p.L3;
    q1 = x(1, :);
    q2 = x(2, :);
    q3 = x(3, :);
    dq1 = x(4, :);
    dq2 = x(5, :);
    dq3 = x(6, :);
    u1 = u(2,:);
    u2 = u(3,:);
    [M11,M12,M13,M21,M22,M23,M31,M32,M33,f1,f2,f3] ...
        = autoGen_acrobot3Dynamics(q1,q2,q3,dq1,dq2,dq3,u1,u2,m1,m2,m3,g,l1,l2,l3);
    nTime = length(q1);
    ddq = zeros(3, nTime);
    for i = 1: nTime
        MM = [M11(i) M12(i) M13(i);
              M21(i) M22(i) M23(i);
              M31(i) M32(i) M33(i)];
        ff = [f1(i);f2(i);f3(i)];
        ddq(:,i) = MM \ ff;
    end
    dx = [dq1;dq2;dq3;ddq];
end

