function dx = dynamics(x,u,p)
    q1 = x(1,:);
    q2 = x(2,:);
    q3 = x(3,:);
    dq1 = x(4,:);
    dq2 = x(5,:);
    dq3 = x(6,:);
    u1 = u(1,:);
    u2 = u(2,:);
    Mb = p.Mb;
    Ml = p.Ml;
    Lb = p.Lb;
    Ll = p.Ll;
    d = p.d;
    I = p.I;
    g = p.g;
    [M11,M12,M13,M21,M22,M23,M31,M32,M33,f1,f2,f3] = ...
        autoGen_3lwalker_dynamics...
        (q1,q2,q3,dq1,dq2,dq3,u1,u2,Mb,Ml,Lb,Ll,d,I,g);
    nTime = length(q1);
    ddq = zeros(3, nTime);
    for i = 1 : nTime
        MM = [
            M11(i) M12(i) M13(i);
            M21(i) M22(i) M23(i);
            M31(i) M32(i) M33(i)
        ];
        ff = [
            f1(i);f2(i);f3(i)    
        ];
        ddq(:,i) = MM \ ff;
    end
    dx = [dq1;dq2;dq3;ddq(1,:);ddq(2,:);ddq(3,:)];
end

