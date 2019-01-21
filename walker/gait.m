function [c,ceq] = gait(xF,x0,p)
    c = [];
    q1 = xF(1,:);
    q2 = xF(2,:);
    q3 = xF(3,:);
    dq1 = xF(4,:);
    dq2 = xF(5,:);
    dq3 = xF(6,:);
    Mb = p.Mb;
    Ml = p.Ml;
    Lb = p.Lb;
    Ll = p.Ll;
    d = p.d;
    I = p.I;
    g = p.g;
    [A11,A12,A13,A21,A22,A23,A31,A32,A33,b1,b2,b3] = ...
        autoGen_heelStrike(q1,q2,q3,dq1,dq2,dq3,Mb,Ml,Lb,Ll,d,I,g);
    nTime = length(q1);
    xNew = zeros(6, nTime);
    for i = 1 : nTime
        AA = [
            A11(i) A12(i) A13(i);
            A21(i) A22(i) A23(i);
            A31(i) A32(i) A33(i)
        ];
        bb = [b1(i);b2(i);b3(i)];
        dqNew = AA \ bb;
        xNew(1,i) = q3(i);
        xNew(2,i) = q2(i);
        xNew(3,i) = q1(i);
        xNew(4,i) = dqNew(1);
        xNew(5,i) = dqNew(2);
        xNew(6,i) = dqNew(3);
    end
    ceq = x0 - xNew;
end

