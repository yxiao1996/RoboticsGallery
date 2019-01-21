function dx = acrobot3_dynamics(x, u, p)
    g = p.g;
    m1 = p.m1;
    m2 = p.m2;
    m3 = p.m3;
    L1 = p.L1;
    L2 = p.L2;
    L3 = p.L3;
    q1 = x(1, :);
    q2 = x(2, :);
    q3 = x(3, :);
    d_q1 = x(4, :);
    d_q2 = x(5, :);
    d_q3 = x(6, :);
    size_q = size(q1);
    n_g = size_q(2);
    
    M11 = (m1+m2+m3)*L1^2 + (m2+m3)*L2^2 + m3*L3^2 ...
        + 2*(m2+m3)*L1*L2*cos(q2) + 2*m3*L1*L3*cos(q2+q3) + 2*m3*L2*L3*cos(q3);
    M12 = (m2+m3)*L2^2 + m3*L3^2 + (m2+m3)*L1*L2*cos(q2) ...
        + m2*L1*L3*cos(q2+q3) + 2*m3*L2*L3*cos(q3);
    M13 = m3*L3^2 + m3*L1*L3*cos(q2+q3) + m3*L2*L3*cos(q3);
    M21 = (m2+m3)*L2^2 + m3*L3^2 + (m2+m3)*L1*L2*cos(q2) ...
        + m3*L1*L3*cos(q2+q3) + 2*m3*L2*L3*cos(q3);
    M22 = (m2+m3)*L2^2 + m3*L3^2 + 2*m3*L2*L3*cos(q3);
    M23 = m3*L3^2 + m3*L2*L3*cos(q3);
    M31 = m3*L3^2 + m3*L1*L3*cos(q2+q3) + m3*L2*L3*cos(q3);
    M32 = m3*L3^2 + m3*L2*L3*cos(q3);
    M33 = m3*L3^2 .* ones(1,n_g);
    C11 = zeros(1,n_g);
    C12 = -(m2+m3)*L1*L2*(2*d_q1+d_q2).*sin(q2) ...
        - m3*L1*L3*(2*d_q1+d_q2+d_q3).*sin(q2+q3);
    C13 = - m3*L1*L3*(2*d_q1+d_q2+d_q3).*sin(q2+q3) ...
        - m3*L2*L3*(2*d_q1+d_q2+d_q3).*sin(q3);
    C21 = -(m2+m3)*L1*L2*d_q2.*sin(q2) - m3*L1*L3*(d_q2+d_q3).*sin(q2+q3) ...
        + (m2+m3)*L1*L2*(d_q1+d_q2).*sin(q2) + m3*L1*L3*(d_q1+d_q2+d_q3).*sin(q3);
    C22 = m3*L1*L3*(d_q1+d_q2+d_q3).*sin(q3);
    C23 = -m3*L2*L3*(2*d_q1+2*d_q2+d_q3).*sin(q3);
    C31 = -m3*L1*L3*(d_q2+d_q3).*sin(q2+q3) + m3*L1*L3*(d_q1+d_q2+d_q3).*sin(q2+q3) ...
        + m3*L2*L3*(d_q1+d_q2+d_q3).*sin(q3);
    C32 = m3*L2*L3*(d_q1+d_q2+d_q3).*sin(q3);
    C33 = -m3*L2*L3*(d_q1+d_q2).*sin(q3);
    
    M = zeros(6, 6, n_g);
    M(1, 1, :) = ones(1, n_g);
    M(2, 2, :) = ones(1, n_g);
    M(3, 3, :) = ones(1, n_g);
    M(4, 1, :) = C11;
    M(4, 2, :) = C12;
    M(4, 3, :) = C13;
    M(4, 4, :) = M11;
    M(4, 5, :) = M12;
    M(4, 6, :) = M13;
    M(5, 1, :) = C21;
    M(5, 2, :) = C22;
    M(5, 3, :) = C23;
    M(5, 4, :) = M21;
    M(5, 5, :) = M22;
    M(5, 6, :) = M23;
    M(6, 1, :) = C31;
    M(6, 2, :) = C32;
    M(6, 3, :) = C33;
    M(6, 4, :) = M31;
    M(6, 5, :) = M32;
    M(6, 6, :) = M33;
    
    f1 = d_q1;
    f2 = d_q2;
    f3 = d_q3;
    f4 = -g * ((m1+m2+m3)*L1*sin(q1)+(m2+m3)*L2*sin(q1+q2)+m3*L3*sin(q1+q2+q3));
    f5 = -g * ((m2+m3)*L2*sin(q1+q2) + m3*L3*sin(q1+q2+q3));
    f6 = -g * m3*L3*sin(q1+q2+q3) + u(3, :);
    d_q = [f1; f2; f3; f4; f5; f6];
    
    dx = zeros(6, n_g);
    for i = 1: n_g
        dx(:, i) = M(:, :, i) \ d_q(:, i);
    end
end

