function dx = acrobot2_dynamics(x, u, p)

    g = p.g;
    m1 = p.m1;
    m2 = p.m2;
    L1 = p.L1;
    L2 = p.L2;
    q1 = x(1, :);
    q2 = x(2, :);
    d_q1 = x(3, :);
    d_q2 = x(4, :);
    size_q = size(q1);
    n_g = size_q(2);
    M11 = (m1 + m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q2);
    M12 = m2*L2^2 + m2*L1*L2*cos(q2);
    M21 = m2*L2^2 + m2*L1*L1*cos(q2);
    M22 = m2*L2^2 .* ones(1,n_g);
    C11 = zeros(1,n_g);
    C12 = -m2*L1*L2*(2*d_q1+d_q2).*sin(q2);
    C21 = m2*L1*L2*d_q1.*sin(q2);
    C22 = zeros(1,n_g);
    
    M = zeros(4, 4, n_g);
    %M = [ones(1,n_g)   zeros(1,n_g)   zeros(1,n_g)   zeros(1,n_g);
    %     zeros(1,n_g)  ones(1,n_g)    zeros(1,n_g)   zeros(1,n_g);
    %     C11 C12 M11 M12
    %     C21 C22 M21 M22];
    M(1, 1, :) = ones(1, n_g);
    M(2, 2, :) = ones(1, n_g);
    M(3, 1, :) = C11;
    M(3, 2, :) = C12;
    M(3, 3, :) = M11;
    M(3, 4, :) = M12;
    M(4, 1, :) = C21;
    M(4, 2, :) = C22;
    M(4, 3, :) = M21;
    M(4, 4, :) = M22;
    %disp(size(M));
    f1 = d_q1;
    f2 = d_q2;
    f3 = -g * ((m1+m2)*L1*sin(q1) + m2*L2*sin(q1+q2));
    f4 = -g * (m2*L2*sin(q1+q2)) + u(2, :);
    d_q = [f1; f2; f3; f4];
    dx = zeros(4, n_g);
    %disp(size(d_q));
    for i = 1: n_g
        dx(:, i) = M(:, :, i) \  d_q(:, i);
    end
end

