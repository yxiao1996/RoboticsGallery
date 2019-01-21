function acrobot
% q(1) = theta1 q(2) = theta2 q(3) = theta1' q(4) = theta2'

m1 = 0.1;
m2 = 0.1;
L1 = 1;
L2 = 1;
g = 9.81;

tspan = linspace(0, 10.00);
q0 = [pi/3, 0, 0, 0]; % upright state
options = odeset('Mass', @mass);

[t, y] = ode45(@f, tspan, q0, options);

theta1 = y(1, 1);
theta2 = y(1, 2);
yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2)];
xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2)];
plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo');
axis([-2 2 -2 2])
for j = 2:length(t)
    if rand < 0.1
        input('enter');
    end
    theta1 = y(j, 1);
    theta2 = y(j, 2);
    yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2)];
    xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2)];
    plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo');
    axis([-2 2 -2 2])
    pause(0.1);
end

  function d_q = f(t, q)
    q1 = q(1); 
    q2 = q(2);
    d_q1 = q(3);
    d_q2 = q(4);
    f1 = d_q1;
    f2 = d_q2;
    f3 = -g * ((m1+m2)*L1*sin(q1) + m2*L2*sin(q1+q2));
    f4 = -g * (m2*L2*sin(q1+q2));
    d_q = [f1; f2; f3; f4];
  end 

  function M = mass(t, q)
    q1 = q(1);
    q2 = q(2);
    d_q1 = q(3);
    d_q2 = q(4);
    M11 = (m1 + m2)*L1^2 + m2*L2^2 + 2*m2*L1*L2*cos(q2);
    M12 = m2*L2^2 + m2*L1*L2*cos(q2);
    M21 = m2*L2^2 + m2*L1*L1*cos(q2);
    M22 = m2*L2^2;
    C11 = 0;
    C12 = -m2*L1*L2*(2*d_q1+d_q2)*sin(q2);
    C21 = m2*L1*L2*d_q1*sin(q2);
    C22 = 0;
    M = [1 0 0 0;
         0 1 0 0;
         C11 C12 M11 M12
         C21 C22 M21 M22];
  end
end