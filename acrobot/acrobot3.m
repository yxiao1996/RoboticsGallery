function acrobot
% q(1) = theta1 q(2) = theta2 q(3) = theta1' q(4) = theta2'

m1 = 0.1;
m2 = 0.1;
m3 = 0.1;
L1 = 0.5;
L2 = 0.5;
L3 = 0.5;
g = 9.81;

tspan = linspace(0, 4.00);
q0 = [pi/3, 0, 0, 0, 0, 0]; % upright state
options = odeset('Mass', @mass);

[t, y] = ode45(@f, tspan, q0, options);

theta1 = y(1, 1);
theta2 = y(1, 2);
theta3 = y(1, 3);
yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2) L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)];
xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2) L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)];
plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo', xvals(4), yvals(4), 'yo');
axis([-2 2 -2 2])
input("enter");
for j = 2:length(t)
    if rand < 0.1
        input('enter');
    end
    theta1 = y(j, 1);
    theta2 = y(j, 2);
    theta3 = y(j, 3);
    yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2) L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)];
    xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2) L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)];
    plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo', xvals(4), yvals(4), 'yo');
    axis([-2 2 -2 2])
    pause(0.1);
end

  function d_q = f(t, q)
    q1 = q(1); 
    q2 = q(2);
    q3 = q(3);
    d_q1 = q(4);
    d_q2 = q(5);
    d_q3 = q(6);
    f1 = d_q1;
    f2 = d_q2;
    f3 = d_q3;
    f4 = -g * ((m1+m2+m3)*L1*sin(q1)+(m2+m3)*L2*sin(q1+q2)+m3*L3*sin(q1+q2+q3));
    f5 = -g * ((m2+m3)*L2*sin(q1+q2) + m3*L3*sin(q1+q2+q3));
    f6 = -g * m3*L3*sin(q1+q2+q3);
    d_q = [f1; f2; f3; f4; f5; f6];
  end

  function M = mass(t, q)
    q1 = q(1);
    q2 = q(2);
    q3 = q(3);
    d_q1 = q(4);
    d_q2 = q(5);
    d_q3 = q(6);
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
    M33 = m3*L3^2;
    C11 = 0;
    C12 = -(m2+m3)*L1*L2*(2*d_q1+d_q2)*sin(q2) ...
        - m3*L1*L3*(2*d_q1+d_q2+d_q3)*sin(q2+q3);
    C13 = - m3*L1*L3*(2*d_q1+d_q2+d_q3)*sin(q2+q3) ...
        - m3*L2*L3*(2*d_q1+d_q2+d_q3)*sin(q3);
    C21 = -(m2+m3)*L1*L2*d_q2*sin(q2) - m3*L1*L3*(d_q2+d_q3)*sin(q2+q3) ...
        + (m2+m3)*L1*L2*(d_q1+d_q2)*sin(q2) + m3*L1*L3*(d_q1+d_q2+d_q3)*sin(q3);
    C22 = m3*L1*L3*(d_q1+d_q2+d_q3)*sin(q3);
    C23 = -m3*L2*L3*(2*d_q1+2*d_q2+d_q3)*sin(q3);
    C31 = -m3*L1*L3*(d_q2+d_q3)*sin(q2+q3) + m3*L1*L3*(d_q1+d_q2+d_q3)*sin(q2+q3) ...
        + m3*L2*L3*(d_q1+d_q2+d_q3)*sin(q3);
    C32 = m3*L2*L3*(d_q1+d_q2+d_q3)*sin(q3);
    C33 = -m3*L2*L3*(d_q1+d_q2)*sin(q3);
    
    M = [1 0 0 0 0 0;
         0 1 0 0 0 0;
         0 0 1 0 0 0;
         C11 C12 C13 M11 M12 M13;
         C21 C22 C23 M21 M22 M23;
         C31 C32 C33 M31 M32 M33];
     
  end
end