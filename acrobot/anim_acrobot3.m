g = 9.81;
m1 = 0.1;
m2 = 0.1;
m3 = 0.1;
L1 = 0.4;
L2 = 0.4;
L3 = 0.8;

t = soln(3).grid.time;
q1 = soln(3).grid.state(1,:);
q2 = soln(3).grid.state(2,:);
q3 = soln(3).grid.state(3,:);
u = soln(3).grid.control;

theta1 = q1(1);
theta2 = q2(1);
theta3 = q3(1);
yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2) L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)];
    xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2) L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)];
    plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo', xvals(4), yvals(4), 'yo');
axis([-2 2 -2 2])
for j = 2:length(t)
    %if rand < 0.1
    %    input('enter');
    %end
    theta1 = q1(j);
    theta2 = q2(j);
    theta3 = q3(j);
    yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2) L1*cos(theta1)+L2*cos(theta1+theta2)+L3*cos(theta1+theta2+theta3)];
    xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2) L1*sin(theta1)+L2*sin(theta1+theta2)+L3*sin(theta1+theta2+theta3)];
    plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo', xvals(4), yvals(4), 'yo');
    axis([-2 2 -2 2])
    pause(0.2);
end