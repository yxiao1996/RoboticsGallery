g = 9.81;
m1 = 0.1;
m2 = 0.1;
L1 = 0.5;
L2 = 1;

t = soln.grid.time;
q1 = soln.grid.state(1,:);
q2 = soln.grid.state(2,:);
u = soln.grid.control;

theta1 = q1(1);
theta2 = q2(1);
yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2)];
xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2)];
plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo');
axis([0 4 -2 2])
for j = 2:length(t)
    if rand < 0.1
        input('enter');
    end
    theta1 = q1(j);
    theta2 = q2(j);
    yvals = -[0 L1*cos(theta1) L1*cos(theta1)+L2*cos(theta1+theta2)];
    xvals = [0 L1*sin(theta1) L1*sin(theta1)+L2*sin(theta1+theta2)];
    plot(xvals,yvals,xvals(1),yvals(1),'ro',xvals(2),yvals(2),'go', xvals(3), yvals(3), 'bo');
    axis([-2 2 -2 2])
    pause(0.2);
end