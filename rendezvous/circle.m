n_p = 20; % number of points in graph

% create some random points
x = rand(n_p, 1);  % random x value
y = rand(n_p, 1);  % random y value 
xy = cat(2, x, y); % coordinates

% construct the set of source node and terminal node
e_s = [];
e_t = [];
% construct the network based on distance
for i = 1: n_p
    for j = i+1: n_p
        d_ij = sqrt((xy(i,1) - xy(j,1))^2 + (xy(i,2) - xy(j,2))^2);
        if d_ij < 0.4
            e_s = [e_s i];
            e_t = [e_t j];
        end
    end
end
% construct the distance set Xi
radius = 0.2;
ofs = [0.0 0.0];
Xi = zeros(n_p, 2);
for i = 1: n_p
    angle = (2*pi/n_p)*i;
    Xi(i, :) = radius * [cos(angle) sin(angle)] + ofs;
end

G = graph(e_s, e_t); % initialize graph
p = plot(G, 'XData', xy(:, 1), 'YData', xy(:, 2));
L = laplacian(G);
lap_eigs = eig(L);
disp(lap_eigs(2));
% updating the graph with agreement protocol
for i = 1:200
    d_xy = -(L * xy - L * Xi);
    xy = xy + d_xy*0.04;
    p = plot(G, 'XData', xy(:, 1), 'YData', xy(:, 2));
    xlim([0 1]);
    ylim([0 1]);
    pause(0.01);
end