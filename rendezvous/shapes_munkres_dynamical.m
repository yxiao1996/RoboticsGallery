n_p = 16; % number of points in graph
dist_c = 0.4;
interval = 20;
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
        if d_ij < dist_c
            e_s = [e_s i];
            e_t = [e_t j];
        end
    end
end
% construct the distance set Xi
radius1 = 0.3;
radius2 = 0.2;
ofs = [0.0 0.0];
Xi = zeros(n_p, 2, 4);
for i = 1: n_p
    angle = (2*pi/n_p)*i;
    Xi(i, :, 1) = radius1 * [cos(angle) sin(angle)] + ofs;
    Xi(i, :, 4) = radius2 * [cos(angle) sin(angle)] + ofs;
    Xi(i, :, 2) = [cos(angle/4) sin(angle/4)]*radius1;
    Xi(i, :, 3) = [i/(2*n_p) i/(2*n_p)];
    if mod(i, 4) == 0
        Xi(i, :, 5) = [0 (i/4)/(2*n_p)];
    elseif mod(i, 4) == 1
        Xi(i, :, 5) = [((i-1)/4)/(2*n_p) 0];
    elseif mod(i, 4) == 2
        Xi(i, :, 5) = [ceil(n_p/4)/(n_p*2) ((i-2)/4)/(2*n_p)];
    else
        Xi(i, :, 5) = [((i-3)/4)/(2*n_p) ceil(n_p/4)/(n_p*2)];
    end
end

G = graph(e_s, e_t); % initialize graph
p = plot(G, 'XData', xy(:, 1), 'YData', xy(:, 2));
L = laplacian(G);
lap_eigs = eig(L);
disp(lap_eigs(2));
% updating the graph with agreement protocol
for i = 1:400
    shape_idx = mod(uint8(floor(i/interval)), 5) + 1;
    if mod(i, interval) == 1
        % assign role using hungarian algorithm
        disp(n_p);
        cost = zeros(n_p);
        for j = 1:n_p  % ith robot
            for k = 1:n_p  % jth target position
                cost(j, k) = sqrt((xy(j, 1) - Xi(k, 1, shape_idx))^2 + (xy(j, 2) - Xi(k, 2, shape_idx))^2);
            end
        end
        [assign, c_opt] = munkres(cost);
        Xi_opt = Xi(assign, :, :);
    end
    d_xy = -(L * xy - L * Xi_opt(:, :, shape_idx));
    xy = xy + d_xy*0.02;
    p = plot(G, 'XData', xy(:, 1), 'YData', xy(:, 2));
    xlim([0 1]);
    ylim([0 1]);
    pause(0.005);
    e_s = [];
    e_t = [];
    for j = 1: n_p
        for k = j+1: n_p
            d_ij = sqrt((xy(j,1) - xy(k,1))^2 + (xy(j,2) - xy(k,2))^2);
            if d_ij < dist_c
                e_s = [e_s j];
                e_t = [e_t k];
            end
        end
    end
    G = graph(e_s, e_t); % initialize graph
    L = laplacian(G);
    lap_eigs = eig(L);
    disp(lap_eigs(2));
end