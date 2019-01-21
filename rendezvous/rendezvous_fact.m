n_p = 5;
dist_c = 0.4;
d_t = 0.04;
% create random point for each group
x1 = rand(n_p, 1) / 2;
y1 = rand(n_p, 1) / 2 + 0.25; 
x2 = rand(n_p, 1) / 2 + 0.5;
y2 = rand(n_p, 1) / 2 + 0.25;
xy1 = cat(2, x1, y1);
xy2 = cat(2, x2, y2);

for i = 1 : 40
   % build graph
   e_s1 = [];
   e_t1 = [];
   e_s2 = [];
   e_t2 = [];
   for j = 1 : n_p
       for k = j+1 : n_p
           d_ij1 = sqrt((xy1(j,1) - xy1(k,1))^2 + (xy1(j,2) - xy1(k,2))^2);
           d_ij2 = sqrt((xy2(j,1) - xy2(k,1))^2 + (xy2(j,2) - xy2(k,2))^2);
           if d_ij1 < dist_c
               e_s1 = [e_s1 j];
               e_t1 = [e_t1 k];
           end
           if d_ij2 < dist_c
               e_s2 = [e_s2 j];
               e_t2 = [e_t2 k];
           end
       end
   end
   G1 = graph(e_s1, e_t1);
   G2 = graph(e_s2, e_t2);
   e_s = [1];
   e_t = [2];
   mean1 = sum(xy1) / n_p;
   mean2 = sum(xy2) / n_p;
   G = graph(e_s, e_t);
   plot(G1, 'ro', 'XData', xy1(:, 1), 'YData', xy1(:, 2));
   hold on;
   plot(G2, 'bo', 'XData', xy2(:, 1), 'YData', xy2(:, 2));
   plot(G, 'go', 'XData', [mean1(1), mean2(1)], 'YData', [mean1(2), mean2(2)])
   xlim([0 1]);
   ylim([0 1]);
   pause(0.01);
   hold off;
   if length(e_s1) == length(e_s2)
       if (e_s1 == e_s2) | (e_t1 == e_t2)
           % two graph are the same
           L1 = laplacian(G1);
           L2 = [1 -1; -1 1];
           % compute kronecker sum of two laplacians
           L = kron(L1, eye(2)) + kron(eye(n_p), L2);
           % concat two states
           xy = cat(1, xy1, xy2);
           d_xy = -L * xy;
           xy = xy + d_xy * d_t;
           xy1 = xy(1:n_p, :);
           xy2 = xy(n_p+1: 2*n_p, :);    
           continue
       end
   end
   % two graphs not the same
   L1 = laplacian(G1);
   L2 = laplacian(G2);
   d_xy1 = -L1 * xy1;
   d_xy2 = -L2 * xy2;
   xy1 = xy1 + d_xy1 * d_t;
   xy2 = xy2 + d_xy2 * d_t;
   hold off;
end