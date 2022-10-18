clc
close all
clear

%% Parameters
waypts = [0,0,0;
    1,2,1;
    2,-1,1;
    4,8,1;
    5,2,0]';

% waypts = [ -2 4;
%     -1 0.5;
%     0 0;
%     1 1;]';

% order of polynomial
n_order = 5;

% total duration
T = 4;

% segment number
k= length(waypts)-1;
[total_dist, segment_length] = calDistance(waypts, k);
% time allocation for each segment
segment_length = segment_length/total_dist*T;
t_alloc = allocate_time(waypts,T);
t_alloc = 10*ones(1, k);
Q_total = calQ(t_alloc);
% t_alloc = [0 0.2 0.4 0.8 1.2];

% x_coeff is the list of polynomial coefficients for k segments, size: k*(n+1)
x_coeff = min_snap_sigle_axis_monomial(n_order, k, t_alloc, Q_total, waypts(1,:));
y_coeff = min_snap_sigle_axis_monomial(n_order, k, t_alloc, Q_total, waypts(2,:));
z_coeff = min_snap_sigle_axis_monomial(n_order, k, t_alloc, Q_total, waypts(3,:));

%% plot result
[f1, x, vx, ax, tx] = plotTrajectory1axis(n_order, k, t_alloc, x_coeff, waypts(1,:), 1, 'x axis');
movegui(f1, 'northwest');
[f2, y, vy, ay, ty] = plotTrajectory1axis(n_order, k, t_alloc, y_coeff, waypts(2,:), 2, 'y axis');
movegui(f2, 'north');
[f3, z, vz, az, tz] = plotTrajectory1axis(n_order, k, t_alloc, z_coeff, waypts(3,:), 3, 'z axis');
movegui(f3, 'northeast');

f4 = plotTrajectory2axis(x,y,waypts,4);
movegui(f4, 'southeast');

f5 = plotTrajectory3axis(x,y,z,waypts,5);
movegui(f5, 'southwest');
% figure(1)
% hold on;
% plot(waypts(1,:),waypts(2,:),'*r');
% plot(waypts(1,:),waypts(2,:),'b--');
% title('minimum snap trajectory monomial');
% color = ['grcb'];
% px=[];
% py=[];
% for i=1:k
%     t_sequence = t_alloc(i):0.01:t_alloc(i+1); %get the time tick for current segment
%     for j=1:length(t_sequence) % for each time tick within the segment, evaluate x and y position
%         px = [px poly_evaluate(0,t_sequence(j),n_order)*x_coeff(:,i)];
%         py = [py poly_evaluate(0,t_sequence(j),n_order)*y_coeff(:,i)];
%         plot(px,py,color(i));
%     end
%     px = [];
%     py = [];
% end


%% Bezier curve
% each n order polynomial segments have n+1 control points
% in total (n+1)*k optimizing variables
% t_alloc_bernstein = zeros(length(waypts),1);
% for i=1:length(waypts)
%     t_alloc_bernstein(i)=i-1;
% end
% 
% 
% Q_total_bern = calQ(t_alloc_bernstein);
% cx=min_snap_sigle_axis_bezier(n_order, k, t_alloc_bernstein, Q_total_bern, waypts(1,:));
% cy=min_snap_sigle_axis_bezier(n_order, k, t_alloc_bernstein, Q_total_bern, waypts(2,:));
% 
% M = getMappingMatrix(n_order)';
% 
% % plot result
% figure(2)
% hold on;
% plot(waypts(1,:),waypts(2,:),'*r');
% plot(waypts(1,:),waypts(2,:),'b--');
% title('minimum snap trajectory bezier');
% color = ['grcb'];
% px=[];
% py=[];
% for i=1:k
%     t_sequence = t_alloc_bernstein(i):0.01:t_alloc_bernstein(i+1); %get the time tick for current segment
%     for j=1:length(t_sequence) % for each time tick within the segment, evaluate x and y position
%         px = [px poly_evaluate(0,t_sequence(j),n_order)*M*cx(:,i)];
%         py = [py poly_evaluate(0,t_sequence(j),n_order)*M*cy(:,i)];
%         plot(px,py,color(i));
%     end
%     px = [];
%     py = [];
% end
% 
% for i=1:k
%     t_sequence = t_alloc_bernstein(i):0.01:t_alloc_bernstein(i+1);
%     cx_ = cx(:,i);
%     cy_ = cy(:,i);
%     for j=1:length(t_sequence)
%         px = [px evaluateB(cx_',t_sequence(j),n_order)];
%         py = [py evaluateB(cy_',t_sequence(j),n_order)];
%         plot(px,py,color(i));
%     end
%     px=[];
%     py=[];
% end

%plot(cx(:,1),cy(:,2),'*r');