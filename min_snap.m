clc
close all
clear

%% Parameters
waypts = [0,0;
    1,2;
    2,-1;
    4,8;
    5,2]';

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

% time allocation for each segment
t_alloc = allocate_time(waypts,T);
% t_alloc = zeros(length(waypts),1);
% for i=1:length(waypts)
%     t_alloc(i)=i-1;
% end

Q_total = calQ(t_alloc);

% x_coeff is the list of polynomial coefficients for k segments, size: k*(n+1)
x_coeff = min_snap_sigle_axis_monomial(n_order, k, t_alloc, Q_total, waypts(1,:));
y_coeff = min_snap_sigle_axis_monomial(n_order, k, t_alloc, Q_total, waypts(2,:));

%% plot result
figure(1)
hold on;
plot(waypts(1,:),waypts(2,:),'*r');
plot(waypts(1,:),waypts(2,:),'b--');
title('minimum snap trajectory monomial');
color = ['grcb'];
px=[];
py=[];
for i=1:k
    t_sequence = t_alloc(i):0.01:t_alloc(i+1); %get the time tick for current segment
    for j=1:length(t_sequence) % for each time tick within the segment, evaluate x and y position
        px = [px poly_evaluate(0,t_sequence(j),n_order)*x_coeff(:,i)];
        py = [py poly_evaluate(0,t_sequence(j),n_order)*y_coeff(:,i)];
        plot(px,py,color(i));
    end
    px = [];
    py = [];
end


%% Bezier curve
% each n order polynomial segments have n+1 control points
% in total (n+1)*k optimizing variables
t_alloc_bernstein = zeros(length(waypts),1);
for i=1:length(waypts)
    t_alloc_bernstein(i)=i-1;
end

Q_total_bern = calQ(t_alloc_bernstein);
cx=min_snap_sigle_axis_bezier(n_order, k, t_alloc_bernstein, Q_total_bern, waypts(1,:));
cy=min_snap_sigle_axis_bezier(n_order, k, t_alloc_bernstein, Q_total_bern, waypts(2,:));

M = getMappingMatrix(n_order)';

%% plot result
figure(2)
hold on;
plot(waypts(1,:),waypts(2,:),'*r');
plot(waypts(1,:),waypts(2,:),'b--');
title('minimum snap trajectory bezier');
color = ['grcb'];
px=[];
py=[];
for i=1:k
    t_sequence = t_alloc_bernstein(i):0.01:t_alloc_bernstein(i+1); %get the time tick for current segment
    for j=1:length(t_sequence) % for each time tick within the segment, evaluate x and y position
        px = [px poly_evaluate(0,t_sequence(j),n_order)*M*cx(:,i)];
        py = [py poly_evaluate(0,t_sequence(j),n_order)*M*cy(:,i)];
        plot(px,py,color(i));
    end
    px = [];
    py = [];
end