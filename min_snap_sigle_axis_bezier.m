function c=min_snap_sigle_axis_bezier(n_order, k, t_alloc, Q_total, waypts)
po = waypts(1);
pf = waypts(k+1);
%% Equality constraints of the form Aeq * q = beq
% order n polynomial, k segments
% q is k*(n+1) column vector since there are k segments with n+1
% coefficients each
% A has      3       +         (k-1)           +     3        +           3 * (k-1)                 = 4k + 2 rows
%     (intial p,v,a)  (intermediate waypoints)  (final p,v,a)  (continuous p,v,a between segments)    
% b = [p0 v0 a0            p1 ... p_(k-1)         pk vk ak            0       ...      0]
%                          |---(k-1)----|                             |-----3(k-1)-----|
A_eq = zeros((4*k+2),(n_order+1)*k);
b_eq = zeros((4*k+2),1);

A = poly_evaluate(1,t_alloc(1),n_order);

% initial p,v,a equality constraints (1,2,3)
for i=1:3
    A_eq(i,1:n_order+1) = poly_evaluate(i-1,t_alloc(1),n_order);
end

b_eq(1) = po; % intial p
b_eq(2) = 0; % intial v
b_eq(3) = 0; % intial a

% final p,v,a equality constraints (4,5,6)
for i=4:6
    A_eq(i,(k-1)*(n_order+1)+1:(n_order+1)*k) = poly_evaluate(i-3-1,t_alloc(k+1),n_order);
end

b_eq(4) = pf; % final p
b_eq(5) = 0; % final v
b_eq(6) = 0; % final a

% intermediate waypoints (7,8,9) total k segments, k+1 waypoints, k-1 points in
% between
row_no=7;
for i=1:k-1
    A_eq(row_no, i*(n_order+1)+1:i*(n_order+1)+1+n_order) = poly_evaluate(0,t_alloc(i+1),n_order);
    b_eq(row_no) = waypts(i+1);
    row_no = row_no+1;
end
% for i=7:7+k-2
%     A_eq(i,(i-6)*(n_order+1)+1:(i-6)*(n_order+1)+1+n_order) = poly_evaluate(0,t_alloc(i-5),n_order);
% end

% continuous constraints
for i=1:k-1
    A_eq(row_no,(i-1)*(n_order+1)+1:(i+1)*(n_order+1)) = [poly_evaluate(0,t_alloc(i+1),n_order) -1*poly_evaluate(0,t_alloc(i+1),n_order)];
    row_no = row_no+1;
    A_eq(row_no,(i-1)*(n_order+1)+1:(i+1)*(n_order+1)) = [poly_evaluate(1,t_alloc(i+1),n_order) -1*poly_evaluate(1,t_alloc(i+1),n_order)];
    row_no = row_no+1;
    A_eq(row_no,(i-1)*(n_order+1)+1:(i+1)*(n_order+1)) = [poly_evaluate(2,t_alloc(i+1),n_order) -1*poly_evaluate(2,t_alloc(i+1),n_order)];
    row_no = row_no+1;
end

%% Convert from control points to polynomial coefficients
M=getMappingMatrix(n_order)';
M_all = [];
for i=1:k
    M_all = blkdiag(M_all,M);
end

A_eq = A_eq*M_all;
Q_total = M_all'*Q_total*M_all;
Q_total = nearestSPD(Q_total);

%% Inequality Constraints
A_ineq = zeros((4*k+2),(n_order+1)*k);
b_ineq = zeros((4*k+2),1);

%% cost funtion linear term
f = zeros((n_order+1)*k,1);

%% solve QP
c = quadprog(Q_total, f, A_ineq, b_ineq, A_eq, b_eq);
c=reshape(c,[n_order+1,k]);

end