clc;clear;

N = 5;

x_r = [0.9; 0.5; 0.3; 0.8];
u_r = 0.059;
det_t = 1;

xk = [0.1; 0.1; 0.1; 0.1];
xr = [x_r;x_r;x_r;x_r;x_r];
x_k = [15 0.0393 0 0.393];
ur = u_r*ones([N,1]);
l = [0.5; 0.2; 0; 0; 0.1];
lambda = [l; l; l; l; l];
%% State Space Matrices

A = [-1.9311*10^-2 8.8157 -32.17 -0.575; -2.5389*10^-4 -1.0189 0 0.9051; 2.9465*10^-12 0.8223 0 -1.0774; 0 0 1 0];
B = [0.1737; -2.1499; -0.1756; 0];
C = [(sin(x_k(4))-(tan(x_k(2))*cos(x_k(4)))) -x_k(1)*cos(x_k(4))*(sec(x_k(2)))^2 0 x_k(1)*(cos(x_k(4))+(tan(x_k(2))*sin(x_k(4))))]*det_t;
D = 0;

%% Constraints

fx = [1/1000 0 0 0; 0 1/0.3491 0 0; 0 0 1/0.3491 0; 0 0 0 1/0.2618];
Fx = [fx, zeros([4,4]), zeros([4,4]), zeros([4,4]), zeros([4,4]); zeros([4,4]), fx, zeros([4,4]), zeros([4,4]), zeros([4,4]); zeros([4,4]), zeros([4,4]), fx, zeros([4,4]), zeros([4,4]); zeros([4,4]), zeros([4,4]), zeros([4,4]), fx, zeros([4,4]); zeros([4,4]), zeros([4,4]), zeros([4,4]), zeros([4,4]), fx];
Fu = 1/0.1396*eye(N);   % 8 deg max elevator deflection
bbx = ones([4,1]);
bx = ones([4*N,1]);
bu = ones([N,1]);
%% Matrices

% for i=1:4:4*N
%     A_c(i:i+length(A)-1,1:length(A)) = A^i;
%     for j=1:N
%         if (((i-1)/4)-j+1)>=0
%             B_c(i:i+length(A)-1,j) = A^(((i-1)/4)-j+1)*B;
%         else
%             B_c(i:i+length(A)-1,j) = 0;
%         end
%     end
% end
A_c = [A;A^2;A^3;A^4;A^5];
B_c = [B zeros([4,1]) zeros([4,1]) zeros([4,1]) zeros([4,1]); A*B B zeros([4,1]) zeros([4,1]) zeros([4,1]); A^2*B A*B B zeros([4,1]) zeros([4,1]); A^3*B A^2*B A*B B zeros([4,1]); A^4*B A^3*B A^2*B A*B B];

Q = [0.5 1.5 0 0; 1.5 0.5 0 0; 0 0 0 0; 0 0 0 0];
R = 1;
Q_c = [Q zeros([4,4]) zeros([4,4]) zeros([4,4]) zeros([4,4]); zeros([4,4]) Q zeros([4,4]) zeros([4,4]) zeros([4,4]); zeros([4,4]) zeros([4,4]) Q zeros([4,4]) zeros([4,4]); zeros([4,4]) zeros([4,4]) zeros([4,4]) Q zeros([4,4]); zeros([4,4]) zeros([4,4]) zeros([4,4]) zeros([4,4]) Q];
R_c = R*eye(N);
M = [Fu; Fx*B_c];
L = [bu;(bx-Fx*A_c*xk)];

%% Optimization
U_opt = -inv(B_c'*Q_c*B_c + R_c)*(B_c'*Q_c*A_c*xk + B_c'*Q_c*xr - R_c'*ur + 0.5*M'*lambda);
X = A_c*xk + B_c*U_opt;