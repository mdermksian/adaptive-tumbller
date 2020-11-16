%% Cart-Pendulum Dynamics with an Offset Mass
% 24774 ACSI Project

%% Set up problem
clear all
close all

vee = @(mat) [mat(1,3); mat(2,3); mat(2,1)];
adj = @(g) [[g(1,1) g(1,2);g(2,1) g(2,2)] [g(2,3);-g(1,3)]; zeros(1,2) 1];

%% Compute the forward kinematics
syms x dx ddx psi dpsi ddpsi l po pu mp md mc Ip Id Ic F g f real;

g_ws = [1 0 x;
        0 1 0;
        0 0 1];

g_s1 = [cos(psi) -sin(psi) 0;
        sin(psi) cos(psi) 0;
        0   0   1];
        
g_12 = [1 0 0;
        0 1 l;
        0 0 1];
        
g_13 = [1 0 po;
        0 1 pu;
        0 0 1];
        
g_w2 = g_ws*g_s1*g_12;
g_w3 = g_ws*g_s1*g_13;

% Body jacobians
Jbws = simplify([vee(inv(g_ws)*diff(g_ws,x)), vee(inv(g_ws)*diff(g_ws,psi))]);
Jbw2 = simplify([vee(inv(g_w2)*diff(g_w2,x)), vee(inv(g_w2)*diff(g_w2,psi))]);
Jbw3 = simplify([vee(inv(g_w3)*diff(g_w3,x)), vee(inv(g_w3)*diff(g_w3,psi))]);

%% Compute the mass matrix M
M1 = [mc 0 0;
      0 mc 0;
      0 0 Ic];
M2 = [mp 0 0;
      0 mp 0;
      0 0 Ip];
M3 = [md 0 0;
      0 md 0;
      0 0 Id];
  
M = simplify(Jbws.'*M1*Jbws + Jbw2.'*M2*Jbw2 + Jbw3.'*M3*Jbw3);

%% Compute the coriolis matrix C
C = sym(zeros(2,2));
q = [x; psi];
dq = [dx; dpsi];
ddq = [ddx; ddpsi];
for i = 1:2
    for j = 1:2
        for k = 1:2
            C(i,j) = C(i,j) + 0.5*(diff(M(i,j), q(k)) + diff(M(i,k), q(j)) - diff(M(k,j), q(i)))*dq(k);
        end
    end
end

C = simplify(C);

%% Compute the nonlinear terms matrix N
V = mp*g*g_w2(2,3) + md*g*g_w3(2,3);

N = simplify([diff(V,x); diff(V,psi)]);
N = N + [f*dx; 0];

%% Compute the final dynamic equations of motion with input Y
Y = [F; 0];

eqns = simplify(Y == M*ddq + C*dq + N);

dd_eqns = simplify(inv(M)*(Y-C*dq-N));

%% Output dynamics to a file using matlabFunction

output_to_file = 0; % Set to 1 if you'd like to write to file

if output_to_file
    matlabFunction(dd_eqns, 'File', 'offsetdyn_fun', 'Vars', {[x, dx, psi, dpsi], ...
        F, [l po pu mp md mc Ip Id Ic g f]}); % Function will take params (states, input, params)
end

%% Linearize EOMs
nonlin_ss = [dx; dd_eqns(1); dpsi; dd_eqns(2)];
states = [x dx psi dpsi];
pts = solve(subs(nonlin_ss == 0, F, 0), states);

A = jacobian(nonlin_ss, states);
B = jacobian(nonlin_ss, F);
A = simplify(A);
B = simplify(B);

% Substitute real values
sym_params = [l po pu mp md mc Ip Id Ic g f];
[params, motorparams] = offset_dynamics_params();
psi_eq = acot((params(4)*params(1)/(params(5)*params(2)))+(params(3)/params(2)));
eq = [0, 0, psi_eq, 0];

A_sys = subs(A, [x, dx, psi, dpsi, F, sym_params], [eq, 0, params]);
B_sys = subs(B, [x, dx, psi, dpsi, F, sym_params], [eq, 0, params]);
A_sys = double(A_sys);
B_sys = double(B_sys);
B_sys = B_sys*(2*motorparams(1))/(motorparams(2)*motorparams(3));
C = eye(4);
D = zeros(4, 1);

% Create linear SS system and design controller
ss_sys = ss(A_sys, B_sys, C, D);
Q = 1000*eye(4);
R = 1;

K = lqr(A_sys, B_sys, Q, R);

save("offset_dyn_controller", "K", "eq");
