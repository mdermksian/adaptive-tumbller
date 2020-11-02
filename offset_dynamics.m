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

dd_eqns = ddq == simplify(inv(M)*(Y-C*dq-N));