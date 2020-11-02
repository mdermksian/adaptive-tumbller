clear all;
close all force;
clc;

% Mechanical parameters
mc = 0.493;     % cart mass (kg)
mp = 0.312;     % pendulum mass (kg)
Ip = 0.00024;   % pendulum rotational inertia (kg-m^2)
L = 0.04;       % penulum COM (m)
f = 0.01;       % damping (N-s/m)
g = 9.81;       % gravitational constant (m/s^2)

% Motor parameters
k_T = 0.11;         % N-s/A
R = 10;             % ohm
r = 0.0335;         % m

s_denom = Ip*(mc+mp) + mc*mp*L^2;
A = [0,                      1,                      0, 0; ...
     0, -(Ip+mp*L^2)*f/s_denom,     g*mp^2*L^2/s_denom, 0; ...
     0,                      0,                      0, 1; ...
     0,        -mp*L*f/s_denom, mp*g*L*(mc+mp)/s_denom, 0];
B = [0; (Ip+mp*L^2)/s_denom; 0; mp*L/s_denom];
C = eye(4);
D = zeros(4, 1);

Q = diag([100, 1, 1000, 1]);
R = 1;
K = lqr(A, B, Q, R);

x0 = 1;
dx0 = 0;
theta0 = -pi/6;
dtheta0 = 0;
init_state = [x0; dx0; theta0; dtheta0];

xr = 0;
dxr = 0;
thetar = 0;
dthetar = 0;
ref = [xr; dxr; thetar; dthetar];

tf = 5;
step = 1e-3;
tol = 1e-6;

sdata = sim('Nonlinear.slx');

t = sdata.tout;
y_lin = sdata.lin_out;
y_nl = sdata.nl_out;

animate_2D(t, y_nl, 0.005);
