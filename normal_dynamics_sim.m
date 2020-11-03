clear;
x0 = [0; 0; 35*pi/36; 0];
tspan = [0 10];

input = 0;

l = 0.04; % m
mp = 0.312; % kg
mc = 0.493; % kg
Ip = 0.00024; % kg.m^2
Ic = 0; % useless...
g = 9.81; % m/s/s
f = 0.01; % N.s/m

params = [l mp mc Ip Ic g f];

[t, xout] = ode45(@(t, x) odefun(t, x, input, params), tspan, x0);

plot(t, xout);
legend('x', 'dx', 'psi', 'dpsi');

animate_2D(t, xout);

function dxdt = odefun(t, x, input, params)
    ddx = normaldyn_fun(x', input, params);
    dxdt = [x(2); ddx(1); x(4); ddx(2)];
end