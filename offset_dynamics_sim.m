clear;
x0 = [0; 0; pi; 0];
tspan = [0 10];

input = 0;

l = 0.04; % m
po = 0.04; % m
pu = 0.01; % m
mp = 0.312; % kg
md = 0.024; % kg
mc = 0.493; % kg
Ip = 0.00024; % kg.m^2
Id = md*(po^2+pu^2); % kg.m^2
Ic = 0; % useless...
g = 9.81; % m/s/s
f = 0.01; % N.s/m

params = [l po pu mp md mc Ip Id Ic g f];

[t, xout] = ode45(@(t, x) odefun(t, x, input, params), tspan, x0);

plot(t, xout);
legend('x', 'dx', 'psi', 'dpsi');

animate_2D(t, xout);

function dxdt = odefun(t, x, input, params)
    ddx = offsetdyn_fun(x', input, params);
    dxdt = [x(2); ddx(1); x(4); ddx(2)];
end