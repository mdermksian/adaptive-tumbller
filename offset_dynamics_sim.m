clear;
params = offset_dynamics_params();
load offset_dyn_controller.mat;
psi_eq = acot((params(4)*params(1)/(params(5)*params(2)))+(params(3)/params(2)));
eq = [0; 0; psi_eq; 0];
x0 = [0; 0; -pi/12; 0];
tspan = [0 10];

[t, xout] = ode45(@(t, x) odefun(t, x, eq, K, params), tspan, x0);

plot(t, xout);
legend('x', 'dx', 'psi', 'dpsi');

animate_2D(t, xout);

function dxdt = odefun(t, x, eq, K, params)
input = -K*(x-eq);
ddx = offsetdyn_fun(x', input, params);
dxdt = [x(2); ddx(1); x(4); ddx(2)];
end