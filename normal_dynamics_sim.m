clear;
x0 = [0; 0; -pi/4; 0];
tspan = [0 10];
input = 0;
params = normal_dynamics_params();
load normal_dyn_controller.mat;

[t, xout] = ode45(@(t, x) odefun(t, x, K, params), tspan, x0);

plot(t, xout);
legend('x', 'dx', 'psi', 'dpsi');

animate_2D(t, xout);

function dxdt = odefun(t, x, K, params)
    input = -K*x;
    ddx = normaldyn_fun(x', input, params);
    dxdt = [x(2); ddx(1); x(4); ddx(2)];
end