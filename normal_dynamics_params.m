%% Parameters for the normal dynamics of the robot
% These are in a separate file because it's convenient to be able to just
% call via "params  = normal_dynamics_params()"
function params = normal_dynamics_params()
l = 0.04;       % m
mp = 0.312;     % kg
mc = 0.493;     % kg
Ip = 2.4e-4;    % kg.m^2
Ic = 0;         % useless...
g = 9.81;       % m/s/s
f = 0.01;       % N.s/m

params = [l, mp, mc, Ip, Ic, g, f];
end