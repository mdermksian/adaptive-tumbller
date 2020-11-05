%% Parameters for the offset dynamics of the robot
% These are in a separate file because it's convenient to be able to just
% call via "params  = offset_dynamics_params()"
function params = offset_dynamics_params()
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
end