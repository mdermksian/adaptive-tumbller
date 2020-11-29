clear all;
close all;
clc;

% System parameters
m1 = 0.5;
m2 = 0.3;
k1 = 5;
k2 = 10;
c1 = 0.1;
c2 = 0.3;
b = 0.1;

params_true = [m1 m2 k1 k2 c1 c2 b].';
n_params = length(params_true);

A = [zeros(2) eye(2); ...
    (-k1-k2)/m1  k2/m1 (-c1-c2-b)/m1      c2/m1; ...
          k2/m2 -k2/m2         c2/m2 (-c2-b)/m2];
B = [0 0 1/m1 0].';
n_states = size(A, 1);

T = 0.01;

theta_true = [A(3,:) A(4,:) B(3)].';
n_theta = length(theta_true);

% Initial parameter guesses
params0 = 2 * rand(n_params,1) .* params_true;
theta0 = genTheta(params0);

% Initial update matrix
P0 = 1e8 * eye(n_theta);

% Forgetting factor
lambda = 0.99;

% Low-pass filter for parameters (DOESNT WORK)
w = 1000;
G = tf(w, [1 w]);
Gd = c2d(G, T, 'foh');

% Initial conditions
x10 = 0;
x20 = x10;
dx10 = 0;
dx20 = 0;
z0 = [x10 x20 dx10 dx20].';

%% Simulation
Fmag = 3;
d = 0.5;

t0 = 0;
tend = 30;
maxstep = 1e-3;
tol = 1e-6;

simdata = sim('MSD2_RLS.slx');
tsim = simdata.tout;
zsim = simdata.zsim;
usim = simdata.usim;

tspl = simdata.tsim_spl;
zspl = simdata.zsim_spl;
uspl = simdata.usim_spl;

zhspl = simdata.zhsim;

thetaspl = simdata.theta;
thetaN = thetaspl(end,:).';

paramsim = simdata.params_sim;
theta_true_sim = genTheta(paramsim.').';

disp(theta0.');
disp(thetaN.');
disp(theta_true.');

figure();
subplot(2, 1, 1);
hold on;
plot(tspl, zspl(:,1), '-k');
% plot(tspl, zhspl(:,1), '-r');
ylabel('x_1 (m)');
subplot(2, 1, 2);
hold on;
plot(tspl, zspl(:,2), '-k');
% plot(tspl, zhspl(:,2), '-r');
ylabel('x_2 (m)');
xlabel('Time (s)');

figure();
for i = 1 : 9
    subplot(3, 3, i);
    hold on;
    plot(tspl, thetaspl(:,i), '-k');
    plot(tsim, theta_true_sim(:,i), '--b');
    title(strcat('\theta_', num2str(i)));
end


%% Functions
function[theta] = genTheta(params)
    N = size(params, 2);

    m1 = params(1, :);
    m2 = params(2, :);
    k1 = params(3, :);
    k2 = params(4, :);
    c1 = params(5, :);
    c2 = params(6, :);
    b = params(7, :);
    
    theta = zeros(9, N);
    theta(1, :) = (-k1-k2)./m1;
    theta(2, :) = k2./m1;
    theta(3, :) = (-c1-c2-b)./m1;
    theta(4, :) = c2./m1;
    theta(5, :) = k2./m2;
    theta(6, :) = -k2./m2;
    theta(7, :) = c2./m2;
    theta(8, :) = (-c2-b)./m2;
    theta(9, :) = 1./m1;
end
