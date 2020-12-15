clear all;
close all;
clc;

run('OffsetNonlinear_setup.m');

n_theta = 6;
n_states = 4;

T = 0.015;

% Initial parameter guesses
% theta0 = zeros(6, 1);
theta0 = [-3.3899 -0.9880 -4.4196 -0.0958 0.4707 3.7728].';
theta_LS = [-3.3899 -0.9880 -4.4196 -0.0958 0.4707 3.7728].';

A0 = [0 1 0 0; ...
0 theta0(1:2).' 0; ...
0 0 0 1; ...
0 theta0(3:4).' 0];
B0 = [0 theta0(5) 0 theta0(6)].';

% Initial update matrix
P0 = 1e8 * eye(n_theta);

% Forgetting factor
% lambda = 1;
lambda = 0.995;

% Initial conditions
z0 = x0;

update_ctlr = 1;

% Q = diag([1, 1, 80000000, 30000]);
% R = 2;
Q = 1000*eye(4);
R = 1;
% K0 = lqr(A0, B0, Q, R);
K0 = lqr(A_sys, B_sys, Q, R);

eq = [0; 0; 0; 0];

%% Simulation
t0 = 0;
tend = 50;
maxstep = 1e-3;
tol = 1e-6;

simdata = sim('Tumbller_STR.slx');
tsim = simdata.tout;
zsim = simdata.zsim;
usim = simdata.usim;

tspl = simdata.tsim_spl;
zspl = simdata.zsim_spl;
uspl = simdata.usim_spl;

zhspl = simdata.zhsim;

thetaspl = simdata.theta;
thetaN = thetaspl(end,:).';

% disp(theta0.');
% disp(thetaN.');
% disp(theta_LS.');

figure();
subplot(2, 1, 1);
hold on;
plot(tspl, zspl(:,1), '-k');
% plot(tspl, zhspl(:,1), '-r');
ylabel('x (m)');
subplot(2, 1, 2);
hold on;
plot(tspl, zspl(:,3), '-k');
% plot(tspl, zhspl(:,3), '-r');
ylabel('\psi (rad)');
xlabel('Time (s)');

figure();
for i = 1 : 6
    subplot(2, 3, i);
    hold on;
    plot(tspl, thetaspl(:,i), '-k');
    plot([tsim(1) tsim(end)], theta_LS(i)*[1 1], '--b');
    title(strcat('\theta_', num2str(i)));
end


%% Least-Squares
Phi = [];
Delta = [];
Z = [];
for k = 2 : length(tspl)
    [phik, deltak] = linParamFunc(zspl(k-1,:).', uspl(k-1), T);
    Phi = [Phi; phik.'];
    Delta = [Delta; deltak];
    
    Z = [Z; zspl(k,:).'];
end

theta1 = (Phi.' * Phi) \ Phi.' * (Z - Delta);
disp(theta0.');
disp(theta1.');
disp(thetaN.');

E0 = Z - Phi*theta0 - Delta;
J0 = E0.' * E0 / 2
E1 = Z - Phi*theta1 - Delta;
J1 = E1.' * E1 / 2


%% Functions
function[phi, delta] = linParamFunc(z, u, T)
    phi = zeros(6, 4);
    phi(1:2, 2) = T * z(2:3);
    phi(5, 2) = T * u;
    phi(3:4, 4) = T * z(2:3);
    phi(6, 4) = T * u;
    
    delta = [z(1)+T*z(2); ...
                    z(2); ...
             z(3)+T*z(4); ...
                    z(4)];
end
