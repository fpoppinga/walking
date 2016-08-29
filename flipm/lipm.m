%% init
close all;
clear all;

%% parameters
global g;
global h;
global maxStep;

maxStep = 0.15;

g = 9.81;
h = 0.25;
m = 5;
Ts = 0.01;

%% model

A = [0      1   
     g/h    0];
B = [0 1/(m*h)]';

C = [1 0];

[Phi, Gamma, C, D] = ssdata(c2d(ss(A, B, C, 0), Ts, 'zoh'));

%% observer
Qe = eye(2);
Re = eye(1);
L = dlqr(Phi', C', Qe, Re)';

%% simulation - the cycle.
kMax = 1000;
x = zeros(2, kMax);
u = zeros(1, kMax);
y = zeros(1, kMax);
energy = zeros(1, kMax);
safetyMargin = 0.3;

x(:, 1) = [0 0.5]';

% state estimation
xe = zeros(2, kMax);
xe(:, 1) = 0.9 * x(:, 1);
ye = zeros(1, kMax);

for k = 1:kMax 
   x(:, k + 1) = Phi * x(:, k) + Gamma * u(k);
   y(k) = C * x(:,k);
   
   ye(k) = C * xe(:, k);
   xe(:, k + 1) = Phi * xe(:, k) + L * (y(k) - ye(k)) + Gamma * u(k);
   
   
   if k == 200 
      x(2, k + 1) = x(2, k + 1) + 0.8;
   end
   
   % The safetyMargin needs to be applied, because otherwise the robot will
   % not be able to stop ever again, once the the calculated capture Step is
   % bigger then the maxStep
   if xe(1, k) >= maxStep/2 * safetyMargin
      % capture point - this position will stop the robot in one step.
      xStep = desiredVel(xe(2, k+1), 0.1);
      
      % the step can only be performed, if it is not restricted by
      % the hardware limit. 
      if abs(xStep - xe(1, k + 1)) > maxStep
        x(1, k + 1) = x(1, k + 1) - maxStep;
        xe(1, k + 1) = xe(1, k + 1) - maxStep;
      else
        x(1, k + 1) = xStep;
        xe(1, k + 1) = xStep;
      end
   end
   
   energy(k) = 0.5 * (x(2, k)^2 - g/h * x(1, k)^2);
end

figure();
stairs(1:kMax+1, x(1, :), '-b', 'DisplayName', 'position');
hold on;
stairs(1:kMax+1, xe(1, :), '-g', 'DisplayName', 'position estimate');

stairs(1:kMax+1, x(2, :), '-r', 'DisplayName', 'velocity');
stairs(1:kMax+1, xe(2, :), '-black', 'DisplayName', 'velocity estimate');
plot(1:kMax, energy, 'DisplayName', 'energy')
grid on;
legend('show');

