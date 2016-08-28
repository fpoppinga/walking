%% init
close all;
clear all;

%% parameters
g = 9.81;
h = 0.25;
m = 5;
Ts = 0.01;

%% model

A = [0      1   
     g/h    0];
B = [0 1/(m*h)]';

C = [1 0];

[Phi, Gamma, C, D] = ssdata(c2d(ss(A, B, C, 0), Ts, 'zoh'))

%% simulation - the cycle.
kMax = 500;
x = zeros(2, kMax);
u = zeros(1, kMax);
y = zeros(1, kMax);
energy = zeros(1, kMax);
safetyMargin = 0.92;


x(:, 1) = [0 0.01]';

maxStep = 0.15;
for k = 1:kMax
   x(:, k + 1) = Phi * x(:, k) + B * u(k);
   y(k) = C * x(:,k);
   
   % The safetyMargin needs to be applied, because otherwise the robot will
   % not be able to stop ever again, once the the calculated capture Step is
   % bigger then the maxStep
   if y(k) >= maxStep/2 * safetyMargin
      % capture point - this position will stop the robot in one step.
      xc = -x(2, k + 1) * sqrt(h / g);
      
      % the capture step can only be performed, if it is not restricted by
      % the hardware limit.
      if abs(xc - x(1, k + 1)) > maxStep
        x(1, k + 1) = x(1, k + 1) - maxStep;
      else
        x(1, k + 1) = xc;  
      end
      
      % constant velocity:
      xv = -x(1, k + 1);
      
%       x(1, k + 1) = xv; 
   end
   
   energy(k) = 0.5 * (x(2, k)^2 - g/h * x(1, k)^2);
end

figure();
stairs(1:kMax+1, x(1, :), '-b', 'DisplayName', 'position');
hold on;

stairs(1:kMax+1, x(2, :), '-r', 'DisplayName', 'velocity');
plot(1:kMax, energy, 'DisplayName', 'energy')
grid on;
legend('show');

