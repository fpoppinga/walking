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
kMax = 200;
x = zeros(2, kMax);
u = zeros(1, kMax);
y = zeros(1, kMax);
energy = zeros(1, kMax);


x(:, 1) = [0 0.01]';

stepLength = 0.2;
for k = 1:kMax
   x(:, k + 1) = Phi * x(:, k) + B * u(k);
   y(k) = C * x(:,k);
   
   if y(k) >= stepLength/2 
      % capture point - this position will stop the robot in one step.
      xc = -x(2, k + 1) * sqrt(h / g);
      
      if abs(xc - x(1, k + 1)) > stepLength
        x(1, k + 1) = x(1, k + 1) - stepLength;
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
stairs(1:kMax+1, x(1, :), '-b');
hold on;

stairs(1:kMax+1, x(2, :), '-r');
plot(1:kMax, energy)
grid on;


