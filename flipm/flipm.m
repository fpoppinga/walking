m0 = 1;
m1 = 0.001;
b = 0.1;
k = 0.1;
g = 9.81;
z = 0.25;
omegaSquared = g / z;

G = tf([-1/omegaSquared 0 1], [1 0 0]) * tf([b k], [m0*m1 (m0+m1)*b (m0+m1)*k]);
step(G)