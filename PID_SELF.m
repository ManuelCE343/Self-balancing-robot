M = 0.01; %masa del carro
m = 0.220;%masa del pendulo 
l = 0.1; % longitud al centro de masa del péndulo
b = 0.1; %coeficiente de fricción para el carro 0.1 N / m / seg
I = (1/3*m)*l^2; %momento de inercia de masa del péndulo
g = 9.8; %gravedad 
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');


P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

Kp = 110;
Ki = 1;
Kd = 22;
C = pid(Kp,Ki,Kd);
T = feedback(P_pend,C);
t=0:0.01:10;
impulse(T,t)
axis([0, 2.5, -0.2, 0.2]);
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control'});