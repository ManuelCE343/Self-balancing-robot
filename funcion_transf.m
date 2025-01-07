M = 0; %masa del carro
m = 0.220;%masa del pendulo 
l = 0.1; % longitud al centro de masa del péndulo
b = 0.1; %coeficiente de fricción para el carro 0.1 N / m / seg
I = (1/3*m)*l^2; %momento de inercia de masa del péndulo
g = 9.8; %gravedad 
q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [P_cart ; P_pend];

%{
 inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
%set(sys_tf,'OutputName',outputs)
%}

Gp=sys_tf(2)
%gc=sys_tf(1)

Gc = pidtune(Gp,'PID')
%GD = pidtune(gc,'PID')
pidTuner(Gp,Gc)
%pidTuner(gc,GD)