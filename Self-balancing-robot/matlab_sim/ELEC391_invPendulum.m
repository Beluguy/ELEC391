M = .427;       % mass of the cart (total = 1.55 kg)
m = 1.09;       % mass of the pendulum   
b = 0.3;        % coefficient of friction for cart
I = 0.06;       % length to pendulum center of mass
g = 9.81;       % gravity
l = m*I^2;      % mass moment of inertia of the pendulum

q = (M+m)*(I+m*l^2)-(m*l)^2;
s = tf('s');

P_cart = (((I+m*l^2)/q)*s^2 - (m*g*l/q))/(s^4 + (b*(I + m*l^2))*s^3/q - ((M + m)*m*g*l)*s^2/q - b*m*g*l*s/q);

P_pend = (m*l*s/q)/(s^3 + (b*(I + m*l^2))*s^2/q - ((M + m)*m*g*l)*s/q - b*m*g*l/q);

sys_tf = [P_cart ; P_pend];

inputs = {'u'};
outputs = {'x'; 'phi'};

set(sys_tf,'InputName',inputs)
set(sys_tf,'OutputName',outputs)

sys_tf;

Kp = 100;
Ki = 0;
Kd = 10;

C = pid(Kp, Ki, Kd);
T = feedback(P_pend,C);

t=0:0.01:10;
impulse(T,t)
%axis([0, 1, -0.2, 0.2]);
title({'Response of Pendulum Position to an Impulse Disturbance';'under PID Control: Kp = 1, Ki = 1, Kd = 1'});
