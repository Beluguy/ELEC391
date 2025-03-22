M = .427;       % mass of the cart (total = 1.55 kg)
m = 1.09;       % mass of the pendulum   
b = 0.1;        % coefficient of friction for cart
I = 0.1;       % length to pendulum center of mass
g = 9.81;       % gravity
l = m*I^2;      % mass moment of inertia of the pendulum

p = I*(M+m)+M*m*l^2; %denominator for the A and B matrices

A = [0      1              0           0;
     0 -(I+m*l^2)*b/p  (m^2*g*l^2)/p   0;
     0      0              0           1;
     0 -(m*l*b)/p       m*g*l*(M+m)/p  0];

B = [     0;
     (I+m*l^2)/p;
          0;
        m*l/p];

C = [1 0 0 0;
     0 0 1 0];

D = [0;
     0];

states = {'x' 'x_dot' 'phi' 'phi_dot'};
inputs = {'u'};
outputs = {'phi'};

sys_phi = ss(A,B,C,D);

% Tune PID for the angle
c = pidtune(sys_phi, 'PID');

% Display gains
disp('PID Gains:');
disp(['Kp = ', num2str(c.Kp)]);
disp(['Ki = ', num2str(c.Ki)]);
disp(['Kd = ', num2str(c.Kd)]);

% Simulate step response
closed_loop = feedback(c * sys_phi, 1);

% Plot results
figure;
plot(t, y);
xlabel('Time (s)');
ylabel('Angle (rad)');
title('Step Response: Tracking Reference Angle');
grid on;