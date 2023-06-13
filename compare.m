clear; close all; clc;

t=0:0.01:5;

% Define the system transfer function
l=0.25; g=9.81; m=0.15; M=0.9;
J=1/3*m*(2*l)^2;
z=M*J+m*M*l^2+m*J;

%Model of Inverted Pendulum
A=[0 1 0 0;
    ((M+m)*m*g*l)/z 0 0 0;
    0 0 0 1;
    -m^2*l^2*g/z 0 0 0];
B=[0; -m*l/z; 0; (J+m*l^2)/z];
C=[1 0 0 0];
D=0;

[num, den]=ss2tf(A,B,C,D);
G=tf(num,den);

load('optimalValues.mat')

controller = pid(Kp, Ki, Kd, 0.03);
sysClosedLoop = feedback(controller * G, 1);
sysClosedLoopInput = feedback(controller, G);
figure(1);
yout=step(sysClosedLoop, t);
s1=stepinfo(yout, t);
plot(t,yout,'-r', 'LineWidth', 2);
title('Step Response of the System \theta(t)');
xlabel('time (s)');
ylabel('angle (rad)');
grid on; grid minor; hold on;
figure(2);
uout=step(sysClosedLoopInput, t);
plot(t,uout*0.01,'-k', 'LineWidth', 2); % 0.01 is radiu, T=rF
title('Input Signal of Plant u(t)');
xlabel('time (s)');
ylabel('torque (Nm)');
grid on; grid minor; hold on;

load('classicalValues.mat')
controller = pid(Kp, Ki, Kd, 0.01);
sysClosedLoop = feedback(controller * G, 1);
sysClosedLoopInput = feedback(controller, G);
figure(1);
yout=step(sysClosedLoop, t);
s2=stepinfo(yout, t);
plot(t,yout,'--b', 'LineWidth', 1);
title('Step Response of the System \theta(t)');
xlabel('time (s)');
ylabel('angle (rad)');
grid on; grid minor; hold on;
legend('ABC','Classical')
figure(2);
uout=step(sysClosedLoopInput, t);
plot(t,uout*0.01,'--g', 'LineWidth', 1); % 0.01 is radiu, T=rF
title('Input Signal of Plant u(t)');
xlabel('time (s)');
ylabel('torque (Nm)');
grid on; grid minor; hold on;
legend('ABC','Classical')