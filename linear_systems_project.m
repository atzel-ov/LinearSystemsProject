%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear;
close all;
clc;


A = [ -0.0507   -3.861      0    -32.2
      -0.00117  -0.5164     1       0
      -0.000129  1.4168  -0.4932    0
          0        0        1       0];

B = [ 0
     -0.0717
     -1.645
      0];

C = [0 0 1 0];


sys = ss(A,B,C,0);



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------PART A------------%
% Meleth dynamikou systhmatos--%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%a

[V,poles] = eig(A);

x0 = [  0       %x0 = (0,+-0.1,0,0)
        -0.1
        0
        0];

[t,x] = ode45(@ssmodel1,[0 10],x0);


figure

subplot(2,2,1)
plot(t,x(:,1))
xlabel('Time (Seconds)')
ylabel('Velocity Difference')

subplot(2,2,2)
plot(t,x(:,2))
xlabel('Time (Seconds)')
ylabel('Angle of Attack')

subplot(2,2,3)
plot(t,x(:,3))
xlabel('Time (Seconds)')
ylabel('Pitch Rate')

subplot(2,2,4)
plot(t,x(:,4))
xlabel('Time (Seconds)')
ylabel('Pitch')


%b - %c

% For the period 0->T we are goind to apply a step input and then
% observe the system's time response

x0 = [  0
        0
        0
        0];

[t,x] = ode45(@ssmodel1,[0 10],x0);

%Run the same section after choosing the corresponding input from
%ssmodel1.m


figure

subplot(2,2,1)
plot(t,x(:,1))
xlabel('Time (Seconds)')
ylabel('Velocity Difference')

subplot(2,2,2)
plot(t,x(:,2))
xlabel('Time (Seconds)')
ylabel('Angle of Attack')

subplot(2,2,3)
plot(t,x(:,3))
xlabel('Time (Seconds)')
ylabel('Pitch Rate')

subplot(2,2,4)
plot(t,x(:,4))
xlabel('Time (Seconds)')
ylabel('Pitch')


%e

rank(ctrb(A,B))

%z

rank(obsv(A,C))



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%------------PART B------------%
% Controller & Observer design-%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%a

% Applying for the control input u the full state feedback :
% u = Fx + r, F = -K, x' = (A-B*K)x + Br = (Ac)x + Br
% We know the eigenvalues of Ac.

p1 = -1.25 + (2.2651)*1i;
p2 = -1.25 - (2.2651)*1i;
p3 = -0.01 + (0.095)*1i;
p4 = -0.01 - (0.095)*1i;

K = place(A,B,[p1 p2 p3 p4]);

sys_cl = ss(A-B*K,B,C,0);


%b

% After applying the full state feedback control law we get a new closed
% loop system x' = (A-B*K)x + Br, r = 0


x0 = [  0       %x0 = (0,+-0.1,0,0)
        -0.1
        0
        0];

[t,x] = ode45(@ssmodel1,[0 300],x0);


figure

subplot(2,2,1)
plot(t,x(:,1))
xlabel('Time (Seconds)')
ylabel('Velocity Difference')

subplot(2,2,2)
plot(t,x(:,2))
xlabel('Time (Seconds)')
ylabel('Angle of Attack')

subplot(2,2,3)
plot(t,x(:,3))
xlabel('Time (Seconds)')
ylabel('Pitch Rate')

subplot(2,2,4)
plot(t,x(:,4))
xlabel('Time (Seconds)')
ylabel('Pitch')


%c

% Switch between the inputs in ssmodel1.m

x0 = [  0
        0
        0
        0];

[t,x] = ode45(@ssmodel1,[0 300],x0);


figure

subplot(2,2,1)
plot(t,x(:,1))
xlabel('Time (Seconds)')
ylabel('Velocity Difference')

subplot(2,2,2)
plot(t,x(:,2))
xlabel('Time (Seconds)')
ylabel('Angle of Attack')

subplot(2,2,3)
plot(t,x(:,3))
xlabel('Time (Seconds)')
ylabel('Pitch Rate')

subplot(2,2,4)
plot(t,x(:,4))
xlabel('Time (Seconds)')
ylabel('Pitch')


%d

op1 = -0.1;
op2 = -0.421;
op3 = -0.587;
op4 = -1;

L = place(A',C',[op1 op2 op3 op4])';


%e

x0 = [  0       %x0 = (0,+-0.1,0,0)
        -0.1
        0
        0];

x0_hat = [  0.2       
            -0.1
            0.1
            -0.1];

e0 = x0 - x0_hat;

[t,e] = ode45(@ssmodel2,[0 50],e0);


figure

subplot(2,2,1)
plot(t,e(:,1))
xlabel('Time (Seconds)')
ylabel('Velocity Difference error')

subplot(2,2,2)
plot(t,e(:,2))
xlabel('Time (Seconds)')
ylabel('Angle of Attack error')

subplot(2,2,3)
plot(t,e(:,3))
xlabel('Time (Seconds)')
ylabel('Pitch Rate error')

subplot(2,2,4)
plot(t,e(:,4))
xlabel('Time (Seconds)')
ylabel('Pitch error')


%z


x0 = [  0       %x0 = (0,+-0.1,0,0)
        -0.1
        0
        0];

x0_hat = [  0.2       
            -0.1
            0.1
            -0.1];

[t,x] = ode45(@ssmodel3,[0 100],[x0 x0_hat]);


figure

subplot(2, 2, 1)
plot(t,x(:,1))
hold on
plot(t,x(:,5),'-r')
legend('x_1','x_1^h^a^t')
xlabel('Time (Seconds)')
ylabel('Velocity Difference')
grid on

subplot(2, 2, 2)
plot(t,x(:,2))
hold on
plot(t,x(:,6),'-r')
legend('x_2','x_2^h^a^t')
xlabel('Time (Seconds)')
ylabel('Angle of Attack')
grid on

subplot(2, 2, 3)
plot(t,x(:,3))
hold on
plot(t,x(:,7),'-r')
legend('x_3','x_3^h^a^t')
xlabel('Time (Seconds)')
ylabel('Pitch Rate')
grid on

subplot(2, 2, 4)
plot(t,x(:,4))
hold on
plot(t,x(:,8),'-r')
legend('x_4','x_4^h^a^t')
xlabel('Time (Seconds)')
ylabel('Pitch')
grid on


%h

At = [ A-B*K           B*K
       zeros(size(A))  A-L*C];

Bt = [ B
       zeros(size(B))];

Ct = [ C    zeros(size(C))];


sys_clob = ss(At,Bt,Ct,0);   % State space representation

systf = tf(sys_clob);        % Transfer Function

rank(ctrb(At,Bt))

%theta

% The states that matter the most in an aircraft are: the pitch rate q and
% the pitch θ since such aircrafts should be very accurate. Also the change
% of speed should be accurate enough and thus the angle of attack.
Q = [ 5  0  0  0       
      0  5 0  0
      0  0  50 0
      0  0  0  50];
  
% an f-16 aircraft may have expensive control but since its use is for
% military purposes lets assume that the cost is not neither high nor low
R = 1;                

K = lqr(A,B,Q,R);

sys_lqr = ss(A-B*K,B,C,0);


[optimalV,optimalPoles] = eig(A-B*K);


x0 = [  0       %x0 = (0,+-0.1,0,0)
        -0.1
        0
        0];

[t,x] = ode45(@ssmodel1,[0 10],x0);

figure
title('Closed-Loop Response to Non-Zero Initial Condition')

subplot(2,2,1)
plot(t,x(:,1))
xlabel('Time (Seconds)')
ylabel('Velocity Difference')

subplot(2,2,2)
plot(t,x(:,2))
xlabel('Time (Seconds)')
ylabel('Angle of Attack')

subplot(2,2,3)
plot(t,x(:,3))
xlabel('Time (Seconds)')
ylabel('Pitch Rate')

subplot(2,2,4)
plot(t,x(:,4))
xlabel('Time (Seconds)')
ylabel('Pitch')


% Comparison of pole placement and lqr cost minimisation
figure
step(-1*sys_cl)
title('Step Response using Pole Placement')

figure
step(-1*sys_lqr)
title('Step Response using LQR')