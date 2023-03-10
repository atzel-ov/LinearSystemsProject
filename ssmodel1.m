function dx = ssmodel1(t,x)   

A = [ -0.0507   -3.861      0    -32.2
      -0.00117  -0.5164     1       0
      -0.000129  1.4168  -0.4932    0
          0        0        1       0];
B = [ 0
     -0.0717
     -1.645
      0];


T = 2;

%PART A
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%a
%u = zeros(size(t)); 

%b1
%u = -((t>=0)-(t>=T));

%b2
%u = -(t>=0);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%PART B
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


p1 = -1.25 + (2.2651)*1i;
p2 = -1.25 - (2.2651)*1i;
p3 = -0.01 + (0.095)*1i;
p4 = -0.01 - (0.095)*1i;

K = place(A,B,[p1,p2,p3,p4]);

%b
%r = zeros(size(t));

%c1
%r = -1*((t>=0)-(t>=T));

%c2
%r = -(t>=0);

%u = -K*x + r;

%%%% LQR %%%%

Q = [ 5  0  0  0
      0  5 0  0
      0  0  50 0
      0  0  0  50];
  

R = 1;

Klqr = lqr(A,B,Q,R);

u = -Klqr*x;



dx = A*x + B*u;
