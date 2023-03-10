function de = ssmodel2(t,e)

A = [ -0.0507   -3.861      0    -32.2
      -0.00117  -0.5164     1       0
      -0.000129  1.4168  -0.4932    0
          0        0        1       0];

C = [0 0 1 0];

op1 = -0.1;
op2 = -0.421;
op3 = -0.587;
op4 = -1;

L = place(A',C',[op1 op2 op3 op4])';

de = (A-L*C)*e;