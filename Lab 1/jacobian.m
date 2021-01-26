%this code was used to calculate the Jacobian, J = dx/dq
syms a b c d;

T1 = [cos(a), -sin(a), 0, 0;
      sin(a), cos(a), 0, 0;
      0, 0, 1, 0;
      0, 0, 0, 1];
T2 = [cos(b), -sin(b), 0, 0;
      0, 0, -1, 0;
      sin(b), cos(b), 0, 0;
      0, 0, 0, 1];
T3 = [cos(c), -sin(c), 0, 12;
      0, 0, -1, -6;
      sin(c), cos(c), 0, 0;
      0, 0, 0, 1];  
T4 = [cos(d), -sin(d), 0, 0;
      0, 0, -1, 0;
      sin(d), cos(d), 0, 0;
      0, 0, 0, 1];
T5 = [1, 0, 0, 0;
      0, 1, 0, 0;
      0, 0, 1, 9;
      0, 0, 0, 1];

  
 y = T1*T2*T3*T4*T5;
 z = y*[0,0,0,1]';
 jacobian(z,[a,b,c,d])

