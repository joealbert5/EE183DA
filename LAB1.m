theta = [-90,90,0,0];

R1 = matrix(0,0,0,theta(1));
R2 = matrix(90,0,0,theta(2));
R3 = matrix(90,12,6,theta(3));
R4 = matrix(90,0,0,theta(4));
R5 = matrix(0,0,9,0);

T = R1*R2*R3*R4*R5;

x = [0,0,0,1];
y = T*x';
