%lab1

R = zeros(4,4);
L2 = 12;
L3 = 6;
L4 = 9;
R01 = zeros(4,4);
R12 = zeros(4,4);
R23 = zeros(4,4);
R34 = zeros(4,4);
R45 = zeros(4,4);
thetas = [0,90,0,0,0];
alphas = [0,90,90,90,0];
ds = [0,0,L3,0,L4];
as = [0,0,L2,0,0];

for n = 1:1:5
	R(1,1) = cosd(thetas(n));
	R(1,2) = -1*sind(thetas(n));
	R(1,3) = 0;
	R(1,4) = as(n);
	R(2,1) = sind(thetas(n))*cosd(alphas(n));
	R(2,2) = cosd(thetas(n))*cosd(alphas(n));
	R(2,3) = -1*sind(alphas(n));
	R(2,4) = -1*ds(n)*sind(alphas(n));
	R(3,1) = sind(thetas(n))*sind(alphas(n));
	R(3,2) = cosd(thetas(n))*sind(alphas(n));
	R(3,3) = cosd(alphas(n));
	R(3,4) = ds(n)*cosd(alphas(n));
	R(4,1) = 0;
	R(4,2) = 0;
	R(4,3) = 0;
	R(4,4) = 1;

	if n == 1
		R01 = R;
	elseif n == 2
		R12 = R;
	elseif n == 3
		R23 = R;
	elseif n == 4
		R34 = R;
	elseif n == 5
		R45 = R;
    end
end

res = R01*R12*R23*R34*R45





%jacobian
syms a b c e;
R = zeros(4,4);
R1 = [cos(a), -1*sin(a), 0, 0;
	sin(a)*cosd(0), cos(a)*cosd(0), -1*sind(0), 0;
	sin(a)*sind(0), cos(a)*sind(0), 1*cosd(0), 0;
	0,0,0,1];
R2 = [cos(b), -1*sin(b), 0, 0;
	sin(b)*cosd(90), cos(b)*cosd(90), -1*sind(90), 0;
	sin(b)*sind(90), cos(b)*sind(90), 1*cosd(90), 0;
	0,0,0,1];
R3 = [cos(c), -1*sin(c), 0, L2;
	sin(c)*cosd(90), cos(c)*cosd(90), -1*sind(90), -1*L3*sind(90);
	sin(c)*sind(90), cos(c)*sind(90), 1*cosd(90), L3*cosd(90);
	0,0,0,1];
R4 = [cos(e), -1*sin(e), 0, 0;
	sin(e)*cosd(90), cos(e)*cosd(90), -1*sind(90), 0;
	sin(e)*sind(90), cos(e)*sind(90), 1*cosd(90), 0;
	0,0,0,1];
R5 = [cos(0), -1*sin(0), 0, 0;
	sin(0)*cosd(0), cos(0)*cosd(0), -1*sind(0), -1*L4*sind(0);
	sin(0)*sind(0), cos(0)*sind(0), 1*cosd(0), L4*cosd(0);
	0,0,0,1];
R = R1*R2*R3*R4*R5*[0,0,0,1]'


%{
while the magnitude of the error is large
while norm(xd - x0) > .1
    %calculate delta x
    dx = xd - x0;
    diffx = dx/N;
    
end
}%



JJ = [-9*sind(q(0))*cos(q(1))*sind(q(2)) - 9*cosd(q(0))*cosd(q(2)) - 12*sind(q(0))*cosd(q(1))

         - 6 sind(q(0))*sind(q(1)) ,

        -9*cosd(q(0))*sind(q(1))*sind(q(2)) - 12*cosd(q(0))*sind(q(1)) + 6 cosd(q(0))*cosd(q(1)) ,

        9*cosd(q(0))*cosd(q(1))*cosd(q(2)) + 9*sind(q(0))*sind(q(2)) , 0;

        9*cosd(q(0))*cosd(q(1))*sind(q(2)) - 9*sind(q(0))*cosd(q(2)) + 12*cosd(q(0))*cosd(q(1))

         + 6 cosd(q(0))*sind(q(1)) ,

        -9*sind(q(0))*sind(q(1))*sind(q(2)) - 12*sind(q(0))*sind(q(1)) + 6 sind(q(0))*cosd(q(1)) ,

        9*sind(q(0))*cosd(q(1))*cosd(q(2)) - 9*cosd(q(0))*sind(q(2)) , 0;

        0 , 9*cosd(q(1))*sind(q(2)) + 12*cosd(q(1)) + 6 sind(q(1)) , 9*sind(q(1))*cosd(q(2)) , 0;

        0 , 0 , 0 , 0]

JJ = [-9*sind(q(0))*cos(q(1))*sind(q(2)) - 9*cosd(q(0))*cosd(q(2)) - 12*sind(q(0))*cosd(q(1)) - 6 sind(q(0))*sind(q(1)), -9*cosd(q(0))*sind(q(1))*sind(q(2)) - 12*cosd(q(0))*sind(q(1)) + 6 cosd(q(0))*cosd(q(1)), 9*cosd(q(0))*cosd(q(1))*cosd(q(2)) + 9*sind(q(0))*sind(q(2)) , 0; 
9*cosd(q(0))*cosd(q(1))*sind(q(2)) - 9*sind(q(0))*cosd(q(2)) + 12*cosd(q(0))*cosd(q(1)) + 6 cosd(q(0))*sind(q(1)), -9*sind(q(0))*sind(q(1))*sind(q(2)) - 12*sind(q(0))*sind(q(1)) + 6 sind(q(0))*cosd(q(1)), 9*sind(q(0))*cosd(q(1))*cosd(q(2)) - 9*cosd(q(0))*sind(q(2)) , 0; 
0 , 9*cosd(q(1))*sind(q(2)) + 12*cosd(q(1)) + 6 sind(q(1)) , 9*sind(q(1))*cosd(q(2)) , 0;
0 , 0 , 0 , 0];

JJ = [-9*sind(q(1))*cos(q(2))*sind(q(3)) - 9*cosd(q(1))*cosd(q(3)) - 12*sind(q(1))*cosd(q(2)) - 6 sind(q(1))*sind(q(2)), -9*cosd(q(1))*sind(q(2))*sind(q(3)) - 12*cosd(q(1))*sind(q(2)) + 6 cosd(q(1))*cosd(q(2)), 9*cosd(q(1))*cosd(q(2))*cosd(q(3)) + 9*sind(q(1))*sind(q(3)) , 0; 
        9*cosd(q(1))*cosd(q(2))*sind(q(3)) - 9*sind(q(1))*cosd(q(3)) + 12*cosd(q(1))*cosd(q(2)) + 6 cosd(q(1))*sind(q(2)), -9*sind(q(1))*sind(q(2))*sind(q(3)) - 12*sind(q(1))*sind(q(2)) + 6 sind(q(1))*cosd(q(2)), 9*sind(q(1))*cosd(q(2))*cosd(q(3)) - 9*cosd(q(1))*sind(q(3)) , 0; 
        0 , 9*cosd(q(2))*sind(q(3)) + 12*cosd(q(2)) + 6 sind(q(2)) , 9*sind(q(2))*cosd(q(3)) , 0; 
        0 , 0 , 0 , 0];

        JJ = [-9*sind(q(1))*cos(q(2))*sind(q(3)) - 9*cosd(q(1))*cosd(q(3)) - 12*sind(q(1))*cosd(q(2)) - 6*sind(q(1))*sind(q(2)), -9*cosd(q(1))*sind(q(2))*sind(q(3)) - 12*cosd(q(1))*sind(q(2)) + 6*cosd(q(1))*cosd(q(2)), 9*cosd(q(1))*cosd(q(2))*cosd(q(3)) + 9*sind(q(1))*sind(q(3)), 0; 
        9*cosd(q(1))*cosd(q(2))*sind(q(3)) - 9*sind(q(1))*cosd(q(3)) + 12*cosd(q(1))*cosd(q(2)) + 6*cosd(q(1))*sind(q(2)), -9*sind(q(1))*sind(q(2))*sind(q(3)) - 12*sind(q(1))*sind(q(2)) + 6*sind(q(1))*cosd(q(2)), 9*sind(q(1))*cosd(q(2))*cosd(q(3)) - 9*cosd(q(1))*sind(q(3)), 0; 
        0 , 9*cosd(q(2))*sind(q(3)) + 12*cosd(q(2)) + 6*sind(q(2)) , 9*sind(q(2))*cosd(q(3)), 0; 
        0 , 0 , 0 , 0];

        